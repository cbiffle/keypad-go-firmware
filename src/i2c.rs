// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! I2C key event buffer driver.
//!
//! Plus STM32G0/STM32C0 generic target driver along the way.
//!
//! # Architecture
//!
//! The I2C driver receives fully processed 8-bit key event bytes from the
//! serial task. This centralizes key processing in one place (serial). Because
//! the serial task always has "permission" to transmit (it does not have to
//! coordinate with any external entity to send on the serial port), and thus
//! always has permission to transmit, that task can always process a key
//! promptly -- whereas the I2C task (hi!) only speaks when asked by the host.
//! So, the serial task drives and we listen.
//!
//! The queue coming from the serial task *is* the outstanding key buffer. We
//! pop from it only when the host asks us to, and otherwise leave it alone. If
//! multiple key events occur between I2C polls, they will pile up in that
//! queue.
//!
//! This driver is strangely factored, but the factoring has proven rather
//! powerful. I'm not sure I'd suggest this as the "right" way of writing an I2C
//! driver, but it has worked well in this case.
//!
//! The basic idea is outlined in:
//!
//! https://cliffle.com/blog/composed-concurrency-in-drivers/
//!
//! and consists of:
//!
//! - Describing the data motion part of I2C target as an async fn.
//! - Additionally describing error aborts, stop conditions, and repeated starts
//!   in another async fn.
//! - Merging them together using `select_biased!`.
//!
//! This has the advantage of ensuring that we didn't miss a check. For
//! instance, we can guarantee by simple inspection that the error state machine
//! will get a chance to speak up any time the data state machine yields and is
//! resumed, because of the ordering used in `select_biased!` and its
//! definition.
//!
//! # Operation of the hardware
//!
//! We run the I2C peripheral in individual byte control mode. This requires
//! more hand-holding than the more pipelined modes, but keeps us from having to
//! track additional state. It does imply slightly more clock stretching than
//! would be ideal, but in practice we don't stretch much.
//!
//! Before you attempt to switch the peripheral out of byte control mode to
//! reduce the clock stretching, keep this in mind: running it in any other mode
//! makes it a hard realtime interface, because the peripheral presents narrow
//! windows of time in which a response must be prepared before it goes ahead
//! and speaks nonsense -- and those windows often happen _one byte earlier_
//! than you would normally expect!

use core::convert::Infallible;
use core::sync::atomic::{AtomicU32, Ordering};
use futures::{select_biased, FutureExt as _};
use lilos::atomic::AtomicArithExt;
use lilos::exec::Notify;
use lilos::spsc;
use lilos::util::FutureExt;

use crate::device;

use device::gpio::vals::Moder;
use device::i2c::vals::{Addmode, Reload, Dir};
use device::interrupt;

/// Our address, expressed as a 7-bit binary number. This corresponds to the I2C
/// address byte with the read/write bit missing, and is (IMO) the least
/// ambiguous way of expressing such an address.
///
/// TODO: it'd be great for this to be online-programmable, stored in flash.
#[allow(clippy::unusual_byte_groupings)] // Deliberately written in 4_3
const ADDR7: u8 = 0b1100_101;

///////////////////////////////////////////////////////////////////////////////
// Event counters. These are written from the application but are intended for
// consumption by a debugger, so they appear (from inspection of the code, and
// from rustc) to never be read. Thus, each should carry the #[used] attribute
// to ensure it makes it into the firmware.

/// Number of bus errors observed.
#[used]
static ERR_BUS: AtomicU32 = AtomicU32::new(0);

/// Number of arbitration lost events.
#[used]
static ERR_ARLO: AtomicU32 = AtomicU32::new(0);

/// Number of times we detected our address.
#[used]
static ADDR_DETECT: AtomicU32 = AtomicU32::new(0);

/// Number of stop conditions we observed (only during our transactions).
#[used]
static STOPS: AtomicU32 = AtomicU32::new(0);

/// Number of times we were asked to transmit.
#[used]
static TXS: AtomicU32 = AtomicU32::new(0);

/// Number of times we were asked to receive.
#[used]
static RXS: AtomicU32 = AtomicU32::new(0);

///////////////////////////////////////////////////////////////////////////////
// The actual driver.

/// Collects bytes from the serial task and makes them available over I2C.
pub async fn task(
    rcc: device::rcc::Rcc,
    gpiob: device::gpio::Gpio,
    i2c: device::i2c::I2c,
    mut bytes_from_serial: spsc::Pop<'_, u8>,
) -> Infallible {
    init(rcc, i2c, gpiob);

    // Safety: this is ungating our interrupt, which is fine because we're not
    // relying on exclusion with this interrupt for safety -- all the ISR
    // (below) does is clear some peripheral flags (atomically) and poke a
    // Notify.
    //
    // Plus, lilos is configured to not allow interrupt preemption anyway.
    unsafe {
        cortex_m::peripheral::NVIC::unmask(device::Interrupt::I2C1);
    }

    // Transaction handling loop. We come up to the top of this whenever we're
    // idle, or if we're re-addressed during a transaction (repeated start).
    loop {
        // Wait until we're addressed.
        i2c.cr1().modify(|w| w.set_addrie(true));
        EVT.until(|| i2c.isr().read().addr()).await;

        ADDR_DETECT.fetch_add_polyfill(1, Ordering::Relaxed);

        // We don't want to clear ADDR immediately, because we need to set up
        // the first byte of the transaction first. Clearing ADDR ends our clock
        // stretch and, for a read operation, starts transmission of our first
        // response byte.

        i2c.cr2().modify(|w| {
            // We're going to move one byte.
            w.set_nbytes(1);
            // It won't be the last one.
            w.set_reload(Reload::NOTCOMPLETED);
        });
        // Clear ADDR flag, ending any clock stretching.
        i2c.icr().write(|w| w.set_addrcf(true));

        // Use select to compose the end-of-transaction monitor with the data
        // handler. If we get a stop condition, error, or repeated start, which
        // we must eventually, the data handler will be cancelled.
        select_biased! {
            result = terminal_condition(i2c).fuse() => {
                match result {
                    Ok(()) => {
                        STOPS.fetch_add_polyfill(1, Ordering::Relaxed);
                        // stop condition -- back to the top of the loop.
                    }
                    Err(Error) => {
                        // errors are counted individually in their respective
                        // detection routines. All we need to do here is let
                        // normal flow control take us back to the top of the
                        // loop.
                    }
                }
            }
            never = handle_data(i2c, &mut bytes_from_serial).fuse() => {
                // Idiom for handling the result from a function that can't
                // return, without the potential code size of an unreachable!
                // macro.
                match never {}
            }
        }
    }
}

/// The data handler. Expressed as an infinite loop, this routine relies on
/// cancellation of its future to end transmission.
async fn handle_data(
    i2c: device::i2c::I2c,
    bytes_from_serial: &mut spsc::Pop<'_, u8>,
) -> ! {
    // Determine the direction of the transfer.
    match i2c.isr().read().dir() {
        Dir::WRITE => receive_data(i2c).await,
        _ => transmit_data(i2c, bytes_from_serial).await,
    }
}

/// Processes an infinite stream of incoming data until it's cancelled.
///
/// This is required to process an _infinite_ stream because the length of the
/// stream is not under our control. Even after we NACK a byte, the host is free
/// to continue clocking us for all eternity if it wishes. It's not clear that
/// the STM32 I2C block lets us abort a transaction / start behaving as though
/// our address never matched, so to avoid stretching the clock, we have to keep
/// on NACKing bogus bytes.
async fn receive_data(i2c: device::i2c::I2c) -> ! {
    let mut nacking = false;

    loop {
        // Ensure we find out about any data that appears.
        i2c.cr1().modify(|w| w.set_rxie(true));

        // Wait for data, being sure to turn our interrupt enable back off if
        // we're cancelled. (The ISR will turn it off if we wake.)
        EVT.until(|| i2c.isr().read().rxne())
            .on_cancel(|| i2c.cr1().modify(|w| w.set_rxie(false)))
            .await;

        // Pull the data out.
        let byte = i2c.rxdr().read().rxdata();
        RXS.fetch_add_polyfill(1, Ordering::Relaxed);

        // Currently we tolerate the byte 0 being written, because a lot of I2C
        // libraries want to write a "register address" before reading. Since we
        // currently only do one thing, we allow a "register address" of zero to
        // mean "the key buffer."
        //
        // This does have the advantage of allowing us to add another thing
        // later at a different address, though.
        nacking |= byte != 0;

        i2c.cr2().modify(|w| {
            // Clear NACK flag so we ACK
            w.set_nack(nacking);
            // We're going to ACK/NACK after the next byte.
            w.set_nbytes(1);
            // It won't be the last one.
            w.set_reload(Reload::NOTCOMPLETED);
        });
    }
}

/// Generates an infinite stream of outgoing data until it's cancelled.
///
/// The rationale for the stream being infinite is the same as for
/// `receive_data` above: the host controls the duration of the transfer, not
/// us. So, we have to generate data forever if they ask us to. In this
/// direction we can't even NACK to express our displeasure. Instead, any
/// outgoing data transfer needs to naturally pad itself with ... something.
async fn transmit_data(
    i2c: device::i2c::I2c,
    bytes_from_serial: &mut spsc::Pop<'_, u8>,
) -> ! {
    loop {
        // Ensure we find out if we need to transmit another byte.
        i2c.cr1().modify(|w| w.set_txie(true));

        // Wait for events, turning that back off if we're cancelled. (The ISR
        // will turn it off if we wake.)
        EVT.until(|| i2c.isr().read().txis())
            .on_cancel(|| i2c.cr1().modify(|w| w.set_txie(false)))
            .await;

        // Send the next byte. If the host reads off the end of the key queue,
        // we'll pad with zeros.
        let byte = bytes_from_serial.try_pop().unwrap_or(0);
        i2c.txdr().write(|w| w.set_txdata(byte));
        TXS.fetch_add_polyfill(1, Ordering::Relaxed);

        // Wait for TCR event only, turning it back off if we're cancelled. (The
        // ISR will, again, take care of disabling TCIE if it wakes us.)
        i2c.cr1().modify(|w| w.set_tcie(true));
        EVT.until(|| i2c.isr().read().tcr())
            .on_cancel(|| i2c.cr1().modify(|w| w.set_tcie(false)))
            .await;

        i2c.cr2().modify(|w| {
            // We'll do one byte again.
            w.set_nbytes(1);
            // It won't be the last one.
            w.set_reload(Reload::NOTCOMPLETED);
        });
    }
}

/// Combined handler for terminal conditions: stop conditions, errors, and
/// repeated starts.
///
/// Our response to any of these conditions is the same, so we don't bother
/// expressing them as separate state machines to save space.
async fn terminal_condition(i2c: device::i2c::I2c) -> Result<(), Error> {
    // Ensure we'll get interrupts on the conditions we're monitoring.
    i2c.cr1().modify(|w| {
        w.set_addrie(true);
        w.set_stopie(true);
        w.set_errie(true);
    });

    // Make sure all our interrupt enables are clear no matter how we leave the
    // routine -- the ISR will have cleared one of them but probably not all.
    // (We likely won't be cancelled because of how the driver is constructed --
    // we do the cancelling, after all -- but it's still nice to do things the
    // right way.)
    scopeguard::defer! {
        i2c.cr1().modify(|w| {
            w.set_addrie(false);
            w.set_stopie(false);
            w.set_errie(false);
        });
    }

    // Wait for a matching interrupt and record what condition we found.
    let result = EVT.until(|| {
        let isr = i2c.isr().read();
        let berr = isr.berr();
        let arlo = isr.arlo();
        if berr {
            ERR_BUS.fetch_add_polyfill(1, Ordering::Relaxed);
        }
        if arlo {
            ERR_ARLO.fetch_add_polyfill(1, Ordering::Relaxed);
        }
        if berr || arlo {
            Some(Err(Error))
        } else if isr.stopf() || isr.addr() {
            Some(Ok(()))
        } else {
            None
        }
    }).await;

    // If we've detected our event, clear our flags for next time. Even though
    // we may have woken up in response to the ADDR flag (repeated start), do
    // NOT clear it here, as that has side effects! It will be processed in
    // the outer loop.
    i2c.icr().write(|w| {
        w.set_stopcf(true);
        w.set_berrcf(true);
        w.set_arlocf(true);
    });

    result
}

/// World's simplest error type.
#[derive(Copy, Clone, Debug)]
struct Error;

/// Sets up clocks, I2C, and pins. Factored out of driver task.
fn init(
    rcc: device::rcc::Rcc,
    i2c: device::i2c::I2c,
    gpiob: device::gpio::Gpio,
) {
    // Un-gate clock to our I2C block.
    rcc.apbenr1().modify(|w| w.set_i2c1en(false));
    cortex_m::asm::dsb();

    // Reference Manual 23.4.5

    // The I2C block comes out of reset with PE=0. For space reasons we'll skip
    // the portion of the init flow from the Reference Manual that's intended
    // for making sure you can safely re-initialize. We have no intention of
    // re-initializing (without a reset).
    
    // Leave analog and digital filters in reset configuration (analog filter
    // on, digital filter off). TODO: research and enable the digital filter.

    // TODO: configure SDADEL and SCLDEL after measuring this on scope - this
    // leaves the prescaler at 1x

    // Expose I2C pins on PB6/7. Note that these were already set to the correct
    // AF setting in main. (TODO: this factoring is questionable.)
    gpiob.moder().modify(|w| {
        w.set_moder(6, Moder::ALTERNATE);
        w.set_moder(7, Moder::ALTERNATE);
    });

    // Configure peripheral to respond to our address.
    i2c.oar1().write(|w| {
        w.set_oa1(u16::from(ADDR7 << 1));
        w.set_oa1mode(Addmode::BIT7);
        w.set_oa1en(true);
    });

    i2c.cr1().write(|w| {
        // Byte-level acking
        w.set_sbc(true);
        // Peripheral on
        w.set_pe(true);
    });
}

/// ISR.
///
/// This is responsible for
///
/// 1. Figuring out which *enabled* interrupt source was responsible for
///    generating the interrupt.
/// 2. Clearing its enable bit, but taking no action to stop it from being
///    active, which ensures that the task handler above will notice it.
/// 3. Poke the Notify to wake the task.
#[interrupt]
fn I2C1() {
    let i2c = device::I2C1;
    let cr1 = i2c.cr1().read();
    let isr = i2c.isr().read();

    let mut bits_to_clear = 0;
    if cr1.txie() && isr.txis() {
        bits_to_clear |= 1 << 1;
    }
    if cr1.rxie() && isr.rxne() {
        bits_to_clear |= 1 << 2;
    }
    if cr1.stopie() && isr.stopf() {
        bits_to_clear |= 1 << 5;
    }
    if cr1.addrie() && isr.addr() {
        bits_to_clear |= 1 << 3;
    }
    if cr1.tcie() && isr.tcr() {
        bits_to_clear |= 1 << 6;
    }
    if cr1.errie() && (isr.arlo() || isr.berr()) {
        bits_to_clear |= 1 << 7;
    }

    if bits_to_clear != 0 {
        i2c.cr1().modify(|w| {
            w.0 &= !bits_to_clear;
        });
        EVT.notify();
    }
}

static EVT: Notify = Notify::new();
