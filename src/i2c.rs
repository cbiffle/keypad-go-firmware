//! I2C key event buffer driver.

use core::convert::Infallible;
use core::sync::atomic::{AtomicU32, Ordering};

use futures::{select_biased, FutureExt};
use lilos::atomic::AtomicArithExt;
use lilos::exec::Notify;
use lilos::spsc;
use scopeguard::ScopeGuard;

use crate::device;
use device::interrupt;

// TODO check this for common conflicts
// TODO make programmable
const ADDR7: u8 = 0b1100_101;

/// Bus error counter, intended for consumption by a debugger.
#[used]
static ERR_BUS: AtomicU32 = AtomicU32::new(0);
/// Arbitration lost error counter, intended for consumption by a debugger.
#[used]
static ERR_ARLO: AtomicU32 = AtomicU32::new(0);

#[used]
static ADDR_DETECT: AtomicU32 = AtomicU32::new(0);
#[used]
static STOPS: AtomicU32 = AtomicU32::new(0);
#[used]
static TXS: AtomicU32 = AtomicU32::new(0);
#[used]
static RXS: AtomicU32 = AtomicU32::new(0);

fn init(
    rcc: &device::RCC,
    i2c: &device::I2C1,
    gpiob: &device::GPIOB,
) {
    // Un-gate clock to our I2C block.
    rcc.apbenr1.modify(|_, w| w.i2c1en().set_bit());
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

    gpiob.moder.modify(|_, w| {
        w.moder6().alternate();
        w.moder7().alternate();
        w
    });

    // Configure peripheral to respond to our address.
    i2c.oar1.write(|w| {
        w.oa1().bits(u16::from(ADDR7 << 1));
        w.oa1mode().bit7();
        w.oa1en().set_bit();
        w
    });

    i2c.cr1.write(|w| {
        // Byte-level acking
        w.sbc().set_bit();
        // TODO interrupts
        // Peripheral on
        w.pe().set_bit();
        w
    });
}

pub async fn task(
    rcc: &device::RCC,
    gpiob: &device::GPIOB,
    i2c: device::I2C1,
    mut bytes_from_serial: spsc::Pop<'_, u8>,
) -> Infallible {
    init(rcc, &i2c, gpiob);

    unsafe {
        cortex_m::peripheral::NVIC::unmask(device::Interrupt::I2C1);
    }

    // Transaction handling loop. We come up to the top of this whenever we're
    // idle, or if we're re-addressed during a transaction.
    loop {
        // Wait until we're addressed.
        i2c.cr1.modify(|_, w| w.addrie().set_bit());
        EVT.until(|| i2c.isr.read().addr().bit_is_set()).await;

        ADDR_DETECT.fetch_add_polyfill(1, Ordering::Relaxed);

        // We don't want to clear ADDR immediately, because we need to set up
        // the first byte of the transaction first. Clearing ADDR ends our clock
        // stretch and, for a read operation, starts transmission of our first
        // response byte.

        i2c.cr2.modify(|_, w| {
            // We're going to move one byte.
            w.nbytes().bits(1);
            // It won't be the last one.
            w.reload().set_bit();
            w
        });
        // Clear ADDR flag, ending any clock stretching.
        i2c.icr.write(|w| w.addrcf().set_bit());

        // Use select to compose the end-of-transaction monitor with the data
        // handler. If we get a stop condition, error, or repeated start, which
        // we must eventually, the data handler will be cancelled.
        select_biased! {
            result = terminal_condition(&i2c).fuse() => {
                match result {
                    Ok(()) => {
                        STOPS.fetch_add_polyfill(1, Ordering::Relaxed);
                        // stop condition
                    }
                    Err(Error) => {
                        // error
                    }
                }
            }
            never = handle_data(&i2c, &mut bytes_from_serial).fuse() => match never {}
        }
    }
}

async fn handle_data(
    i2c: &device::I2C1,
    bytes_from_serial: &mut spsc::Pop<'_, u8>,
) -> ! {
    // Determine the direction of the transfer.
    match i2c.isr.read().dir().variant() {
        device::i2c1::isr::DIR_A::Write => receive_data(i2c).await,
        device::i2c1::isr::DIR_A::Read => transmit_data(i2c, bytes_from_serial).await,
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
async fn receive_data(i2c: &device::I2C1) -> ! {
//    i2c.cr2.modify(|_, w| {
//        // We're going to ACK/NACK after the next byte.
//        w.nbytes().bits(1);
//        // It won't be the last one.
//        w.reload().set_bit();
//        w
//    });
//    // Clear ADDR flag, ending any clock stretching.
//    i2c.icr.write(|w| w.addrcf().set_bit());

    let mut nacking = false;

    loop {
        // Ensure we find out about any data that appears.
        i2c.cr1.modify(|_, w| w.rxie().set_bit());

        // Ensure we turn that back off if we're cancelled.
        scopeguard::defer! {
            i2c.cr1.modify(|_, w| w.rxie().clear_bit());
        }

        EVT.until(|| i2c.isr.read().rxne().bit_is_set()).await;

        // Pull the data out.
        let byte = i2c.rxdr.read().rxdata().bits();
        RXS.fetch_add_polyfill(1, Ordering::Relaxed);

        nacking |= byte != 0;

        i2c.cr2.modify(|_, w| {
            // Clear NACK flag so we ACK
            w.nack().bit(nacking);
            // We're going to ACK/NACK after the next byte.
            w.nbytes().bits(1);
            // It won't be the last one.
            w.reload().set_bit();
            w
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
    i2c: &device::I2C1,
    bytes_from_serial: &mut spsc::Pop<'_, u8>,
) -> ! {
//    i2c.cr2.modify(|_, w| {
//        // We're going to move one byte.
//        w.nbytes().bits(1);
//        // It won't be the last one.
//        w.reload().set_bit();
//        w
//    });
//    // Clear ADDR flag, ending any clock stretching.
//    i2c.icr.write(|w| w.addrcf().set_bit());

    loop {
        // Ensure we find out about any data that appears.
        i2c.cr1.modify(|_, w| w.txie().set_bit());

        // Ensure we turn that back off if we're cancelled.
        let txguard = scopeguard::guard((), |()| {
            i2c.cr1.modify(|_, w| w.txie().clear_bit());
        });

        EVT.until(|| i2c.isr.read().txis().bit_is_set()).await;

        ScopeGuard::into_inner(txguard);

        // Send the next byte.
        let byte = bytes_from_serial.try_pop().unwrap_or(0);
        i2c.txdr.write(|w| w.txdata().bits(byte));
        TXS.fetch_add_polyfill(1, Ordering::Relaxed);

        // Wait for TCR event.
        i2c.cr1.modify(|_, w| w.tcie().set_bit());

        // Ensure we turn that back off if we're cancelled.
        let tcguard = scopeguard::guard((), |()| {
            i2c.cr1.modify(|_, w| w.tcie().clear_bit());
        });

        EVT.until(|| i2c.isr.read().tcr().bit_is_set()).await;

        ScopeGuard::into_inner(tcguard);

        i2c.cr2.modify(|_, w| {
            // We'll do one byte again.
            w.nbytes().bits(1);
            // It won't be the last one.
            w.reload().set_bit();
            w
        });
    }
}

async fn terminal_condition(i2c: &device::I2C1) -> Result<(), Error> {
    // Ensure we'll get interrupts on the conditions we're monitoring.
    i2c.cr1.modify(|_, w| {
        w.addrie().set_bit();
        w.stopie().set_bit();
        w.errie().set_bit();
        w
    });

    scopeguard::defer! {
        // Make sure all our interrupt enables are clear -- the ISR will have
        // cleared one of them but probably not both. (We likely won't be
        // cancelled because of how the driver is constructed, but it's still
        // nice to do things the right way.)
        i2c.cr1.modify(|_, w| {
            w.addrie().clear_bit();
            w.stopie().clear_bit();
            w.errie().clear_bit();
            w
        });
    }

    let result = EVT.until(|| {
        let isr = i2c.isr.read();
        let berr = isr.berr().bit_is_set();
        let arlo = isr.arlo().bit_is_set();
        if berr {
            ERR_BUS.fetch_add_polyfill(1, Ordering::Relaxed);
        }
        if arlo {
            ERR_ARLO.fetch_add_polyfill(1, Ordering::Relaxed);
        }
        if berr || arlo {
            Some(Err(Error))
        } else if isr.stopf().bit_is_set() || isr.addr().bit_is_set() {
            Some(Ok(()))
        } else {
            None
        }
    }).await;

    // If we've detected our event, clear our flags for next time. Do NOT
    // clear the ADDR flag, as that has side effects! It will be processed in
    // the outer loop.
    i2c.icr.write(|w| {
        w.stopcf().set_bit();
        w.berrcf().set_bit();
        w.arlocf().set_bit();
        w
    });

    result
}

#[derive(Copy, Clone, Debug)]
struct Error;

#[interrupt]
fn I2C1() {
    let i2c = unsafe { &*device::I2C1::PTR };
    let cr1 = i2c.cr1.read();
    let isr = i2c.isr.read();

    let mut bits_to_clear = 0;
    if cr1.txie().bit_is_set() && isr.txis().bit_is_set() {
        bits_to_clear |= 1 << 1;
    }
    if cr1.rxie().bit_is_set() && isr.rxne().bit_is_set() {
        bits_to_clear |= 1 << 2;
    }
    if cr1.stopie().bit_is_set() && isr.stopf().bit_is_set() {
        bits_to_clear |= 1 << 5;
    }
    if cr1.addrie().bit_is_set() && isr.addr().bit_is_set() {
        bits_to_clear |= 1 << 3;
    }
    if cr1.tcie().bit_is_set() && isr.tcr().bit_is_set() {
        bits_to_clear |= 1 << 6;
    }
    if cr1.errie().bit_is_set() && (isr.arlo().bit_is_set() || isr.berr().bit_is_set()) {
        bits_to_clear |= 1 << 7;
    }

    if bits_to_clear != 0 {
        i2c.cr1.modify(|r, w| unsafe {
            w.bits(r.bits() & !bits_to_clear)
        });
        EVT.notify();
    }
}

static EVT: Notify = Notify::new();
