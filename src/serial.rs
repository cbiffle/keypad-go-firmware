// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Serial interface.
//!
//! # Architecture
//!
//! This combines two different modes: normal operation and setup.
//!
//! In normal operation, the serial task receives key events from the scanner
//! task, processes them against the configured ASCII key map, and transmits
//! them out the UART. As the UART is asynchronous, we always have "permission"
//! to transmit, so we can always process keypresses promptly. This prevents the
//! queue coming from the scanner task from backing up.
//!
//! During normal operation, the serial task also copies every transmitted byte
//! into a queue for the I2C interface. This queue _can_ back up, in which case
//! data will be lost from the perspective of an I2C client. This causes the
//! I2C interface's behavior on key buffer overflow, which is described in the
//! product manual.
//!
//! During setup mode, we do something entirely different and slightly grody,
//! which I'll discuss at the `setup` routine below.
//!
//! # Peripheral choice
//!
//! Note: On the STM32G0/C0, only USART1 is suitable for use here, because
//! USART2 is less featureful -- in particular, it has no FIFO, and the driver
//! is currently written to make use of the FIFO.
//!
//! So if you're here wanting to switch the active USART... you've got some work
//! ahead of you.
//!
//! From an ergonomics point of view, USART1 is the right one to wire up because
//! it's the one that can serve the UART-based bootloader -- meaning, the user
//! can do firmware updates on the same pins they use to receive keypresses,
//! which is the goal.

use core::convert::Infallible;
use core::slice::from_ref;

use device::{gpio::vals::Moder, usart::vals::Stop};
use futures::{Future, select_biased, FutureExt};
use lilos::{exec::{Notify, with_timeout, sleep_for}, handoff, spsc, time::Millis};

use crate::{device::{self, interrupt}, scanner::{KeyState, self}, flash::{Storage, SystemConfig}};

// This type alias was more useful when I was using the stm32 PAC, which made
// every UART a different type to make my life difficult. It could probably be
// removed now that I'm on the metapac (TODO).
type Uart = device::usart::Usart;

/// Serial processing task.
///
/// After setting up the serial port, this immediately makes a decision about
/// which mode to enter based on the `setup_mode` flag, and stays in that mode
/// forever until reset.
///
/// `uart` is our UART, in case that was not apparent.
///
/// The task will configure `gpio` for the UART.
///
/// `keymap` is the _initial_ configured keymap loaded from Flash, assigning an
/// ASCII character to each matrix position. It will be used verbatim in normal
/// operation, and ignored completely in setup mode.
///
/// `setup_mode` enters setup mode when set. Could this instead be used to
/// choose one of two different tasks to call in main? Sure. But that's not how
/// it works right now.
///
/// `from_scanner` is events coming from the scan task, while
/// `config_to_scanner` lets us push new configurations into the scanner during
/// setup.
///
/// `bytes_to_i2c` is used to send processed key bytes to the I2C task.
///
/// Finally, after startup we own `storage` so we can rewrite flash during setup
/// mode.
pub async fn task(
    uart: Uart,
    gpioa: device::gpio::Gpio,
    keymap: &[[u8; 8]; 8],
    setup_mode: bool,
    mut from_scanner: spsc::Pop<'_, scanner::KeyEvent>,
    config_to_scanner: handoff::Push<'_, scanner::Config>,
    mut bytes_to_i2c: spsc::Push<'_, u8>,
    storage: Storage,
) -> Infallible {
    init(gpioa, uart);

    if setup_mode {
        setup(uart, config_to_scanner, from_scanner, storage).await
    } else {
        loop {
            let event = from_scanner.pop().await;
            // Look up the mapped key corresponding to the event.
            let byte = keymap[event.driven_line()][event.sensed_line()];
            // Adjust its MSB to indicate state transitions.
            let byte = match event.state {
                KeyState::Down => byte,
                KeyState::Up => byte | 0x80,
            };
            // Stuff the byte into the I2C key event queue best-effort. If the
            // bus initiator fails to keep up, we lose keys. So be it.
            bytes_to_i2c.try_push(byte).ok();

            transmit_byte(uart, &byte).await;
        }
    }
}

/// Serial port initialization routine, factored out of task.
fn init(gpioa: device::gpio::Gpio, uart: Uart) {
    gpioa.moder().modify(|w| {
        w.set_moder(9, Moder::ALTERNATE); // USART1_TX
        w.set_moder(10, Moder::ALTERNATE); // USART1_RX
    });

    // NOTE: this is where I originally wanted to activate a UART pullup, which
    // would let us transmit serial at whatever VBUS is by driving the UART open
    // drain.
    //
    // But!
    //
    // Turning on the pullup (or pulldown!) resistors causes the pin to stop
    // being 5V tolerant on the G0/C0 series. I feel like providing
    // 5V-compatible I/O is much more important than avoiding some spurious UART
    // errors, so, no pullups are being set here.
    //
    // That being said, we're "5V-compatible" iff 3.3V is above Vih for the
    // receiver, which it often isn't.

    // The UART is being clocked at 48 MHz. In the default 16x oversampled mode,
    // this makes the math really freaking easy. (The unwrap gets compiled out.)
    let brr = u16::try_from(48_000_000_u32 / 19_200).unwrap();
    uart.brr().write(|w| w.set_brr(brr));
    // Switch to two stop bits for widest compatibility.
    uart.cr2().write(|w| w.set_stop(Stop::STOP2));
    // Turn everything on.
    uart.cr1().write(|w| {
        w.set_fifoen(true);
        w.set_te(true);
        w.set_re(true);
        w.set_ue(true);
    });

    unsafe {
        cortex_m::peripheral::NVIC::unmask(device::Interrupt::USART1);
    }
}

/// Single-byte transmit routine.
///
/// Thin wrapper around the slice transmit routine, which gets called in many
/// more places.
fn transmit_byte<'a>(uart: Uart, byte: &'a u8) -> impl Future<Output = ()> + 'a {
    transmit(uart, from_ref(byte))
}

/// Slice transmit routine.
///
/// # Cancellation
///
/// This currently takes no particular pains to be cancel-safe, which I can do
/// because it's only called from a top-level task, which won't be cancelled.
///
/// That said, I _think_ if you cancel this it'll work.
async fn transmit(uart: Uart, buffer: &[u8]) {
    for &byte in buffer {
        // Sleep only if we need to. In many cases we'll have gone off and done
        // enough other stuff that sleep won't be necessary.
        //
        // Note: ISR.TXE == ISR.TXFNF (bit 7)
        //       CR1.TXEIE == CR1.TXFNFIE (bit 7)
        // (the metapac does not match the RM here.)
        if !uart.isr().read().txe() {
            uart.cr1().modify(|w| w.set_txeie(true));
            TX_AVAIL.until(|| uart.isr().read().txe()).await;
        }

        uart.tdr().write(|w| w.set_dr(u16::from(byte)));
    }
}

/// Empty the UART's incoming FIFO to discard spurious data.
fn drain(uart: Uart) {
    while uart.isr().read().rxne() {
        let _discard = uart.rdr().read();
    }

    uart.icr().write(|w| {
        w.set_fe(true);
        w.set_ne(true);
        w.set_ore(true);
    });
}

/// Receive a single byte from the UART.
///
/// This will avoid sleeping if data is available, to make sure we keep up.
async fn recv(uart: Uart) -> Result<u8, RxErr> {
    if let Some(out) = recv_one(uart) {
        return out;
    }

    uart.cr1().modify(|w| {
        w.set_rxneie(true);
    });
    uart.cr3().modify(|w| {
        w.set_eie(true);
    });
    RX_AVAIL.until(|| recv_one(uart)).await
}

/// Non-blocking version of recv, because I found it easier to reason about this
/// and then make it async above.
fn recv_one(uart: Uart) -> Option<Result<u8, RxErr>> {
    let isr = uart.isr().read();

    // The overrun bit bypasses the FIFO. Check it first. Otherwise, we could
    // dequeue an inconsistent sequence of frames without noticing the overrun.
    if isr.ore() {
        // Ignore other flags and discard all pending data.
        drain(uart);
        Some(Err(RxErr::Desync))
    } else if isr.fe() || isr.ne() {
        // There are three bits that come through the FIFO: Framing Error, Noise
        // Error, and Parity Error. This driver explicitly ignores the existence
        // of parity, so that leaves us with two flags.
        //
        // If either of these flags is set, we need to pop the RX FIFO despite
        // not honoring the data that comes out of it.
        //
        // This isn't bothering distinguishing framing error from noise, because
        // currently BRK isn't significant.

        // TODO: the manual is really ambiguous on this - do we need to clear
        // flags if we pop the FIFO? i.e. do flags accumulate or replace when
        // the FIFO is popped? This is worth testing as it could save some
        // bytes.
        uart.icr().write(|w| {
            w.set_fe(true);
            w.set_ne(true);
        });

        let _discard = uart.rdr().read();

        Some(Err(RxErr::FrameErr(FrameErr)))
    } else if isr.rxne() {
        Some(Ok(uart.rdr().read().0 as u8))
    } else {
        None
    }
}

/// Serial+FIFO error type.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum RxErr {
    /// A FIFO overrun error means that any protocol running atop this UART has
    /// desynchronized and must resync.
    Desync,
    /// We had some sort of framing error with the incoming data.
    FrameErr(FrameErr),
}

/// This indicates either noise or a break; we don't currently need to
/// distinguish.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
struct FrameErr;

static TX_AVAIL: Notify = Notify::new();
static TX_COMPLETE: Notify = Notify::new();

static RX_AVAIL: Notify = Notify::new();

#[interrupt]
fn USART1() {
    let uart = device::USART1;
    let isr = uart.isr().read();
    let cr1 = uart.cr1().read();
    let cr3 = uart.cr3().read();

    let mut bits_to_clear = 0;

    if cr1.txeie() {
        if isr.txe() {
            TX_AVAIL.notify();
            bits_to_clear |= 1 << 7;
        }
    }
    if cr1.tcie() {
        if isr.tc() {
            TX_COMPLETE.notify();
            bits_to_clear |= 1 << 6;
        }
    }

    let mut notify_rx = false;
    if cr3.eie() {
        if isr.ore() || isr.fe() || isr.ne() {
            notify_rx = true;
            // This is the only bit we clear in CR3, so go ahead and do it
            // instead of combining writes.
            uart.cr3().modify(|w| w.set_eie(false));
        }
    }
    if cr1.rxneie() {
        if isr.rxne() {
            notify_rx = true;
            bits_to_clear |= 1 << 5;
        }
    }

    if notify_rx {
        RX_AVAIL.notify();
    }

    // This could be made conditional on bits_to_clear != 0, but (1) it should
    // almost always be nonzero, (2) it is harmless if zero, (3) this saves a
    // couple instructions and I'm feeling stingy.
    uart.cr1().modify(|w| w.0 &= !bits_to_clear);
}

///////////////////////////////////////////////////////////////////////////////
// The Setup Interface
//
// Below you'll find all the bits that interact with the user to work out the
// key matrix configuration.
//
// This is slightly grody right now because (1) I was in a hurry to finish the
// art and (2) I avoided using any higher-level I/O routines for space reasons,
// and (3) we can't use basic multi-line strings in Rust because serial devices
// generally need \r\n endings, but Rust does \n. And I haven't bothered
// translating in the driver.
//
// Could almost certainly be improved!

/// It's super weird that Rust doesn't have an escape sequence for this, but, it
/// doesn't!
const ESC: u8 = 0x1B;

/// Amount of time we wait after seeing the _first_ change indicating a
/// keypress, before we decide that we've found _all_ paths to that key.
///
/// Slightly arbitrary.
const SETTLE: Millis = Millis(100);

/// The setup routine. Takes over the UART. Does not drive the I2C interface,
/// so, during setup the I2C port will be dead.
async fn setup(
    uart: Uart,
    mut config_to_scanner: handoff::Push<'_, scanner::Config>,
    mut from_scanner: lilos::spsc::Pop<'_, scanner::KeyEvent>,
    mut storage: Storage,
) -> Infallible {
    sleep_for(Millis(100)).await;

    // Arbitrary but non-zero epoch chosen to mark our initial config. We use
    // this to distinguish any key presses that occur while we're still getting
    // rolling (which will arrive with epoch 0) from key presses after our
    // configuration has been applied (which will use this epoch).
    const SETUP_EPOCH: u8 = 0x55;

    config_to_scanner.push(scanner::Config {
        // Recognizable epoch in case we raced the scan
        epoch: SETUP_EPOCH,
        // All lines should be driven during setup.
        driven_lines: 0xFF,
        // No ghost keys should be ignored, we want to know about everything.
        ghost_mask: [0; 8],
    }).await;

    transmit_v(uart, &[
        b"\r\n\r\nSETUP MODE\r\n",
        b"Firmware version: ",
        env!("CARGO_PKG_VERSION").as_bytes(),
        b"\r\n\r\n",
    ]).await;

    // Keymap we're maintaining. Holds an ASCII character per matrix
    // intersection, or 0 if not yet configured.
    let mut keys = [[0; 8]; 8];
    'learnloop:
    loop {
        transmit(uart,
          b"Press+hold any keypad button.\r\n\
          Type ESC here if no more.\r\n").await;
        // Bit per matrix intersection recording the paths we've observed this
        // time.
        let mut connectivity = [0u8; 8];
        // Bit per matrix intersection noting if we've seen something release,
        // so that we don't sit there insisting the user must release a key that
        // they've simply pressed and released inhumanly fast.
        let mut early_releases = [0u8; 8];
        // At this point we want to either....
        let evt = loop {
            select_biased! {
                // ...receive a character from the UART, which we'll ignore
                // unless it's trying to abort the setup process, or...
                c = recv(uart).fuse() => {
                    if c == Ok(ESC) {
                        break 'learnloop;
                    }
                }
                // ...receive an event from the key scanner, which will drop us
                // into the collection loop below.
                evt = from_scanner.pop().fuse() => break evt,
            }
        };
        if evt.epoch != SETUP_EPOCH {
            // Presumably this is cruft from startup. This will also ignore
            // events if the epoch somehow changes during setup, but, it won't,
            // and this is the simplest code.
            continue;
        }
        if evt.state != KeyState::Down {
            // Huh.
            continue;
        }
        // This is our first key! Record its connectivity.
        connectivity[evt.driven_line()] |= 1 << evt.sensed_line();
        // To allow for settling when multiple paths are connected, not to
        // mention the delay inherent in scanning multiple lines, we'll wait a
        // somewhat arbitrary period of time for more events to come in.
        with_timeout(SETTLE, async {
            loop {
                let evt = from_scanner.pop().await;
                connectivity[evt.driven_line()] |= 1 << evt.sensed_line();
                // If the user presses and releases the key in less than 100ms
                // -- which is totally possible under normal circumstances --
                // we'll see an Up event here and need to track it so that we
                // don't sit there insisting the user release a key that they've
                // released long ago.
                if evt.state == KeyState::Up {
                    early_releases[evt.driven_line()] |= 1 << evt.sensed_line();
                }
            }
        }).await;

        // Print all connectivity we discovered this round.
        transmit(uart, b"Found:\r\n").await;
        let mut first_path = None;
        for (i, &mask) in connectivity.iter().enumerate() {
            if mask != 0 {
                transmit_v(uart, &[
                    from_ref(&pin_digit(i as u8)),
                    b" ->",
                ]).await;
                for col in 0..8 {
                    if mask & (1 << col) != 0 {
                        transmit_v(uart, &[
                            b" ",
                            from_ref(&pin_digit(col as u8)),
                            if first_path.is_none() {
                                first_path = Some((i, col));
                                b"*"
                            } else {
                                b""
                            },
                        ]).await;
                    }
                }
                transmit(uart, b"\r\n").await;
            }
        }
        // Because we got here in response to a key event that set at least one
        // bit in the connectivity matrix, this will be non-None.
        let first_path = first_path.unwrap();
        // Check for duplicate keys. We don't immediately continue the loop here
        // because we do some cleanup, below. We just record it in the known
        // flag.
        let known = keys[first_path.0][first_path.1] != 0;
        if known {
            transmit(uart, b"\r\nAlready configured.\r\n\
                To reconfigure, hit RESET.\r\n").await;
        }

        // Process early releases
        for (conn, rel) in connectivity.iter_mut().zip(&early_releases) {
            *conn &= !rel;
        }

        // We really shouldn't be able to get here, but, handle it just in case
        // -- because the alternative is a potentially weird experience.
        if connectivity != [0; 8] {
            transmit(uart, b"\r\nPlease release button.\r\n").await;

            while connectivity != [0; 8] {
                let evt = from_scanner.pop().await;
                if evt.state == KeyState::Up {
                    connectivity[evt.driven_line()] &= !(1 << evt.sensed_line());
                }
            }
        }

        // _Now_ we deduplicate keys.
        if known { continue; }

        // Throw away anything that accumulated while we were being chatty.
        drain(uart);

        transmit(uart, b"\r\nType the key's character here: ").await;
        let entered_char = loop {
            match recv(uart).await {
                // Bus errors and breaks can theoretically happen rn, just
                // discard them.
                Err(_) => continue,
                Ok(x) => if x != 0 { break x },
            }
        };

        keys[first_path.0][first_path.1] = entered_char;
        // Key echo makes for happier users.
        transmit_v(uart, &[
            from_ref(&entered_char),
            b"\r\nOK\r\n\r\n",
        ]).await;
    }

    // Report the overall mapping we've discovered.
    transmit(uart, b"Mapping:\r\n").await;
    let mut driven_lines = 0;
    let mut ghost_mask = [0xFF; 8];
    for p in 0..8 {
        if keys[usize::from(p)] != [0; 8] {
            // The set of driven lines is the set of rows in the `keys` matrix
            // where at least one assigned key exists.
            driven_lines |= 1 << p;
            transmit_v(uart, &[
                b"drive ",
                from_ref(&pin_digit(p)),
                b"\r\n",
            ]).await;
            for col in 0..8 {
                let k = keys[usize::from(p)][usize::from(col)];
                if k != 0 {
                    // The ghost key mask contains a 0 in each position where a
                    // mapped key exists.
                    ghost_mask[usize::from(p)] &= !(1 << col);

                    transmit_v(uart, &[
                        b" - sense ",
                        from_ref(&pin_digit(col)),
                        b" => '",
                        from_ref(&k),
                        b"'\r\n",
                    ]).await;
                }
            }
        }
    }

    // Build a config with yet another epoch so we can print keypresses.
    const DEMO_EPOCH: u8 = 1;
    let cfg = SystemConfig {
        scanner: scanner::Config {
            epoch: DEMO_EPOCH,
            driven_lines,
            ghost_mask,
        },
        keymap: keys,
    };

    // Optionally save the configuration to flash.
    transmit(uart, b"\r\nSave? Y/N\r\n").await;
    loop {
        match recv(uart).await {
            Ok(b'y') | Ok(b'Y') => {
                storage.write_config(&cfg);
                transmit(uart, b"Saved!\r\n").await;
                break;
            }
            Ok(b'n') | Ok(b'N') => {
                break;
            }
            _ => (),
        }
    }

    // Push the configuration over whether or not we saved it, so we can run the
    // demo loop.
    config_to_scanner.push(scanner::Config {
        epoch: DEMO_EPOCH,
        driven_lines,
        ghost_mask,
    }).await;

    transmit(uart, b"Running demo until reset:\r\n").await;

    loop {
        let evt = from_scanner.pop().await;
        if evt.epoch != DEMO_EPOCH {
            continue;
        }
        transmit_v(uart, &[
            match evt.state {
                KeyState::Down => b"down: ",
                KeyState::Up => b"up:   ",
            },
            from_ref(&keys[evt.driven_line()][evt.sensed_line()]),
            b"\r\n",
        ]).await;
    }
}

/// Decimal digit formatting the cheap way. Only correct for pins 0-9.
fn pin_digit(pin: u8) -> u8 {
    b'0'.wrapping_add(pin)
}

/// Vectored version of transmit. Sends a slice of slices.
///
/// This exists for two reasons:
///
/// 1. It's convenient, if you're sending a mix of static and dynamic strings,
///    and avoids allocating e.g. a buffer to copy a formatted message into.
/// 2. It reduces the _static_ number of calls to async fns. (A call in a loop
///    only counts as one.) This reduces flash size.
async fn transmit_v(uart: Uart, buffers: &[&[u8]]) {
    for buffer in buffers {
        transmit(uart, buffer).await;
    }
}
