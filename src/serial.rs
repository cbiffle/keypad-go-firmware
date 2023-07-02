//! Serial interface.
//!
//! Note: On the STM32G0/C0, only USART1 is suitable for use here, because
//! USART2 is less featureful -- in particular, it has no FIFO, and the driver
//! is currently written to make use of the FIFO.
//!
//! So if you're here wanting to switch the active USART... you've got some work
//! ahead of you.

use core::convert::Infallible;
use core::slice::from_ref;

use device::gpio::vals::Moder;
use futures::{Future, select_biased, FutureExt};
use lilos::{exec::{Notify, with_timeout}, handoff, spsc, time::Millis};

use crate::{device::{self, interrupt}, scanner::{KeyState, self}, flash::{Storage, SystemConfig}};

pub type Uart = device::usart::Usart;

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
            let byte = keymap[usize::from(event.driven_line())][usize::from(event.sensed_line())];
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

/// It's super weird that Rust doesn't have an escape sequence for this, but, it
/// doesn't!
const ESC: u8 = 0x1B;

pub async fn setup(
    uart: Uart,
    mut config_to_scanner: handoff::Push<'_, scanner::Config>,
    mut from_scanner: lilos::spsc::Pop<'_, scanner::KeyEvent>,
    mut storage: Storage,
) -> Infallible {
    const SETUP_EPOCH: u8 = 0x55;
    config_to_scanner.push(scanner::Config {
        // Recognizable epoch in case we raced the scan
        epoch: SETUP_EPOCH,
        // All lines should be driven during setup.
        driven_lines: 0xFF,
        // No ghost keys should be ignored.
        ghost_mask: [0; 8],
    }).await;
    transmit(uart, b"\r\n\r\nSETUP MODE\r\n\r\n").await;

    let mut keys = [[0; 8]; 8];
    'learnloop:
    loop {
        transmit(uart,
          b"Press+hold any keypad button.\r\n\
          Type ESC here if no more.\r\n").await;
        let mut connectivity = [0u8; 8];
        let mut early_releases = [0u8; 8];
        let evt = loop {
            select_biased! {
                c = recv(uart).fuse() => {
                    if c == Ok(ESC) {
                        break 'learnloop;
                    }
                }
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
        connectivity[usize::from(evt.driven_line())] |= 1 << evt.sensed_line();
        // To allow for settling when multiple paths are connected, not to
        // mention the delay inherent in scanning multiple lines, we'll wait a
        // somewhat arbitrary period of time for more events to come in.
        with_timeout(Millis(100), async {
            loop {
                let evt = from_scanner.pop().await;
                connectivity[usize::from(evt.driven_line())] |= 1 << evt.sensed_line();
                // If the user presses and releases the key in less than 100ms
                // -- which is totally possible under normal circumstances --
                // we'll see an Up event here and need to track it so that we
                // don't sit there insisting the user release a key that they've
                // released long ago.
                if evt.state == KeyState::Up {
                    early_releases[usize::from(evt.driven_line())] |= 1 << evt.sensed_line();
                }
            }
        }).await;

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
                newline(uart).await;
            }
        }
        let first_path = first_path.unwrap();
        let known = keys[first_path.0][first_path.1] != 0;
        if known {
            transmit(uart, b"\r\nAlready configured.\r\n\
                To reconfigure, hit RESET.\r\n").await;
        }

        // Process early releases
        for (conn, rel) in connectivity.iter_mut().zip(&early_releases) {
            *conn &= !rel;
        }

        if connectivity != [0; 8] {
            transmit(uart, b"\r\nPlease release button.\r\n").await;

            while connectivity != [0; 8] {
                let evt = from_scanner.pop().await;
                if evt.state == KeyState::Up {
                    connectivity[usize::from(evt.driven_line())] &= !(1 << evt.sensed_line());
                }
            }
        }

        if known { continue; }

        drain(uart);
        transmit(uart, b"\r\nType the key's character here: ").await;
        let entered_char = loop {
            match recv(uart).await {
                Err(_) => continue,
                Ok(x) => if x != 0 { break x },
            }
        };

        keys[first_path.0][first_path.1] = entered_char;
        transmit_v(uart, &[
            from_ref(&entered_char),
            b"\r\nOK\r\n\r\n",
        ]).await;
    }

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

    let cfg = SystemConfig {
        scanner: scanner::Config {
            epoch: DEMO_EPOCH,
            driven_lines,
            ghost_mask,
        },
        keymap: keys,
    };
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

    const DEMO_EPOCH: u8 = 1;
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
            from_ref(&keys[usize::from(evt.driven_line())][usize::from(evt.sensed_line())]),
            b"\r\n",
        ]).await;
    }
}

fn pin_digit(pin: u8) -> u8 {
    b'0'.wrapping_add(pin)
}

pub fn init(gpioa: device::gpio::Gpio, uart: Uart) {
    gpioa.moder().modify(|w| {
        w.set_moder(9, Moder::ALTERNATE); // USART1_TX
        w.set_moder(10, Moder::ALTERNATE); // USART1_RX
    });

    // NOTE: this is where I originally wanted to activate a UART pullup.
    // Turning on the pullup (or pulldown!) resistors causes the pin to stop
    // being 5V tolerant on the G0/C0 series. I feel like providing
    // 5V-compatible I/O is much more important than avoiding some spurious UART
    // errors, so, no pullups are being set here.

    // The UART is being clocked at 48 MHz. In the default 16x oversampled mode,
    // this makes the math really freaking easy. (The unwrap gets compiled out.)
    let brr = u16::try_from(48_000_000_u32 / 19_200).unwrap();
    uart.brr().write(|w| w.set_brr(brr));
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

pub async fn transmit_v(uart: Uart, buffers: &[&[u8]]) {
    for buffer in buffers {
        transmit(uart, buffer).await;
    }
}

pub fn transmit_byte<'a>(uart: Uart, byte: &'a u8) -> impl Future<Output = ()> + 'a {
    transmit(uart, from_ref(byte))
}

pub async fn transmit(uart: Uart, buffer: &[u8]) {
    for &byte in buffer {
        // Note: ISR.TXE == ISR.TXFNF (bit 7)
        //       CR1.TXEIE == CR1.TXFNFIE (bit 7)
        if !uart.isr().read().txe() {
            uart.cr1().modify(|w| w.set_txeie(true));
            TX_AVAIL.until(|| uart.isr().read().txe()).await;
        }

        uart.tdr().write(|w| w.set_dr(u16::from(byte)));
    }
}

pub fn newline(uart: Uart) -> impl Future<Output = ()> {
    transmit(uart, b"\r\n")
}

pub fn drain(uart: Uart) {
    while uart.isr().read().rxne() {
        let _discard = uart.rdr().read();
    }

    uart.icr().write(|w| {
        w.set_fe(true);
        w.set_ne(true);
        w.set_ore(true);
    });
}

pub async fn recv(uart: Uart) -> Result<u8, RxErr> {
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

fn recv_one(uart: Uart) -> Option<Result<u8, RxErr>> {
    let isr = uart.isr().read();

    // The overrun bit bypasses the FIFO. Check it first. Otherwise, we could
    // dequeue an inconsistent sequence of frames without noticing the overrun.
    if isr.ore() {
        // Ignore other flags and discard all pending data.
        drain(uart);
        return Some(Err(RxErr::Desync));
    }

    // There are three bits that come through the FIFO: Framing Error, Noise
    // Error, and Parity Error. This driver explicitly ignores the existence of
    // parity, so that leaves us with two flags.
    //
    // If either of these flags is set, we need to pop the RX FIFO despite not
    // honoring the data that comes out of it.
    if isr.fe() || isr.ne() {
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

        return Some(Err(RxErr::FrameErr(FrameErr)));
    }

    if isr.rxne() {
        return Some(Ok(uart.rdr().read().0 as u8));
    }

    None
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum RxErr {
    Desync,
    FrameErr(FrameErr),
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct FrameErr;

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
