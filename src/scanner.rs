//! Keypad matrix scan driver.
//!
//! # Theory of operation
//!
//! We have 8 controlled pins. A keypad will indicate keys down by shorting
//! particular pins together. Some types of fancier keypads will use diodes to
//! do this, meaning it's not a _short_ per se (it only conducts one way).
//!
//! We configure the processor so that the idle state of all these pins is
//! "weakly pulled up." (The internal pullup resistors are roughly 20 kR.) To
//! sense the state of the matrix, we drive one pin low at a time. Any
//! conducting paths through the matrix will show up as other pins going low
//! shortly thereafter. (In the event that there's a conductive path to _every
//! other pin,_ the pin we're driving low is now fighting about 2.5 kR
//! resistance, which is 1.3 mA, which is fine.)
//!
//! By cycling through our pins, we can discover every conducting path through
//! the matrix.
//!
//! In a normal keypad, with no diodes, paths will appear as bidirectionally
//! conducting short circuits. This means keys will appear twice from our
//! perspective: as a short between (say) pins 3 and 7, and also as a short
//! between pins 7 and 3. We rely on configuration information (below) to tell
//! whether those indicate the same key (in which case we call one of the two a
//! "ghost"), or whether the keypad uses diodes and those are two entirely
//! separate keys down (in which case neither is a ghost).
//!
//! # Configuration
//!
//! To customize our operation to a _particular_ keypad, we expose two pieces of
//! configuration information:
//!
//! - A "driven line mask" determines which subset of our 8 pins we actually
//!   bother pulling low. This can improve update rate in the common case where
//!   a subset of the pins act as "rows" opposed to the other pins' "columns,"
//!   and avoid detecting ghost keys from bidirectionally conductive paths.
//!
//! - A "ghost mask" determines which intersections of the matrix should be
//!   treated as real keys (and thus debounced and reported), vs which ones are
//!   ghosts (and thus should be ignored).
//!
//! Together, this information helps to reduce the amount of work done per scan,
//! which improves scanning report rate. It also helps to reduce the number of
//! spurious key reports that need to be filtered out by our consumer, which
//! means we can use a smaller queue to deliver those reports, meaning less RAM
//! used.
//!
//! # Limitations
//!
//! The approach above means that this driver is limited to a maximum keypad
//! size of:
//!
//! - Normal keypad without diodes: 16 keys (in 4 rows, 4 cols).
//! - Fancy keypad with diodes: 56 keys.

use core::convert::Infallible;
use core::sync::atomic::{AtomicUsize, Ordering};

use lilos::exec::PeriodicGate;
use lilos::{handoff, spsc};
use lilos::time::{TickTime, Millis};

use crate::device;

#[used]
static STATE: AtomicUsize = AtomicUsize::new(0);

#[derive(Copy, Clone, Debug)]
pub struct Config {
    /// Repeated in events to distinguish events derived from this config from
    /// those derived from some previous config.
    pub epoch: u8,
    /// For each connection to the keypad 0-7, this byte contains a 1 in the
    /// corresponding bit if the line should be driven low during scan, and a 0
    /// if it should be skipped.
    pub driven_lines: u8,
    /// `ghost_mask[x]` is a bitmask of which pins should be ignored when line
    /// `x` is driven. A 1 indicates ignored (ghost), a 0 indicates not ignored
    /// (not ghost).
    pub ghost_mask: [u8; 8],
}

impl Default for Config {
    fn default() -> Self {
        Self {
            epoch: 0,
            driven_lines: 0xFF,
            ghost_mask: [0; 8],
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct KeyEvent {
    /// Epoch of config that generated this event.
    pub epoch: u8,
    /// Designates a matrix position by encoding the driven line (0-7) in the
    /// least significant 3 bits, and the received line (0-7) in the most
    /// significant 3 bits, producing a number between 0 and 63, inclusive.
    pub coord: u8,
    /// What state the key was determined to be in, now.
    pub state: KeyState,
}

impl KeyEvent {
    pub fn driven_line(&self) -> u8 {
        self.coord & 0x7
    }
    pub fn sensed_line(&self) -> u8 {
        self.coord >> 3
    }
}

pub async fn task(
    mut config: Config,
    mut config_update: handoff::Pop<'_, Config>,
    gpio: &device::GPIOA,
    mut out_queue: spsc::Push<'_, KeyEvent>,
) -> Infallible {
    configure_pins(gpio);

    let mut debouncers = [[Debounce::default(); 8]; 8];
    let mut scan_gate = PeriodicGate::from(Millis(1));
    loop {
        STATE.store(1, Ordering::SeqCst);
        scan_gate.next_time().await;

        // Process any config update before starting the scan.
        if let Some(new_config) = config_update.try_pop() {
            STATE.store(6, Ordering::SeqCst);
            // Handle any "killed" drive lines. This occurs when the
            // `driven_lines` bit was 1 in the old config, and is becoming 0.
            for line in 0..8 {
                let line_mask = 1 << line;
                if config.driven_lines & line_mask != 0 && new_config.driven_lines & line_mask == 0 {
                    for col in 0..8 {
                        debouncers[line][col] = Default::default();
                    }
                }
            }
            // Handle any "killed" keys. This occurs when the key's bit in the
            // `ghost_mask` was not set before, but has become set.
            for line in 0..8 {
                for col in 0..8 {
                    let col_mask = 1 << col;
                    if config.ghost_mask[line] & col_mask == 0 && new_config.ghost_mask[line] & col_mask != 0 {
                        debouncers[line][col] = Default::default();
                    }
                }
            }
            // Apply the config.
            config = new_config;
        }
        // Config doesn't get modified anywhere else in this loop -- see:
        let config = &config;

        STATE.store(2, Ordering::SeqCst);
        let down_mask = scan(config, gpio).await;
        STATE.store(3, Ordering::SeqCst);

        for line in 0..8 {
            for col in 0..8 {
                let s = if down_mask & (1 << (8 * line + col)) != 0 { KeyState::Down } else { KeyState::Up };
                if let Some(new_state) = debouncers[line][col].step(s) {
                    let evt = KeyEvent {
                        epoch: config.epoch,
                        coord: (line | (col << 3)) as u8,
                        state: new_state,
                    };
                    out_queue.try_push(evt).ok();
                }
            }
        }

    }
}

fn configure_pins(gpio: &device::GPIOA) {
    // Ensure all scan pins are open drain (only drive low)
    gpio.otyper.modify(|_, w| {
        w.ot0().open_drain();
        w.ot1().open_drain();
        w.ot2().open_drain();
        w.ot3().open_drain();
        w.ot4().open_drain();
        w.ot5().open_drain();
        w.ot6().open_drain();
        w.ot7().open_drain();
        w
    });
    gpio.pupdr.modify(|_, w| {
        w.pupdr0().pull_up();
        w.pupdr1().pull_up();
        w.pupdr2().pull_up();
        w.pupdr3().pull_up();
        w.pupdr4().pull_up();
        w.pupdr5().pull_up();
        w.pupdr6().pull_up();
        w.pupdr7().pull_up();
        w
    });

    // Ensure all scan pins are initially driven high (i.e. not driven)
    gpio.odr.modify(|r, w| unsafe { w.bits(r.bits() | 0xFF) });

    // Switch pins to outputs. Note that this also enables the schmitt trigger
    // input, which is important for reading scan results.
    gpio.moder.modify(|_, w| {
        w.moder0().output();
        w.moder1().output();
        w.moder2().output();
        w.moder3().output();
        w.moder4().output();
        w.moder5().output();
        w.moder6().output();
        w.moder7().output();
        w
    });
}

#[derive(Copy, Clone, Debug, Default)]
struct Debounce {
    stable_state: KeyState,
    last_change: Option<TickTime>,
}

impl Debounce {
    const PERIOD: Millis = Millis(5);
    pub fn step(&mut self, input_state: KeyState) -> Option<KeyState> {
        if input_state == self.stable_state {
            // No longer tracking a change.
            self.last_change = None;
        } else if let Some(last_time) = self.last_change {
            if last_time.elapsed() >= Self::PERIOD {
                // We're changing!
                self.stable_state = input_state;
                self.last_change = None;
                return Some(input_state);
            }
        } else {
            // New change to track.
            self.last_change = Some(TickTime::now());
        }

        None
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum KeyState { #[default] Up, Down }

/// Scans a keypad matrix of up to 64 keys (well, 56, really) using the geometry
/// defined in `config`.
///
/// The key detected by driving line X and noticing a change on line Y is mapped
/// to bit `8 * X + Y` of the returned `u64`. That bit will be 1 if the key was
/// seen down, 0 if not.
///
/// Prereqs:
/// - All scan lines are configured for OTYPE=Open Drain
/// - All scan lines are configured for MODE=Output
/// - All scan lines are configured for PUPD=Up
/// - All scan lines are set to 1 (not driven)
///
/// # Cancellation
///
/// This is cancel-safe in the strict sense: if the scan is canceled, all row
/// pins are returned to idle state (weak pull up) to ensure the _next_ scan can
/// work.
async fn scan(config: &Config, gpio: &device::GPIOA) -> u64 {
    let mut down_mask = 0;

    for line in 0..8 {
        let line_mask = 1 << line;
        if config.driven_lines & line_mask == 0 {
            continue;
        }
        // Drive the line low.
        gpio.bsrr.write(|w| unsafe {
            // Set the corresponding RESET bit to pull the pin low.
            w.bits(u32::from(line_mask) << 16)
        });
        // Make sure this pin gets released if we're canceled in the yield
        // below.
        let pin_guard = scopeguard::guard((), |_| {
            // Release the row line by setting the corresponding SET bit.
            gpio.bsrr.write(|w| unsafe { w.bits(u32::from(line_mask)) });
        });

        // Sleep a bit to allow charge to move around. (CANCEL POINT)
        //lilos::exec::yield_cpu().await;
        STATE.store(4, Ordering::SeqCst);
        lilos::exec::sleep_for(Millis(2)).await;
        STATE.store(5, Ordering::SeqCst);

        // Collect return states. Inactive lines will be pulled up by their
        // resistors, active columns will be pulled down by the driven pin. So
        // we invert here.
        let return_states = gpio.idr.read().bits() as u8 ^ 0xFF;
        // Discharge the guard so we release the pin and the matrix can
        // start returning to idle charge.
        drop(pin_guard);

        // Pick over our results.
        let ghost_mask = config.ghost_mask[line];
        for col in 0..8 {
            if col == line {
                // Trivial ghost key, skip
                continue;
            }

            let col_mask = 1 << col;
            if ghost_mask & col_mask != 0 {
                // Configured ghost key, skip
                continue;
            }
            if return_states & col_mask != 0 {
                // Oh hey! A non-ghost key!
                down_mask |= 1 << (8 * line + col);
            }
        }
    }

    down_mask
}
