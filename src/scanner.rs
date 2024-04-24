// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

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

use device::gpio::vals::{Ot, Pupdr, Moder};
use lilos::spsc;
use lilos::time::{TickTime, Millis, PeriodicGate};

use crate::device;
use crate::util::StaticResource;

/// Wanna alter the scan interval? Well here it is.
const SCAN_INTERVAL: Millis = Millis(1);

/// Debouncers are kept in static RAM because (1) they're relatively large and
/// might be a surprising thing to find on the stack, and (2) this way they're
/// easily visible in a debugger.
static DEBOUNCERS: StaticResource<[[Debounce; 8]; 8]> = StaticResource::new(
    [[Debounce::DEFAULT; 8]; 8]
);

/// Scanner configuration state.
#[derive(Copy, Clone, Debug)]
pub struct Config {
    /// Repeated in events to distinguish events derived from this config from
    /// those derived from some previous config. (This only matters during
    /// online reconfiguration.)
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

impl Config {
    /// The default for `Config` causes no lines to be driven. This is the
    /// conservative option if we don't know anything about the outside world.
    /// This only applies during the brief window at the start of setup, before
    /// the setup program starts overriding the config.
    ///
    /// Having this value be all zeroes also ensures that statics that include
    /// it go in BSS, rather than getting an init image.
    ///
    /// This is a const because it needs to appear in static initializers, where
    /// `Default` can't yet go.
    pub const DEFAULT: Self = Self {
        epoch: 0,
        driven_lines: 0,
        ghost_mask: [0; 8],
    };
}

impl Default for Config {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Event produced, and stuffed into a queue, when something happens.
#[derive(Copy, Clone, Debug)]
pub struct KeyEvent {
    /// Epoch of config that generated this event.
    pub epoch: u8,
    /// Designates a matrix position by encoding the driven line (0-7) in the
    /// least significant 3 bits, and the received line (0-7) in the next-most
    /// significant 3 bits, producing a number between 0 and 63, inclusive.
    ///
    /// (See `driven_line` and `sensed_line` for help here.)
    pub coord: u8,
    /// What state the key was determined to be in, now.
    pub state: KeyState,
}

impl KeyEvent {
    /// Extracts the index (0-7) of the line that was being driven when this
    /// event was sensed.
    pub fn driven_line(&self) -> usize {
        usize::from(self.coord & 0x7)
    }
    /// Extracts the index (0-7) of the line that was sensed, causing this
    /// event.
    pub fn sensed_line(&self) -> usize {
        usize::from((self.coord >> 3) & 0x7)
    }
}

/// States keys can be in.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub enum KeyState { #[default] Up, Down }

/// Keypad scanner loop.
///
/// Pass this its initial `config` plus a handoff channel through which it can
/// receive updated configs. It'll scan the provided GPIO port (pins 0-7) and
/// emit events through `out_queue`.
pub async fn task(
    mut config: Config,
    mut config_update: lilos_handoff::Popper<'_, Config>,
    gpio: device::gpio::Gpio,
    mut out_queue: spsc::Pusher<'_, KeyEvent>,
) -> Infallible {
    configure_pins(gpio);

    let debouncers = DEBOUNCERS.take();
    let mut scan_gate = PeriodicGate::from(SCAN_INTERVAL);
    loop {
        scan_gate.next_time().await;

        // Process any config update before starting the scan.
        if let Some(new_config) = config_update.try_pop() {
            // Handle any "killed" drive lines. This occurs when the
            // `driven_lines` bit was 1 in the old config, and is becoming 0.
            // When a drive line is killed we need to reset the state of its
            // debouncers so they don't do anything.
            for (line, debounce_row) in debouncers.iter_mut().enumerate() {
                let line_mask = 1 << line;
                let in_use_before = config.driven_lines & line_mask != 0;
                let still_in_use = new_config.driven_lines & line_mask != 0;
                if in_use_before && !still_in_use {
                    for debouncer in debounce_row {
                        *debouncer = Default::default();
                    }
                }
            }
            // Handle any "killed" keys. This occurs when the key's bit in the
            // `ghost_mask` was not set before, but has become set. As in the
            // killed line case, we need to reset its debouncer state.
            //
            // This is written using a range and indexing because, believe it or
            // not, enumerate generates much larger code -- at least as of
            // 1.75.0. This is because we're at opt-level="z", which makes dumb
            // decisions like "not inlining the enumerate iterator for slices."
            #[allow(clippy::needless_range_loop)]
            for line in 0..8 {
                for col in 0..8 {
                    let col_mask = 1 << col;
                    let real_before = config.ghost_mask[line] & col_mask == 0;
                    let still_real = new_config.ghost_mask[line] & col_mask != 0;
                    if real_before && !still_real {
                        debouncers[line][col] = Default::default();
                    }
                }
            }
            // Apply the config.
            config = new_config;
        }
        // Config doesn't get modified anywhere else in this loop -- see:
        let config = &config;

        // Run the scan algorithm to determine what connections are currently
        // live.
        let down_mask = scan(config, gpio).await;

        // Process each potential key in the grid. This is written using a range
        // and indexing because, believe it or not, enumerate generates much
        // larger code -- at least as of 1.75.0. This is because we're at
        // opt-level="z", which makes dumb decisions like "not inlining the
        // enumerate iterator for slices."
        #[allow(clippy::needless_range_loop)]
        for line in 0..8 {
            for col in 0..8 {
                let s = if down_mask & (1 << (8 * line + col)) != 0 {
                    KeyState::Down
                } else {
                    KeyState::Up
                };
                if let Some(new_state) = debouncers[line][col].step(s) {
                    // Try to send the key event, but if the queue fills up,
                    // tolerate that and move on. This means we won't block
                    // keyboard scanning in the event of backpressure (the
                    // consumer failing to keep up with our generated events).
                    // We've gotta make this decision _somewhere_ and this is
                    // the simplest place. In practice, our consumer is real
                    // fast and can't block.
                    out_queue.try_push(KeyEvent {
                        epoch: config.epoch,
                        coord: (line | (col << 3)) as u8,
                        state: new_state,
                    }).ok();
                }
            }
        }

    }
}

fn configure_pins(gpio: device::gpio::Gpio) {
    // Ensure all scan pins are open drain (only drive low)
    gpio.otyper().modify(|w| {
        for pin in 0..=7 {
            w.set_ot(pin, Ot::OPENDRAIN);
        }
    });
    // Activate weak pullups.
    gpio.pupdr().modify(|w| {
        for pin in 0..=7 {
            w.set_pupdr(pin, Pupdr::PULLUP);
        }
    });

    // Ensure all scan pins are initially driven high (i.e. not driven)
    gpio.odr().modify(|w| w.0 |= 0xFF);

    // Switch pins to outputs. Note that this also enables the schmitt trigger
    // input, which is important for reading scan results.
    gpio.moder().modify(|w| {
        for pin in 0..=7 {
            w.set_moder(pin, Moder::OUTPUT);
        }
    });
}

/// State maintained for debouncing a single key.
#[derive(Copy, Clone, Debug, Default)]
struct Debounce {
    /// We're pretty sure the key has been hanging out in this state.
    stable_state: KeyState,
    /// If we've seen the key leave `stable_state`, this will be `Some`
    /// containing the timestamp when that happened. If the key goes back to
    /// `stable_state` we'll clear this back to `None`. Debouncing happens by
    /// noticing that this is `Some` and that the timestamp has gotten old
    /// enough.
    last_change: Option<TickTime>,
}

impl Debounce {
    /// How long a key is required to be stable before we commit to it.
    ///
    /// This also serves to limit the maximum frequency of events emitted by the
    /// scanner, and was chosen to be somewhat larger than
    ///
    ///     16 * BITS_PER_FRAME / 19_200
    ///
    /// ...where `BITS_PER_FRAME` is 11. This ensures that one deboucing
    /// interval is long enough to emit 16 key events on the serial interface.
    const PERIOD: Millis = Millis(10);

    /// Default const for static initializers.
    const DEFAULT: Self = Self {
        stable_state: KeyState::Up,
        last_change: None,
    };

    /// Move the debouncing state machine forward. `input_state` is the observed
    /// electrical state of the key right now. If the state machine decides a
    /// real key change has occurred, it will return `Some` with the new state.
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
async fn scan(config: &Config, gpio: device::gpio::Gpio) -> u64 {
    let mut down_mask = 0;

    for line in 0..8 {
        let line_mask = 1 << line;
        // Skip any lines we don't drive. (This processor doesn't have a cheap
        // count-trailing-zeros operation so we do this the hard way.)
        if config.driven_lines & line_mask == 0 {
            continue;
        }
        // Drive the line low.
        gpio.bsrr().write(|w| {
            // Set the corresponding RESET bit to pull the pin low.
            w.0 = u32::from(line_mask) << 16;
        });
        // Make sure this pin gets released if we're canceled in the yield
        // below.
        let return_states = {
            scopeguard::defer! {
                // Release the row line by setting the corresponding SET bit.
                gpio.bsrr().write(|w| w.0 = u32::from(line_mask));
            }

            // Sleep a bit to allow charge to move around. (CANCEL POINT)
            //
            // This sleep is long enough for all the matrices I've tested, but
            // is not of any particular defined period -- it depends on the
            // implementation of the executor and the number of pending tasks.
            // Might be worth considering converting this back to a sleep(1)?
            lilos::exec::yield_cpu().await;

            // Collect return states. Inactive lines will be pulled up by their
            // resistors, active columns will be pulled down by the driven pin.
            // So we invert here.
            !(gpio.idr().read().0 as u8)
            // The deferred block above will fire here, releasing the pin and
            // returning the matrix to idle charge level.
        };

        // Pick over our results.
        //
        // Get the configured ghost keys for this line, and add in the bit
        // corresponding to the trivial "pin drives itself" net. (This isn't
        // permanently set in the ghost mask in RAM because that'd require its
        // initializer to be something other than all zeroes, forcing more data
        // into flash.)
        let ghost_mask = config.ghost_mask[line] | (1 << line);
        // Handle all of the columns in parallel.
        //
        // For each column we want to set a bit in down_mask if the
        // corresponding bit in return_states is set, _unless_ that bit is also
        // set in ghost_mask. This comes out to (pretending for the moment that
        // we can index bits in bytes like in Verilog):
        //
        // !ghost_mask[col] & return_states[col]
        //
        // ...which is equivalent to the byte-level logical operation.
        let line_state = return_states & !ghost_mask;
        down_mask |= u64::from(line_state) << (8 * line);
    }

    down_mask
}
