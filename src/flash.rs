// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! STM32G0/C0 in-application flash programming driver.
//!
//! Citations to RM here refer to the STM32G0 Reference Manual (RM0454),
//! revision 5. The C0 has essentially the same flash controller, so the
//! references work for both chips; only the clock setup is different.
//!
//! # Flash storage format
//!
//! The flash configuration is stored as a series of u64s to make the flash
//! programming interface simpler. It consists of:
//!
//! - 0: A header word
//!     - Top 32 bits: 0xCEEBAD00
//!     - Bits 15:8: sequence number
//!     - Bits 7:0: drive line mask
//! - 1: A word containing the ghost mask
//!     - Starting from the LSB
//! - 2-9: Eight words containing the keymap
//!     - Not sparse, totally profligate, but predictably sized
//! - 10: A trailer word
//!     - Should match the header.
//!
//! Currently there's no CRC or anything. We're attempting to catch failed
//! partial writes and erased sectors, not corruption. That's what ECC is for.
//!
//! # Flash storage location
//!
//! There are two flash locations used in a ping-pong fashion, so that a power
//! loss or crash while writing the flash config will fall back to the previous
//! config.
//!
//! From Rust they're `STORAGE_PAGE_A` and `STORAGE_PAGE_B`. If both appear
//! valid (per the format above) we choose the one with the greater sequence
//! number. Under normal operation the sequence numbers differ by at most 1, but
//! the "greater sequence number" comparison is defined using circular
//! arithmetic, so 1 is considered greater than 255 (for instance).
//!
//! The concrete location of the flash storage is chosen in the linker script.

use core::{ptr::addr_of_mut, sync::atomic::{AtomicBool, Ordering}};
use lilos::atomic::AtomicExt;

use crate::{device, scanner};

/// RAM representation of the configuration we store in Flash. This isn't
/// represented the same way as Flash because we don't generate direct
/// references to config pages in Flash (for aliasing reasons).
#[derive(Debug)]
pub struct SystemConfig {
    /// Configuration for the keyboard scanner.
    pub scanner: scanner::Config,
    /// Mapping from scan coordinate to ASCII character.
    pub keymap: [[u8; 8]; 8],
}

impl SystemConfig {
    /// `Default` can't be used from a const fn, incl. a static initializer, so,
    /// here.
    pub const DEFAULT: Self = Self {
        scanner: scanner::Config::DEFAULT,
        keymap: [[0; 8]; 8],
    };
}

impl Default for SystemConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Wraps the flash controller and references two storage areas.
///
/// # Safety
///
/// To avoid violating aliasing rules, the flash storage areas must not overlap
/// one another, and must not overlap any other flash data structure (such as
/// code or read-only data used by the program). The linker script ensures this.
///
/// They must also each be aligned to a 2 kiB page boundary. The linker script,
/// again, ensures this.
pub struct Storage {
    flash: device::flash::Flash,
    pages: [*mut [u64; 256]; 2],
}

impl Storage {
    /// Takes ownership of the flash controller and sets up initial state for
    /// accessing flash storage.
    ///
    /// # Panics
    ///
    /// This can only be called once in the lifetime of the program. Otherwise,
    /// you could wind up with two `Storage` drivers fighting. It's possible,
    /// using the metapac API, to generate a second pointer to the flash
    /// controller and call `Storage::new` a second time. This routine will
    /// detect that and panic.
    ///
    /// So, don't do that.
    pub fn new(flash: device::flash::Flash) -> Self {
        static STORAGE_TAKEN: AtomicBool = AtomicBool::new(false);
        if STORAGE_TAKEN.swap_polyfill(true, Ordering::SeqCst) {
            panic!()
        }

        extern "C" {
            static mut STORAGE_PAGE_A: [u64; 256];
            static mut STORAGE_PAGE_B: [u64; 256];
        }

        Self {
            flash,
            pages: [
                // Safety: the code above ensures we only get to this point
                // once, so the &mut won't alias; we happen to know that these
                // extern "C" variables are not being modified outside of Rust,
                // so the meaning of &mut won't be violated. Finally, we don't
                // actually use these as &mut ever, so all this is a little bit
                // overkill, probably.
                unsafe { addr_of_mut!(STORAGE_PAGE_A) },
                // Safety: same as line above.
                unsafe { addr_of_mut!(STORAGE_PAGE_B) },
            ],
        }
    }

    /// Checks flash storage for valid configurations and, if any exist, reads
    /// out the most recent one into `out`, overwriting its prior contents.
    ///
    /// If a config is found and `out` is overwritten, returns `true`.
    ///
    /// Otherwise, the contents of `out` are preserved, and this routine returns
    /// `false`.
    pub fn load_active_config(&self, out: &mut SystemConfig) -> bool {
        let Some((i, _)) = self.active() else { return false; };
        self.read_config(i, out);
        true
    }

    /// Persists the contents of `src` into one of the flash storage pages.
    ///
    /// Flash storage page is chosen as follows:
    /// 1. If two valid configs are already in flash, this will overwrite the
    ///    older one.
    /// 2. If there is only one valid config, this will use the other, free
    ///    page.
    /// 3. If no config is present, this will use page A.
    pub fn write_config(&mut self, src: &SystemConfig) {
        let (i, serial) = match self.active() {
            Some(result) => result,
            // Choose page A and set "previous serial" to FF, so we start at 0.
            None => (0, 0xFF),
        };
        // Flip to other page.
        let i = i ^ 1;
        // Advance the serial.
        let serial = serial.wrapping_add(1);

        let base = self.pages[i] as *mut u64;
        let page_offset = (base as u32).wrapping_sub(0x08000000);

        // If debug assertions are enabled, check the properties our linker
        // script is supposed to ensure.
        debug_assert!(page_offset >= 28 * 1024 && page_offset < 32 * 1024);
        debug_assert!(page_offset & 0x7FF == 0);

        let page_index = u8::try_from(page_offset >> 11).unwrap();
        let header = 0xCEEBAD01_0000_0000
            | u64::from(src.scanner.driven_lines)
            | u64::from(serial) << 8;

        unlock(self.flash);

        page_erase(self.flash, page_index);
        enable_programming(self.flash);
        unsafe {
            program(self.flash, base, header);
            program(self.flash, base.add(1), u64::from_le_bytes(src.scanner.ghost_mask));
        }
        for (i, chunk) in src.keymap.iter().enumerate() {
            let word = u64::from_le_bytes(*chunk);
            unsafe {
                program(self.flash, base.add(2 + i), word);
            }
        }
        unsafe {
            program(self.flash, base.add(10), header);
        }
        disable_programming(self.flash);
        lock(self.flash);
    }

    /// Reads a specific config index `i` into `out`. The index must be either 0
    /// or 1.
    fn read_config(&self, i: usize, out: &mut SystemConfig) {
        let base = self.pages[i] as *mut u64;
        let (header, mask) = unsafe {
            (
                core::ptr::read_volatile(base),
                core::ptr::read_volatile(base.add(1)),
            )
        };
        out.scanner.driven_lines = header as u8;
        out.scanner.ghost_mask = mask.to_le_bytes();

        for (i, chunk) in out.keymap.iter_mut().enumerate() {
            let map_word = unsafe {
                core::ptr::read_volatile(base.add(2 + i))
            };
            *chunk = map_word.to_le_bytes();
        }
    }

    /// Determines which slot is active, and returns its serial number that was
    /// found to be the larger of the two.
    fn active(&self) -> Option<(usize, u8)> {
        match (self.serial(0), self.serial(1)) {
            (Some(a), Some(b)) => {
                let delta = (a as i8).wrapping_sub(b as i8);
                if delta > 0 {
                    Some((0, a))
                } else {
                    Some((1, b))
                }
            }
            (Some(a), _) => Some((0, a)),
            (_, Some(b)) => Some((1, b)),
            (_, _) => None,
        }
    }

    /// Loads the serial number from a given slot.
    fn serial(&self, i: usize) -> Option<u8> {
        let base = self.pages[i] as *mut u64;
        let (header, trailer) = unsafe {
            (
                core::ptr::read_volatile(base),
                core::ptr::read_volatile(base.add(10)),
            )
        };
        if header & 0xFFFF_FFFF_FFFF_0000 != 0xCEEB_AD01_0000_0000 {
            return None;
        }
        if header != trailer {
            return None;
        }

        Some((header >> 8) as u8)
    }
}

// General G0/C0 flash bits, not specific to our storage scheme.

/// Perform the flash controller unlock dance, as specified by RM 3.3.6.
/// Necessary before doing any mutations to flash.
fn unlock(flash: device::flash::Flash) {
    flash.keyr().write(|w| {
        w.set_keyr(0x4567_0123);
    });
    flash.keyr().write(|w| {
        w.set_keyr(0xCDEF_89AB);
    });
}

/// Re-locks flash so mutations will be refused.
fn lock(flash: device::flash::Flash) {
    flash.cr().modify(|w| w.set_lock(true));
}

/// Clear any errors hanging around.
fn clear_flags(flash: device::flash::Flash) {
    // These bits are all write-one-to-clear.
    flash.sr().write(|w| {
        w.set_optverr(true);
        w.set_fasterr(true);
        w.set_miserr(true);
        w.set_pgserr(true);
        w.set_sizerr(true);
        w.set_pgaerr(true);
        w.set_wrperr(true);
        w.set_progerr(true);
        w.set_operr(true);
        w.set_eop(true);
    });
}

/// Blocks (the entire CPU) until the flash controller is not busy. This is a
/// reasonable decision because the wait periods of the flash controller are
/// at most small numbers of cycles -- far smaller than our timer resolution.
fn wait_for_not_busy(flash: device::flash::Flash) {
    // Check that no operation is in progress.
    while flash.sr().read().bsy() {
        // spin.
    }
}

/// Kicks off an erase of a page by index and then blocks until it completes.
fn page_erase(flash: device::flash::Flash, page: u8) {
    // As specified by RM 3.3.7

    wait_for_not_busy(flash);
    clear_flags(flash);

    // Set PER and PNB in CR
    flash.cr().modify(|w| {
        w.set_per(true);
        w.set_pnb(page);
    });
    // Set START in CR. The manual strongly suggests that this needs to happen
    // on a second write, but I haven't tested it.
    flash.cr().modify(|w| {
        w.set_strt(true);
    });
    // Wait until BSY1 clears.
    while flash.sr().read().bsy() {
        // spin.
    }
    flash.cr().modify(|w| w.set_per(false));
}

/// Configures the flash controller to allow data to be written. The controller
/// must have previously been `unlock`-d.
fn enable_programming(flash: device::flash::Flash) {
    wait_for_not_busy(flash);
    clear_flags(flash);
    flash.cr().modify(|w| w.set_pg(true));
}

fn disable_programming(flash: device::flash::Flash) {
    flash.cr().modify(|w| w.set_pg(false));
}

/// Programs a 64-bit word into flash at `address`.
///
/// # Safety
///
/// This operation is essentially equivalent to `ptr::write`, and requires the
/// same assurances:
///
/// - `address` must not alias any references that exist in the program.
/// - `address` must be properly aligned and non-null
unsafe fn program(flash: device::flash::Flash, address: *mut u64, source: u64) {
    // We're using the more flexible flash programming interface that works in
    // 64-bit chunks. It's very picky about the order of writes within the
    // 64-bit chunks, however, so we're going to dissect the double-word and
    // write it manually thus:
    let low = source as u32;
    let high = (source >> 32) as u32;
    core::ptr::write_volatile(address as *mut u32, low);
    core::ptr::write_volatile((address as *mut u32).add(1), high);
    wait_for_not_busy(flash);

    let sr = flash.sr().read();
    // EOP won't be set if we're not requesting an interrupt with EOPIE, which
    // is weird and inconsistent with the other peripherals, but ok. Instead,
    // we'll check that no errors have occurred.
    //
    // Since the flash controller is not defined as _capable_ of producing
    // errors at this point, we'll just throw up our hands and crash if one
    // occurs. Wrong microcontroller model maybe?
    if sr.0 & 0x3FA != 0 {
        panic!("SR: {:08x}", sr.0);
    }
}
