//! STM32G0 in-application flash programming driver.
//!
//! Citations to RM here refer to the STM32G0 Reference Manual (RM0454) revision
//! 5.
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

use core::ptr::addr_of_mut;

use crate::{device, scanner};

fn unlock(flash: &device::FLASH) {
    // Perform the flash controller unlock dance, as specified by RM 3.3.6
    flash.keyr.write(|w| unsafe {
        w.bits(0x4567_0123)
    });
    flash.keyr.write(|w| unsafe {
        w.bits(0xCDEF_89AB)
    });
}

fn lock(flash: &device::FLASH) {
    flash.cr.modify(|_, w| w.lock().set_bit());
}

fn clear_flags(flash: &device::FLASH) {
    // Clear any errors hanging around. These bits are all write-one-to-clear.
    flash.sr.write(|w| {
        w.optverr().set_bit();
        w.fasterr().set_bit();
        w.miserr().set_bit();
        w.pgserr().set_bit();
        w.sizerr().set_bit();
        w.pgaerr().set_bit();
        w.wrperr().set_bit();
        w.progerr().set_bit();
        w.operr().set_bit();
        w.eop().set_bit();
        w
    });
}

fn wait_for_not_busy(flash: &device::FLASH) {
    // Check that no operation is in progress.
    while flash.sr.read().bsy().bit_is_set() {
        // spin.
    }

}

fn page_erase(flash: &device::FLASH, page: u8) {
    // As specified by RM 3.3.7

    wait_for_not_busy(flash);
    clear_flags(flash);

    // Set PER and PNB in CR
    flash.cr.modify(|_, w| {
        w.per().set_bit();
        unsafe {
            w.pnb().bits(page);
        }
        w
    });
    // Set START in CR. The manual strongly suggests that this needs to happen
    // on a second write, but I haven't tested it.
    flash.cr.modify(|_, w| w.strt().set_bit());
    // Wait until BSY1 clears.
    while flash.sr.read().bsy().bit_is_set() {
        // spin.
    }
    flash.cr.modify(|_, w| w.per().clear_bit());
}

fn enable_programming(flash: &device::FLASH) {
    wait_for_not_busy(flash);
    clear_flags(flash);
    flash.cr.modify(|_, w| w.pg().set_bit());
}

fn disable_programming(flash: &device::FLASH) {
    flash.cr.modify(|_, w| w.pg().clear_bit());
}

unsafe fn program(flash: &device::FLASH, address: *mut u64, source: u64) {
    // We're using the more flexible flash programming interface that works in
    // 64-bit chunks. It's very picky about the order of writes within the
    // 64-bit chunks, however, so we're going to dissect the double-word and
    // write it manually thus:
    let low = source as u32;
    let high = (source >> 32) as u32;
    core::ptr::write_volatile(address as *mut u32, low);
    core::ptr::write_volatile((address as *mut u32).add(1), high);
    wait_for_not_busy(flash);

    let sr = flash.sr.read();
    // EOP won't be set if we're not requesting an interrupt with EOPIE, which
    // is weird and inconsistent with the other peripherals, but ok. Instead,
    // we'll check that no errors have occurred.
    if sr.bits() & 0x3FA != 0 {
        panic!("SR: {:08x}", sr.bits());
    }
}

#[derive(Debug)]
pub struct SystemConfig {
    pub scanner: scanner::Config,
    pub keymap: [[u8; 8]; 8],
}

impl Default for SystemConfig {
    fn default() -> Self {
        Self {
            scanner: scanner::Config::default(),
            keymap: [[0; 8]; 8],
        }
    }
}

pub struct Storage {
    flash: device::FLASH,
    pages: [*mut [u64; 256]; 2],
}

impl Storage {
    pub fn new(flash: device::FLASH) -> Self {
        Self {
            flash,
            pages: [
                unsafe { addr_of_mut!(STORAGE_PAGE_A) },
                unsafe { addr_of_mut!(STORAGE_PAGE_B) },
            ],
        }
    }

    pub fn load_active_config(&self, out: &mut SystemConfig) -> bool {
        let Some((i, _)) = self.active() else { return false; };
        self.read_config(i, out);
        true
    }

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

    pub fn write_config(&mut self, src: &SystemConfig) {
        let (i, serial) = match self.active() {
            Some(result) => result,
            None => (0, 0xFF),
        };
        // Flip to other page.
        let i = i ^ 1;
        // Advance the serial.
        let serial = serial.wrapping_add(1);

        let base = self.pages[i] as *mut u64;
        let page_offset = (base as u32).wrapping_sub(0x08000000);
        debug_assert!(page_offset >= 28 * 1024 && page_offset < 32 * 1024);
        debug_assert!(page_offset & 0x7FF == 0);
        let page_index = u8::try_from(page_offset >> 11).unwrap();
        let header = 0xCEEBAD01_0000_0000
            | u64::from(src.scanner.driven_lines)
            | u64::from(serial) << 8;

        unlock(&self.flash);

        page_erase(&self.flash, page_index);
        enable_programming(&self.flash);
        unsafe {
            program(&self.flash, base, header);
            program(&self.flash, base.add(1), u64::from_le_bytes(src.scanner.ghost_mask));
        }
        for (i, chunk) in src.keymap.iter().enumerate() {
            let word = u64::from_le_bytes(*chunk);
            unsafe {
                program(&self.flash, base.add(2 + i), word);
            }
        }
        unsafe {
            program(&self.flash, base.add(10), header);
        }
        disable_programming(&self.flash);
        lock(&self.flash);
    }
}

extern "C" {
    static mut STORAGE_PAGE_A: [u64; 256];
    static mut STORAGE_PAGE_B: [u64; 256];
}
