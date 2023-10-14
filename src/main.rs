// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Keyboard scanner firmware entry point.
//!
//! # Target architecture
//!
//! This firmware supports both STM32G030 and STM32C011 microcontrollers, to
//! keep costs down. This has introduced some complexity. In cases where you
//! encounter a `cfg(feature = "stm32c011")` or `cfg(feature = "stm32g030")`,
//! that's providing specific code for one microcontroller or another.
//!
//! Note that these microcontrollers are pretty similar, but are _not_ binary
//! compatible, so if you flash the build for the wrong chip, things will go
//! poorly and you'll need a SWD interface to recover.

#![no_std]
#![no_main]

// The default production config halts on panic, excluding the formatting
// machinery and saving a solid 7 kiB.
#[cfg(feature = "panic-halt")]
extern crate panic_halt;

// Enable this instead to get nicely formatted panics through your debug probe.
#[cfg(feature = "panic-semihosting")]
extern crate panic_semihosting;

mod serial;
mod scanner;
mod flash;
mod i2c;

use stm32_metapac as device;

use core::{mem::MaybeUninit, pin::pin};
use cortex_m_rt::pre_init;
use device::{gpio::vals::{Pupdr, Moder, Idr}, rcc::regs::Gpioenr, flash::vals::Latency, syscfg::vals::MemMode};
use lilos::{handoff::Handoff, spsc::Queue};

use crate::flash::SystemConfig;

#[cortex_m_rt::entry]
fn main() -> ! {
    // Safety: as long as this only happens in main we won't anger cortex_m's
    // notion of peripheral exclusiveness. Doing it this way avoids compiling in
    // the panic that they otherwise unconditionally include.
    let mut cp = unsafe { cortex_m::Peripherals::steal() };

    // Shorthand!
    let rcc = device::RCC;
    let gpioa = device::GPIOA;
    let gpiob = device::GPIOB;
    let gpioc = device::GPIOC;

    // Clock situation:
    // G0: We are currently at 16 MHz on HSI.
    // C0: We are currently at 12 MHz on HSI48/4.

    // Turn on the I/O ports we use. If we wind up jumping to the bootloader
    // instead, we'll reverse this.
    rcc.gpioenr().modify(|w| {
        w.set_gpioaen(true);
        w.set_gpioben(true);
        w.set_gpiocen(true);
    });
    cortex_m::asm::dsb();

    // Switch the two control buttons to pulled-up inputs.
    gpioc.pupdr().modify(|w| {
        w.set_pupdr(14, Pupdr::PULLUP); // Update button
        w.set_pupdr(15, Pupdr::PULLUP); // Setup button
    });
    gpioc.moder().modify(|w| {
        w.set_moder(14, Moder::INPUT); // Update button
        w.set_moder(15, Moder::INPUT); // Setup button
    });
    // Delay long enough for mode sense pins to charge. TODO: this choice, which
    // is about 1.25 ms, is somewhat arbitrary.
    cortex_m::asm::delay(20_000);
    // Read mode sense pins.
    let (update_mode, mut setup_mode) = {
        let idr = gpioc.idr().read();
        (idr.idr(14) == Idr::LOW, idr.idr(15) == Idr::LOW)
    };

    // Go ahead and reset port C. We need to do it before jumping into the
    // bootloader, and we ought to do it before starting the rest of our work,
    // so, why not do it here?
    gpioc.moder().write_value(device::gpio::regs::Moder(0xFFFFFFFF));
    gpioc.pupdr().write_value(device::gpio::regs::Pupdr(0));

    // Handle update request.
    if update_mode {
        do_update(cp, rcc);
    }

    // Now we can mess around with machine state to our heart's content, since
    // we won't need to precisely reverse the changes to avoid confusing ST's
    // boot ROM!
    //
    // Let's go faster.
    clock_setup();
    const CLOCK_HZ: u32 = 48_000_000;

    // For compactness (in flash) we're going to turn on the peripherals we use
    // here, in one block, instead of in each driver.
    //
    // This decision may turn out to be silly, given how much flash async fns
    // burn.
    rcc.apbenr2().write(|w| {
        w.set_usart1en(true);
        w.set_syscfgen(true);
    });

    cortex_m::asm::dsb(); // probably not necessary on M0? Eh, whatev

    // Pin configuration:
    //
    // PA0-7: keyboard matrix GPIO
    // PA8: debug/logic analyzer output
    // PA11[PA9]: as PA9, USART1_TX (AF1)
    // PA12[PA10]: as PA10, USART1_RX (AF1)
    // PA13: SWDIO (default)
    // PA14: SWCLK (default)
    //
    // PB6: I2C1 SDA (AF6)
    // PB7: I2C1 SCL (AF6)
    //
    // PC14: Update button (active low, needs pullup to read)
    // PC15: Setup button (active low, needs pullup to read)
    //
    // NOTE: we do _not_ set pins to output/alternate here because we may want
    // to do tests first.

    // Expose PA9/PA10 instead of PA11/PA12.
    device::SYSCFG.cfgr1().write(|w| {
        // TODO metapac models these bits wrong
        w.0 = w.0 | (1 << 4) | (1 << 3);
        // This should be implicit in the definition of write, but I'm still
        // working on understanding the value defaults in the metapac.
        w.set_mem_mode(MemMode(0));
    });

    device::GPIOA.afr(1).write(|w| {
        w.set_afr(9 - 8, 1); // USART1_TX
        w.set_afr(10 - 8, 1); // USART1_RX

        // Since we're using write instead of modify to save space, this will
        // change _all pins on the port_ ... which includes the SWD pins.
        // However, their default of AF0 is correct for keeping the debug port
        // open, so, no action required.
    });

    device::GPIOB.afr(0).write(|w| {
        w.set_afr(6, 6); // I2C1_SDA
        w.set_afr(7, 6); // I2C1_SCL
    });
    // Use PA8 as profiling output.
    device::GPIOA.moder().modify(|w| {
        w.set_moder(8, Moder::OUTPUT);
    });

    let storage = flash::Storage::new(device::FLASH);

    // Ensure that the RAM config goes somewhere I can find in a debugger! i.e.
    // somewhere with a name, off the stack.
    let cfg = {
        static mut ACTIVE_CONFIG: SystemConfig = SystemConfig::DEFAULT;
        // Safety: we're relying on the fact that this is in main to ensure that
        // it only gets executed once. That is not a spectacularly robust
        // approach to this, but it has the advantage of being essentially free.
        unsafe { &mut ACTIVE_CONFIG }
    };
    // Attempt to override the default config with what we find in flash. If we
    // find nothing in flash, force `setup_mode` to true.
    setup_mode |= !storage.load_active_config(cfg);

    //
    // Task setup starts here-ish
    //

    // Allocate the scanner-to-serial event queue. Small enough that I'm just
    // putting it on the stack.
    let mut scan_event_storage = [MaybeUninit::uninit(); 16];
    let mut scan_event_q = pin!(Queue::new(&mut scan_event_storage));
    let (scan_event_from_scanner, scan_event_to_serial) = scan_event_q.split();

    // Allocate the serial-to-scanner synchronous config handoff (no storage
    // required)
    let mut config_handoff = Handoff::new();
    let (config_to_scanner, config_from_serial) = config_handoff.split();

    // Allocate the serial-to-I2C byte queue, also on the stack.
    let mut i2c_byte_storage = [MaybeUninit::uninit(); 16];
    let mut i2c_byte_q = pin!(Queue::new(&mut i2c_byte_storage));
    let (i2c_byte_from_serial, i2c_byte_to_i2c) = i2c_byte_q.split();

    let serial_task = pin!(serial::task(
        device::USART1,
        gpioa,
        &cfg.keymap,
        setup_mode,
        scan_event_to_serial,
        config_to_scanner,
        i2c_byte_from_serial,
        storage,
    ));

    let scanner_task = pin!(scanner::task(
        cfg.scanner,
        config_from_serial,
        gpioa,
        scan_event_from_scanner,
    ));

    let i2c_task = pin!(i2c::task(
        rcc,
        gpiob,
        device::I2C1,
        i2c_byte_to_i2c,
    ));

    // Set up and run the scheduler.
    lilos::time::initialize_sys_tick(&mut cp.SYST, CLOCK_HZ);
    lilos::exec::run_tasks_with_idle(
        &mut [
            serial_task,
            scanner_task,
            i2c_task,
        ],
        lilos::exec::ALL_TASKS,
        || {
            // Uncomment these lines to get "CPU active" signals on PA8 for
            // performance measurement.
            //p.GPIOA.bsrr.write(|w| w.br8().set_bit());
            cortex_m::asm::wfi();
            //p.GPIOA.bsrr.write(|w| w.bs8().set_bit());
        },
    )
}

#[cfg(feature = "stm32g030")]
fn clock_setup() {
    use device::rcc::vals::{Pllsrc, Sw};

    let flash = device::FLASH;
    let rcc = device::RCC;
    // We come out of reset at 16 MHz on HSI. We would like to be running at 48
    // MHz (not 64, because we'd like to simulate the limitations of the
    // STM32C0).
    //
    // This implies that we need to boost the clock speed by 4 using the PLL.
    //
    // Note: to achieve this, we must be in voltage range 1 (required for
    // frequencies above 16 MHz).
    //
    // The PLL's input frequency can go up to 16 MHz, and its internal VCO must
    // be between 64 and 344 MHz (in voltage range 1). Its R-tap is the one that
    // feeds the CPU and buses, so we need the R output to be 48 MHz. The R
    // divisor is limited to integers between 2 and 8 (inclusive), so, we can
    // get what we want by
    //
    // - Setting the PLL input divisor (VCO multiplier) to 6x for an fVCO of 96
    //   MHz. This requires using the fractional divisor to choose 12/2.
    // - Setting the R divisor to /2 for a 48 MHz output.
    // - Leaving the P and Q taps off. (Q tap doesn't actually exist on our
    //   part.)

    // The part resets into voltage range 1, delightfully. This means we don't
    // have to adjust it.

    // Adjust our wait states to reflect our target voltage + freq combination.
    // At 48 MHz we'll need 1 wait state. We come out of reset at 0 wait states,
    // so we must override it.
    //
    // Note: the SVD apparently incorrectly models an undocumented reserved bit
    // in this register (bit 18), so DO NOT use `write` or `reset`. If you clear
    // bit 18, bad shit will happen to you -- it interferes with debugger
    // access.
    flash.acr().modify(|w| {
        w.set_latency(Latency::WS1);
    });

    // Fire up the PLL.
    // First, set up our divisors.
    rcc.pllsyscfgr().write(|w| {
        // Source input from HSI16.
        w.set_pllsrc(Pllsrc::HSI16);
        // Divide input by 2 to 8 MHz.
        w.set_pllm(2 - 1);
        // Multiply that by 12 in the VCO for 96 MHz.
        w.set_plln(12);
        // Configure the R-tap to 48 MHz output by dividing by 2. Configure P
        // and Q to valid configurations while we're at it.
        w.set_pllr(2 - 1);
        w.set_pllp(2 - 1);
        w.set_pllq(2 - 1);
        // But we only actually turn the R tap on.
        w.set_pllren(true);
    });
    // Switch it on at the RCC and wait for it to come up. RCC should still be
    // at its reset values, so we don't need to RMW it.
    rcc.cr().write(|w| {
        w.set_pllon(true);
    });
    while !rcc.cr().read().pllrdy() {
        // spin
    }
    // Now switch over to it.
    rcc.cfgr().write(|w| {
        w.set_sw(Sw::PLLRCLK);
    });
    while rcc.cfgr().read().sws() != Sw::PLLRCLK {
        // spin
    }
}

#[cfg(feature = "stm32c011")]
fn clock_setup() {
    use device::rcc::vals::Hsidiv;

    let flash = device::FLASH;
    let rcc = device::RCC;
    // We come out of reset at 12 MHz on HSI48. We would like to be running at
    // 48 MHz.
    //
    // This is ... shockingly easy on this part. We just need to add a flash
    // wait state and then change the HSI48 output divider.
    flash.acr().modify(|w| {
        w.set_latency(Latency::WS1);
    });

    while flash.acr().read().latency() != Latency::WS1 {
        // spin
    }

    rcc.cr().write(|w| {
        w.set_hsidiv(Hsidiv::DIV1);
    });
}

/// Runs ST's program instead of ours.
fn do_update(cp: cortex_m::Peripherals, rcc: device::rcc::Rcc) -> ! {
    // Reverse the changes we made to check the button state.
    rcc.gpioenr().write_value(Gpioenr(0));

    // Configure to run the ROM. It appears that the ROM does not require itself
    // to be mapped at low addresses, so we can avoid reconfiguring syscfg.

    let rom_addr = 0x1FFF_0000;

    // Switch the vector table to use the ROM's. TODO: it's not actually clear
    // if the ROM requires this, ST's examples tend not to show it.
    unsafe {
        cp.SCB.vtor.write(rom_addr);
    }
    // Load the stack pointer and reset vector, in that order, from the ROM.
    let stack_pointer = unsafe {
        core::ptr::read_volatile(rom_addr as *const u32)
    };
    let reset_vector = unsafe {
        core::ptr::read_volatile((rom_addr + 4) as *const u32)
    };

    // Chain-load.
    unsafe {
        core::arch::asm!("
            mov sp, {stack_pointer}
            bx {reset_vector}
            ",
            stack_pointer = in(reg) stack_pointer,
            reset_vector = in(reg) reset_vector,
            // "nostack" means we do not use the stack, meaning it does not need to
            // be ABI-aligned on entry; this is trivially true since our first act
            // is to overwrite it.
            //
            // "noreturn" is more obvious.
            options(nostack, noreturn)
        )
    }
}

#[pre_init]
unsafe fn pre_init() {
    // Work around erratum "2.2.5 SRAM write error" where a reset timed _just
    // right_ can cause the first access to an SRAM to be treated as a READ,
    // losing a write.
    #[cfg(feature = "stm32c011")]
    core::arch::asm!("
        ldr r0, =0x20000000
        ldr r0, [r0]
        ",
        out("r0") _,
        options(readonly, preserves_flags, nostack),
    );

    // Using a pre-init hook here to fill the stack with a recognizable bit
    // pattern, to help my fledgling debugger.
    core::arch::asm!("
        @ Linker script marks end of BSS+uninit with __sheap
        ldr r0, =__sheap
        @ We'll work down from the current stack pointer,
        @ _exclusive,_ to avoid corrupting startup state.
        mov r1, sp
        ldr r2, =0xDEDEDEDE

        @ Bump working pointer down
    0:  subs r1, #4
        @ Compare to BSS+uninit end
        cmp r0, r1
        @ Exit if we've passed it
        bhi 1f
        @ Initialize word
        str r2, [r1]
        b 0b
    1:
        ",
        out("r0") _,
        out("r1") _,
        out("r2") _,
        // "readonly" in that we don't write any memory visible to Rust.
        // "nostack" technically requires that we not write the stack redzone --
        // if our architecture had one, we'd be writing it, but it does not, and
        // nostack is required to avoid pushing things to the stack, which would
        // mess up the C0 erratum fix above.
        options(readonly, nostack),
    );
}
