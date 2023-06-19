#![no_std]
#![no_main]

#[cfg(feature = "panic-halt")]
extern crate panic_halt;

#[cfg(feature = "panic-semihosting")]
extern crate panic_semihosting;

mod serial;
mod scanner;
mod flash;

use core::pin::pin;

use enum_map::MaybeUninit;
use lilos::handoff::Handoff;
use stm32g0 as _;
use stm32g0::stm32g030 as device;
use lilos::spsc::Queue;

use crate::flash::SystemConfig;


#[cortex_m_rt::entry]
fn main() -> ! {
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    let p = unsafe { device::Peripherals::steal() };

    // We are currently at 16 MHz on HSI.

    // Turn on the I/O ports we use. If we wind up jumping to the bootloader
    // instead, we'll reverse this.
    p.RCC.iopenr.write(|w| {
        w.iopaen().set_bit();
        w.iopben().set_bit();
        w.iopcen().set_bit();
        w
    });

    // Switch the two control buttons to pulled-up inputs.
    p.GPIOC.pupdr.write(|w| {
        w.pupdr14().pull_up(); // Update button
        w.pupdr15().pull_up(); // Setup button
        w
    });
    p.GPIOC.moder.write(|w| {
        w.moder14().input(); // Update button
        w.moder15().input(); // Setup button
        w
    });
    // Delay long enough for mode sense pins to charge. TODO: this choice, which
    // is about 1.25 ms, is somewhat arbitrary.
    cortex_m::asm::delay(20_000);
    // Read mode sense pins.
    let (update_mode, mut setup_mode) = {
        let idr = p.GPIOC.idr.read();
        (idr.idr14().bit_is_clear(), idr.idr15().bit_is_clear())
    };

    // Go ahead and reset port C. We need to do it before jumping into the
    // bootloader, and we ought to do it before starting the rest of our work,
    // so, why not do it here?
    p.GPIOC.moder.reset();
    p.GPIOC.pupdr.reset();

    // Handle update request.
    if update_mode {
        do_update(cp, p);
    }

    // Now we can mess around with machine state to our heart's content, since
    // we won't need to precisely reverse the changes!
    //
    // Let's go faster.
    clock_setup(&p);
    const CLOCK_HZ: u32 = 48_000_000;

    // For compactness (in flash) we're going to turn on the peripherals we use
    // here, in one block, instead of in each driver.
    //
    // This decision may turn out to be silly, given how much flash async fns
    // burn.
    p.RCC.apbenr2.write(|w| {
        w.usart1en().set_bit();
        w.syscfgen().set_bit();
        w
    });

    cortex_m::asm::dsb(); // probably not necessary on M0?

    // Pin configuration:
    //
    // PA0-7: keyboard matrix GPIO
    // PA8: VCC sense (possibly unnecessary, try using internal source)
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
    p.SYSCFG.cfgr1.write(|w| {
        // fucking PAC missed these bits somehow
        unsafe {
            w.bits((1 << 4) | (1 << 3))
        }
    });

    p.GPIOA.afrh.write(|w| {
        w.afsel9().af1(); // USART1_TX
        w.afsel10().af1(); // USART1_RX

        // Since we're using write instead of modify to save space, this will
        // change _all pins on the port_ ... which includes the SWD pins.
        // However, their reset default of AF0 is correct for keeping the debug
        // port open, so, no action required.
        w
    });

    p.GPIOB.afrl.write(|w| {
        w.afsel6().af6(); // I2C1_SDA
        w.afsel7().af6(); // I2C1_SCL
        w
    });

    let storage = flash::Storage::new(p.FLASH);
    // Ensure that the RAM config goes somewhere I can find in a debugger!
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

    let mut serial_key_storage = [MaybeUninit::uninit(); 16];
    let mut serial_key_q = pin!(Queue::new(&mut serial_key_storage));
    let (serial_key_push, serial_key_pop) = serial_key_q.split();

    let mut config_handoff = Handoff::new();
    let (config_to_scanner, config_from_serial) = config_handoff.split();

    let serial_task = pin!(serial::task(
        &p.USART1,
        &p.GPIOA,
        &cfg.keymap,
        setup_mode,
        serial_key_pop,
        config_to_scanner,
        storage,
    ));

    let scanner_task = pin!(scanner::task(
        cfg.scanner,
        config_from_serial,
        &p.GPIOA,
        serial_key_push,
    ));

    // Set up and run the scheduler.
    lilos::time::initialize_sys_tick(&mut cp.SYST, CLOCK_HZ);
    lilos::exec::run_tasks(
        &mut [
            serial_task,
            scanner_task,
        ],
        lilos::exec::ALL_TASKS,
    )
}

fn clock_setup(p: &device::Peripherals) {
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
    p.FLASH.acr.modify(|_, w| {
        unsafe {
            w.latency().bits(1);
        }
        w
    });

    // Fire up the PLL.
    // First, set up our divisors.
    p.RCC.pllsyscfgr.write(|w| {
        // Source input from HSI16.
        unsafe {
            w.pllsrc().bits(0b10);
        }
        // Divide input by 2 to 8 MHz.
        unsafe {
            w.pllm().bits(2 - 1);
        }
        // Multiply that by 12 in the VCO for 96 MHz.
        unsafe {
            w.plln().bits(12);
        }
        // Configure the R-tap to 48 MHz output by dividing by 2. Configure P
        // and Q to valid configurations while we're at it.
        unsafe {
            w.pllr().bits(2 - 1);
            w.pllp().bits(2 - 1);
            w.pllq().bits(2 - 1);
        }
        // But we only actually turn the R tap on.
        w.pllren().set_bit();
        w
    });
    // Switch it on at the RCC and wait for it to come up. RCC should still be
    // at its reset values, so we don't need to RMW it.
    p.RCC.cr.write(|w| {
        w.pllon().set_bit();
        w
    });
    while p.RCC.cr.read().pllrdy().bit_is_clear() {
        // spin
    }
    // Now switch over to it.
    p.RCC.cfgr.write(|w| {
        unsafe {
            w.sw().bits(0b010);
        }
        w
    });
    while p.RCC.cfgr.read().sws().bits() != 0b010 {
        // spin
    }
}

/// Runs ST's program instead of ours.
fn do_update(cp: cortex_m::Peripherals, p: device::Peripherals) -> ! {
    // Reverse the changes we made to check the button state.
    p.RCC.iopenr.reset();

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
