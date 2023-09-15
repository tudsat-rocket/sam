//! Code for interacting with the STM32's bootloader

use hal::pac::{PWR, RCC, RTC};
use hal::rcc::Enable;
use stm32f4xx_hal as hal;

use cortex_m::interrupt::free;

use crate::log;
use crate::logging::*;

#[no_mangle]
pub fn jump_to_bootloader() {
    unsafe {
        core::arch::asm!("ldr r0, =0x40023844"); // RCC_APB2ENR
        core::arch::asm!("ldr r1, =0x00004000"); // ENABLE SYSCFG CLOCK
        core::arch::asm!("str r1, [r0, #0]");
        core::arch::asm!("ldr r0, =0x40013800"); // SYSCFG_MEMRMP
        core::arch::asm!("ldr r1, =0x00000001"); // MAP ROM AT ZERO
        core::arch::asm!("str r1, [r0, #0]");
        core::arch::asm!("ldr r0, =0x1FFF0000"); // ROM BASE
        core::arch::asm!("ldr sp,[r0, #0]"); // SP @ +0
        core::arch::asm!("ldr r0,[r0, #4]"); // PC @ +4
        core::arch::asm!("bx r0");
    }
}

pub fn reboot() {
    free(|_cs| loop {
        cortex_m::peripheral::SCB::sys_reset();
    })
}

pub fn reboot_to_bootloader() {
    log!(Info, "Rebooting to bootloader...");

    // Write bootloader request to RTC backup register
    let rtc = unsafe { &(*RTC::ptr()) };
    rtc.bkpr[0].write(|w| w.bits(1));

    reboot();
}

pub fn init_bootloader() {
    // Enable power to the backup registers
    let rcc = unsafe { &(*RCC::ptr()) };
    let pwr = unsafe { &(*PWR::ptr()) };
    PWR::enable(rcc);
    pwr.cr.modify(|_, w| w.dbp().set_bit());

    // Check backup register. If > 0, clear the register and reboot
    // to bootloader
    let rtc = unsafe { &(*RTC::ptr()) };
    if rtc.bkpr[0].read().bits() > 0 {
        rtc.bkpr[0].write(|w| w.bits(0));
        defmt::info!("Jumping to bootloader...");

        jump_to_bootloader();
    }
}
