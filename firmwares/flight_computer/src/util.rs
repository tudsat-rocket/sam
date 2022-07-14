use cortex_m::interrupt::free;

const BOOTLOADER_ADDRESS: usize = 0x1fff0000;
const BOOTLOADER_REQUEST_ADDRESS: usize = 0x40024000;

#[no_mangle]
pub fn jump_to_bootloader() {
    //const BOOTLOADER_ADDRESS: usize = 0x1fff77de;

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

        //core::arch::asm!("str sp, [{addr}, #0]", addr = in(reg) BOOTLOADER_ADDRESS);

        //core::arch::asm!("ldr r0, =0x1FFF0000"); // ROM BASE
        ////core::arch::asm!("ldr sp,[r0, #0]"); // SP @ +0
        //core::arch::asm!("ldr r0,[r0, #4]"); // PC @ +4
        //core::arch::asm!("bx {addr}", addr = in(reg) BOOTLOADER_ADDRESS + 4);

        //core::arch::asm!("bx r0", in("r0") BOOTLOADER_ADDRESS + 4);
        //let bootloader_func: extern "C" fn() = core::mem::transmute(BOOTLOADER_ADDRESS + 4);
        //bootloader_func();
    }
}

pub fn get_bootloader_request() -> bool {
    let raw = unsafe { *core::mem::transmute::<_, *const u8>(BOOTLOADER_REQUEST_ADDRESS) };
    raw == 1
}

pub fn set_bootloader_request(boot_to_bootloader: bool) {
    unsafe {
        *(core::mem::transmute::<_, *mut u8>(BOOTLOADER_REQUEST_ADDRESS)) =
            boot_to_bootloader as u8;
    }
}

pub fn reboot() {
    loop {
        cortex_m::peripheral::SCB::sys_reset();
    }
}

pub fn reboot_to_bootloader() {
    free(|_cs| {
        set_bootloader_request(true);
        reboot();
    });
}
