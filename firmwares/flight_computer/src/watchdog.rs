use hal::pac::IWDG;
use stm32f4xx_hal as hal;

pub fn init_watchdog() {
    let iwdg = unsafe { &(*IWDG::ptr()) };
    iwdg.kr.write(|w| unsafe { w.bits(0xcccc) });
    iwdg.kr.write(|w| unsafe { w.bits(0x5555) });

    // Set clock prescaler and reset counter
    // IWDG timeout (seconds) is (1/32000) * 4 * 2^PR * (RL + 1),
    // For PR = 0 & RL = 0x7f, the timeout is 256ms
    iwdg.pr.write(|w| unsafe { w.bits(0) });
    iwdg.rlr.write(|w| unsafe { w.bits(0x7ff) });

    iwdg.kr.write(|w| unsafe { w.bits(0xaaaa) });
}

pub fn reset_watchdog() {
    let iwdg = unsafe { &(*IWDG::ptr()) };
    iwdg.kr.write(|w| unsafe { w.bits(0xaaaa) });
}
