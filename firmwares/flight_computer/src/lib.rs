#![no_std]
#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(unused_mut)]

use core::borrow::Borrow;
use core::net::Ipv4Addr;
use core::net::SocketAddr;
use core::net::SocketAddrV4;

use embassy_stm32::adc::AdcChannel;
use embassy_stm32::adc::AnyAdcChannel;
use embassy_stm32::usb::Driver;
use embassy_usb::UsbDevice;
use flash_task::FlashOp;
use rand::RngCore;

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_futures::join::join;
use embassy_futures::select::Either;
use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use embassy_time::{Delay, Duration, Instant, Ticker};

use embassy_stm32::adc::Adc;
use embassy_stm32::bind_interrupts;
use embassy_stm32::can::Can;
use embassy_stm32::can::CanConfigurator;
use embassy_stm32::eth::GenericPhy;
use embassy_stm32::eth::{Ethernet, PacketQueue};
use embassy_stm32::exti::Channel as _;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, OutputType, Pin, Pull, Speed};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::*;
use embassy_stm32::rcc::*;
use embassy_stm32::rng::Rng;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{Config, interrupt};

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embedded_io_async::Write;
use embedded_nal_async::TcpConnect;

use embedded_can::Id;
use embedded_can::StandardId;

use embassy_net::Ipv4Cidr;
use embassy_net::StackResources;
use embassy_net::tcp::client::TcpClient;
use embassy_net::tcp::client::TcpClientState;

use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::mod_params::Bandwidth;
use lora_phy::mod_params::CodingRate;
use lora_phy::mod_params::PacketStatus;
use lora_phy::mod_params::SpreadingFactor;
use lora_phy::sx126x::{self, Sx126x, Sx1262};
use lora_phy::{LoRa, RxMode};

use static_cell::StaticCell;

use shared_types::Settings;

//mod can;
pub mod drivers;
pub mod ethernet;
pub mod lora;
pub mod storage;
pub mod usb;
pub mod vehicle;

mod flash_task;
// mod storage_new;

use drivers::sensors::*;
use storage::{Flash, FlashHandle, FlashType};

bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => embassy_stm32::can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => embassy_stm32::can::IT1InterruptHandler<FDCAN1>;
    FDCAN2_IT0 => embassy_stm32::can::IT0InterruptHandler<FDCAN2>;
    FDCAN2_IT1 => embassy_stm32::can::IT1InterruptHandler<FDCAN2>;
    OTG_FS => embassy_stm32::usb::InterruptHandler<USB_OTG_FS>;
    ETH => embassy_stm32::eth::InterruptHandler;
    RNG => embassy_stm32::rng::InterruptHandler<embassy_stm32::peripherals::RNG>;
});

pub type OurSpiDevice<'a> = SpiDevice<'a, CriticalSectionRawMutex, Spi<'static, Async>, Output<'a>>;
pub type LoraVariant = GenericSx126xInterfaceVariant<Output<'static>, ExtiInput<'static>>;
pub type LoraTransceiver = Sx126x<OurSpiDevice<'static>, LoraVariant, Sx1262>;

pub struct BoardSensors {
    pub imu1: LSM6<OurSpiDevice<'static>>,
    pub imu2: ICM42688P<OurSpiDevice<'static>>,
    pub imu3: ICM42670P<OurSpiDevice<'static>>,
    pub highg: H3LIS331DL<OurSpiDevice<'static>>,
    pub mag: LIS3MDL<OurSpiDevice<'static>>,
    pub baro1: MS56<OurSpiDevice<'static>>,
    pub baro2: LPS22<OurSpiDevice<'static>>,
    pub baro3: BMP580<OurSpiDevice<'static>>,
}

pub struct BoardOutputs {
    pub leds: (Output<'static>, Output<'static>, Output<'static>),
    pub recovery_high: Output<'static>,
    pub recovery_lows: (Output<'static>, Output<'static>, Output<'static>, Output<'static>),
    pub continuity_check: Output<'static>,
}

pub struct BoardAdc {
    pub adc1: Adc<'static, ADC1>,
    pub adc2: Adc<'static, ADC2>,
    pub adc3: Adc<'static, ADC3>,
    pub dma: DMA2_CH7,
    pub main_voltage: AnyAdcChannel<ADC1>,
    pub supply_voltage: AnyAdcChannel<ADC1>,
    pub recovery_voltage: AnyAdcChannel<ADC1>,
    pub main_current: AnyAdcChannel<ADC1>,
    pub recovery_current: AnyAdcChannel<ADC1>,
    pub continuity_check: AnyAdcChannel<ADC1>,
}

pub struct Board {
    pub sensors: BoardSensors,
    pub outputs: BoardOutputs,
    pub adc: BoardAdc,
    pub can1: Can<'static>,
    pub can2: Can<'static>,
    pub lora1: LoRa<LoraTransceiver, Delay>,
    pub lora2: LoRa<LoraTransceiver, Delay>,
    pub flash: FlashType,
    pub flash_handle: FlashHandle,
    pub usb_driver: Driver<'static, USB_OTG_FS>,
    pub ethernet: Ethernet<'static, ETH, GenericPhy>,
    pub rng: Rng<'static, RNG>,
    pub iwdg: IndependentWatchdog<'static, IWDG1>,
}

static USB_EP_OUT_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();

static SPI1_SHARED: StaticCell<Mutex<CriticalSectionRawMutex, Spi<Async>>> = StaticCell::new();
static SPI2_SHARED: StaticCell<Mutex<CriticalSectionRawMutex, Spi<Async>>> = StaticCell::new();
static SPI3_SHARED: StaticCell<Mutex<CriticalSectionRawMutex, Spi<Async>>> = StaticCell::new();
static SPI4_SHARED: StaticCell<Mutex<CriticalSectionRawMutex, Spi<Async>>> = StaticCell::new();

pub async fn init_board() -> (Board, Settings, u64) {
    // Basic setup, including clocks
    // Divider values taken from STM32CubeMx
    let mut config = Config::default();
    config.rcc.hse = Some(embassy_stm32::rcc::Hse {
        mode: embassy_stm32::rcc::HseMode::Oscillator,
        freq: Hertz::mhz(25), // our high-speed external oscillator speed
    });
    config.rcc.sys = Sysclk::HSE;
    config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true }); // needed for USB
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        //prediv: PllPreDiv::DIV2,
        prediv: PllPreDiv::DIV5,
        //mul: PllMul::MUL64,
        mul: PllMul::MUL192,
        //divp: Some(PllDiv::DIV2),
        divp: Some(PllDiv::DIV4),
        //divq: Some(PllDiv::DIV4),
        divq: Some(PllDiv::DIV30),
        divr: None,
    });
    config.rcc.pll2 = Some(Pll {
        source: PllSource::HSE,
        //prediv: PllPreDiv::DIV2,
        prediv: PllPreDiv::DIV5,
        //mul: PllMul::MUL64,
        mul: PllMul::MUL192,
        //divp: Some(PllDiv::DIV2),
        divp: Some(PllDiv::DIV2),
        //divq: Some(PllDiv::DIV4),
        divq: Some(PllDiv::DIV30),
        divr: None,
    });
    config.rcc.pll3 = Some(Pll {
        source: PllSource::HSE,
        //prediv: PllPreDiv::DIV2,
        prediv: PllPreDiv::DIV5,
        //mul: PllMul::MUL64,
        mul: PllMul::MUL192,
        //divp: Some(PllDiv::DIV2),
        divp: Some(PllDiv::DIV2),
        //divq: Some(PllDiv::DIV4),
        divq: Some(PllDiv::DIV30),
        divr: None,
    });
    //config.rcc.per_clock_source = PerClockSource::HSE;
    config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
    config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
    config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
    config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
    config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
    config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
    config.rcc.voltage_scale = VoltageScale::Scale1;
    config.rcc.mux.fdcansel = embassy_stm32::rcc::mux::Fdcansel::HSE;
    // usbsel clock was chosen by trial and error
    config.rcc.mux.usbsel = embassy_stm32::rcc::mux::Usbsel::HSI48;

    let p = embassy_stm32::init(config);

    // let (usb, usb_flash) = UsbHandle::init(p.USB_OTG_FS, p.PA12, p.PA11).await;

    let mut spi1_config = embassy_stm32::spi::Config::default();
    spi1_config.frequency = Hertz::mhz(10);
    let spi1 = Spi::new(p.SPI1, p.PA5, p.PD7, p.PB4, p.DMA2_CH3, p.DMA2_CH2, spi1_config);
    let spi1 = Mutex::<CriticalSectionRawMutex, _>::new(spi1);
    let spi1 = SPI1_SHARED.init(spi1);

    let spi1_cs_imu1 = Output::new(p.PD4, Level::High, Speed::VeryHigh);
    let spi1_cs_highg = Output::new(p.PD9, Level::High, Speed::VeryHigh);
    let spi1_cs_mag = Output::new(p.PD5, Level::High, Speed::VeryHigh);
    let spi1_cs_baro1 = Output::new(p.PD2, Level::High, Speed::VeryHigh);

    let mut spi2_config = embassy_stm32::spi::Config::default();
    spi2_config.frequency = Hertz::mhz(10);
    let spi2 = Spi::new(p.SPI2, p.PD3, p.PB15, p.PB14, p.DMA1_CH4, p.DMA1_CH3, spi2_config);
    let spi2 = Mutex::<CriticalSectionRawMutex, _>::new(spi2);
    let spi2 = SPI2_SHARED.init(spi2);

    let spi2_cs_imu2 = Output::new(p.PD8, Level::High, Speed::VeryHigh);
    let spi2_cs_imu3 = Output::new(p.PC13, Level::High, Speed::VeryHigh);
    let spi2_cs_baro2 = Output::new(p.PE4, Level::High, Speed::VeryHigh);
    let spi2_cs_baro3 = Output::new(p.PE3, Level::High, Speed::VeryHigh);

    //let spi1_cs_radio = Output::new(p.PA1, Level::High, Speed::VeryHigh);

    let imu1 = LSM6::init(SpiDevice::new(spi1, spi1_cs_imu1)).await.unwrap();
    let imu2 = ICM42688P::init(SpiDevice::new(spi2, spi2_cs_imu2)).await.unwrap();
    let imu3 = ICM42670P::init(SpiDevice::new(spi2, spi2_cs_imu3)).await.unwrap();
    let highg = H3LIS331DL::init(SpiDevice::new(spi1, spi1_cs_highg)).await.unwrap();
    let mag = LIS3MDL::init(SpiDevice::new(spi1, spi1_cs_mag)).await.unwrap();
    let baro1 = MS56::init(MS56Variant::MS5607, SpiDevice::new(spi1, spi1_cs_baro1)).await.unwrap();
    let baro2 = LPS22::init(SpiDevice::new(spi2, spi2_cs_baro2)).await.unwrap();
    let baro3 = BMP580::init(SpiDevice::new(spi2, spi2_cs_baro3)).await.unwrap();

    let mut spi4_config = embassy_stm32::spi::Config::default();
    spi4_config.frequency = Hertz::mhz(10);
    let spi4 = Spi::new(p.SPI4, p.PE2, p.PE6, p.PE5, p.DMA1_CH1, p.DMA1_CH2, spi4_config);
    let spi4 = Mutex::<CriticalSectionRawMutex, _>::new(spi4);
    let spi4 = SPI4_SHARED.init(spi4);

    let lora1_cs = Output::new(p.PE7, Level::High, Speed::Low);
    let lora1_reset = Output::new(p.PE9, Level::High, Speed::Low);
    let lora1_irq = ExtiInput::new(p.PE8, p.EXTI8, Pull::Down);
    let lora1_busy = ExtiInput::new(p.PE10, p.EXTI10, Pull::Down);

    let lora2_cs = Output::new(p.PD13, Level::High, Speed::Low);
    let lora2_reset = Output::new(p.PD12, Level::High, Speed::Low);
    let lora2_irq = ExtiInput::new(p.PD14, p.EXTI14, Pull::Down);
    let lora2_busy = ExtiInput::new(p.PD15, p.EXTI15, Pull::Down);

    let lora1_config = sx126x::Config {
        chip: Sx1262,
        tcxo_ctrl: None,
        rx_boost: true,
        use_dcdc: false,
    };
    let lora1_spi = SpiDevice::new(spi4, lora1_cs);
    let lora1_iv = GenericSx126xInterfaceVariant::new(lora1_reset, lora1_irq, lora1_busy, None, None).unwrap();
    let mut lora1 = LoRa::new(Sx126x::new(lora1_spi, lora1_iv, lora1_config), false, Delay).await.unwrap();

    let lora2_config = sx126x::Config {
        chip: Sx1262,
        tcxo_ctrl: None,
        rx_boost: true,
        use_dcdc: false,
    };
    let lora2_spi = SpiDevice::new(spi4, lora2_cs);
    let lora2_iv = GenericSx126xInterfaceVariant::new(lora2_reset, lora2_irq, lora2_busy, None, None).unwrap();
    let mut lora2 = LoRa::new(Sx126x::new(lora2_spi, lora2_iv, lora2_config), false, Delay).await.unwrap();

    let mut can1 = CanConfigurator::new(p.FDCAN1, p.PB8, p.PB9, Irqs);
    let mut can2 = CanConfigurator::new(p.FDCAN2, p.PB5, p.PB6, Irqs);

    can1.set_bitrate(1_000_000);
    can2.set_bitrate(1_000_000);

    let mut can1 = can1.into_normal_mode();
    let mut can2 = can2.into_normal_mode();

    let mut rng = Rng::new(p.RNG, Irqs);
    let mut seed = [0; 8];
    rng.fill_bytes(&mut seed);
    let seed = u64::from_le_bytes(seed);

    let mac_addr = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

    static PACKETS: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
    let ethernet = Ethernet::new(
        PACKETS.init(PacketQueue::<4, 4>::new()),
        p.ETH,
        Irqs,
        p.PA1,  // ref_clk
        p.PA2,  // mdio
        p.PC1,  // eth_mdc
        p.PA7,  // CRS_DV: Carrier Sense
        p.PC4,  // RX_D0: Received Bit 0
        p.PC5,  // RX_D1: Received Bit 1
        p.PB12, // TX_D0: Transmit Bit 0
        p.PB13, // TX_D1: Transmit Bit 1
        p.PB11, // TX_EN: Transmit Enable
        GenericPhy::new(0),
        mac_addr,
    );

    let mut led_red = Output::new(p.PA8, Level::Low, Speed::Low);
    let mut led_yellow = Output::new(p.PA10, Level::Low, Speed::Low);
    let mut led_green = Output::new(p.PA15, Level::Low, Speed::Low);
    let leds = (led_red, led_yellow, led_green);

    // Set up the independent watchdog. This reboots the processor
    // if it is not pet regularly, even if the main clock fails.
    // TODO: check if the current boot is a watchdog reset and react
    // appropriately
    let iwdg = IndependentWatchdog::new(p.IWDG1, 512_000); // 512ms timeout

    let mut config = embassy_stm32::usb::Config::default();
    config.vbus_detection = true;
    let usb_driver = embassy_stm32::usb::Driver::new_fs(
        p.USB_OTG_FS,
        Irqs,
        p.PA12,
        p.PA11,
        USB_EP_OUT_BUFFER.init([0; 256]),
        config,
    );
    //
    // let mut config = embassy_usb::Config::new(0x0483, 0x5740);
    // config.manufacturer = Some("TUDSaT");
    // config.product = Some("Sting FC"); // TODO
    // config.serial_number = Some("12345678"); // TODO
    // config.device_class = 0xEF;
    // config.device_sub_class = 0x02;
    // config.device_protocol = 0x01;
    // config.composite_with_iads = true;
    //
    // let mut builder = embassy_usb::Builder::new(
    //     driver,
    //     config,
    //     USB_CONFIG_DESCRIPTOR_BUFFER.init([0; 256]),
    //     USB_BOS_DESCRIPTOR_BUFFER.init([0; 256]),
    //     USB_MSOS_DESCRIPTOR_BUFFER.init([0; 256]),
    //     USB_CONTROL_BUFFER.init([0; 128]),
    // );
    // let usb = builder.build();
    // let usb_class = CdcAcmClass::new(&mut builder, CDC_ACM_STATE.init(State::new()), 64);

    //// SPI 3
    let mut spi3_config = embassy_stm32::spi::Config::default();
    spi3_config.frequency = Hertz::mhz(20);
    let spi3 = Spi::new(p.SPI3, p.PC10, p.PC12, p.PC11, p.DMA1_CH7, p.DMA1_CH0, spi3_config);
    let spi3 = Mutex::<CriticalSectionRawMutex, _>::new(spi3);
    let spi3 = SPI3_SHARED.init(spi3);

    use storage::{Flash, FlashHandle};

    let spi3_cs_flash = Output::new(p.PD6, Level::High, Speed::VeryHigh);
    #[cfg(not(feature = "gcs"))]
    let (flash, flash_handle, settings) =
        Flash::init(SpiDevice::new(spi3, spi3_cs_flash)).await.map_err(|_e| ()).unwrap();

    //// Initialize GPS
    //#[cfg(not(feature = "gcs"))]
    //let (gps, gps_handle) = GPS::init(p.USART2, p.PA3, p.PA2, p.DMA1_CH6, p.DMA1_CH5);

    //#[cfg(not(feature = "gcs"))]
    //let adc = Adc::new(p.ADC1, &mut Delay);
    ////#[cfg(not(feature = "gcs"))]
    ////let power = PowerMonitor::init(adc, p.PB0, p.PC5, p.PC4).await;

    //let buzzer = {
    //    let gpiob_block = {
    //        use embassy_stm32::gpio::low_level::Pin;
    //        p.PB10.block()
    //    };
    //    let pwm_pin = PwmPin::new_ch3(p.PB10, OutputType::PushPull);
    //    let pwm = SimplePwm::new(p.TIM2, None, None, Some(pwm_pin), None, Hertz::hz(440), Default::default());
    //    Buzzer::init(pwm, Channel::Ch3, gpiob_block, 10)
    //};

    let recovery_high = Output::new(p.PE13, Level::Low, Speed::Low);
    let recovery_lows = (
        Output::new(p.PC9, Level::Low, Speed::Low),
        Output::new(p.PC8, Level::Low, Speed::Low),
        Output::new(p.PC7, Level::Low, Speed::Low),
        Output::new(p.PC6, Level::Low, Speed::Low),
    );
    let continuity_check = Output::new(p.PE14, Level::High, Speed::Low);

    let adc1 = Adc::new(p.ADC1);
    let adc2 = Adc::new(p.ADC2);
    let adc3 = Adc::new(p.ADC3);

    let mut adc_dma = p.DMA2_CH7;
    let adc_main_voltage = p.PB0.degrade_adc();
    let adc_supply_voltage = p.PB1.degrade_adc();
    let adc_recovery_voltage = p.PA3.degrade_adc();
    let adc_main_current = p.PA0.degrade_adc();
    let adc_recovery_current = p.PA6.degrade_adc();
    let adc_continuity_check = p.PC0.degrade_adc();

    let board = Board {
        sensors: BoardSensors {
            imu1,
            imu2,
            imu3,
            highg,
            mag,
            baro1,
            baro2,
            baro3,
        },
        outputs: BoardOutputs {
            leds,
            recovery_high,
            recovery_lows,
            continuity_check,
        },
        adc: BoardAdc {
            adc1,
            adc2,
            adc3,
            dma: *adc_dma,
            main_voltage: adc_main_voltage,
            supply_voltage: adc_supply_voltage,
            recovery_voltage: adc_recovery_voltage,
            main_current: adc_main_current,
            recovery_current: adc_recovery_current,
            continuity_check: adc_continuity_check,
        },
        lora1,
        lora2,
        can1,
        can2,
        flash,
        flash_handle,
        usb_driver,
        ethernet,
        rng,
        iwdg,
    };

    (board, settings, seed)
}
