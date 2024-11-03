#![no_std]
#![no_main]

use core::cell::RefCell;
use defmt::*;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Delay, Timer};
use embedded_ads1220::ADS1220;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello World!");

    spawner.spawn(ads1220()).unwrap();
}

#[embassy_executor::task]
async fn ads1220() {
    let p = embassy_stm32::init(Default::default());

    let mut spi_config= spi::Config::default();
    spi_config.frequency = Hertz(2_000_000);

    static SPI_BUS: StaticCell<NoopMutex<RefCell<Spi<Blocking>>>> = StaticCell::new();
    let spi = Spi::new_blocking(p.SPI3, p.PC10, p.PC12, p.PC11, spi_config);
    let spi_bus = SPI_BUS.init(NoopMutex::new(RefCell::new(spi)));
    let chip_select = Output::new(p.PC9, Level::Low, Speed::Low);
    let device = SpiDevice::new(spi_bus, chip_select);
    let data_ready = Input::new(p.PC8, Pull::Down);

    let mut adc = ADS1220::new(device, data_ready);

    adc.begin(&mut Delay).unwrap();
    loop {
        Timer::after_millis(300).await;
        adc.status()
    }
}
