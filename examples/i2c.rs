//#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m_rt::entry;
use stm32l0xx_hal::{pac, prelude::*, rcc::Config};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Configure the clock.
    let mut rcc = dp.RCC.freeze(Config::hsi16());

    // Acquire the GPIOA peripheral. This also enables the clock for GPIOA in
    // the RCC register.]
    #[cfg(feature = "stm32l0x1")]
    let gpioa = dp.GPIOA.split(&mut rcc);

    #[cfg(feature = "stm32l0x1")]
    let sda = gpioa.pa10.into_open_drain_output();
    #[cfg(feature = "stm32l0x1")]
    let scl = gpioa.pa9.into_open_drain_output();

    // Acquire the GPIOB peripheral. This also enables the clock for GPIOB in
    // the RCC register.]
    #[cfg(feature = "stm32l0x2")]
    let gpiob = dp.GPIOB.split(&mut rcc);

    #[cfg(feature = "stm32l0x2")]
    let sda = gpiob.pb9.into_open_drain_output();
    #[cfg(feature = "stm32l0x2")]
    let scl = gpiob.pb8.into_open_drain_output();


    let mut i2c = dp
        .I2C1
        .i2c(sda, scl, 100.khz(), &mut rcc);

    let write_buf = [0xFAu8; 1];
    let mut buffer = [0u8; 1];
    const BMP280: u8 = 0x77;

    loop {
        i2c.write_read(BMP280, &write_buf, &mut buffer).unwrap();
    }
}
