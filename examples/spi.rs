#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_halt;

use cortex_m_rt::entry;
use stm32l0xx_hal::{pac, prelude::*, rcc::Config, spi};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Configure the clock.
    let mut rcc = dp.RCC.freeze(Config::hsi16());

    // Acquire the GPIOA peripheral. This also enables the clock for GPIOA in
    // the RCC register.
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);


/*
    let sck = gpioa.pa5;
    let miso = gpioa.pa11;
    let mosi = gpioa.pa12;
*/

    let sck = gpiob.pb3;
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7;
    let mut nss = gpioa.pa15.into_push_pull_output();
    nss.set_high();


    // Initialise the SPI peripheral.
    let mut spi = dp
        .SPI1
        .spi((sck, miso, mosi), spi::MODE_0, 100_000.hz(), &mut rcc);

    loop {
        nss.set_low();


        spi.write(&[0x1]).unwrap();
        let mut read_buf: [u8; 1] = [0; 1];
        spi.write(&mut read_buf).unwrap();

        

        nss.set_high();
    }
}
