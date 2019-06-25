//#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_halt;

use core::fmt::Write;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use stm32l0xx_hal::{pac, prelude::*, rcc::Config, spi, serial};

use stm32l0xx_hal as hal;

use stm32l0xx_hal::gpio::gpioa::*;
use stm32l0xx_hal::gpio::{Input, Floating, Output, PushPull};
use stm32l0xx_hal::pac::SPI1;


#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Configure the clock.
    let mut rcc = dp.RCC.freeze(Config::hsi16());


    // Acquire the GPIOA peripheral. This also enables the clock for GPIOA in
    // the RCC register.
    let gpioa = dp.GPIOA.split(&mut rcc);

    let tx_pin = gpioa.pa9;
    let rx_pin = gpioa.pa10;

    // Configure the serial peripheral.
    let serial = dp
        .USART1
        .usart((tx_pin, rx_pin), serial::Config::default(), &mut rcc)
        .unwrap();

    let (mut tx, _rx) = serial.split();

    write!(tx, "Hello, world!\r\n").unwrap();


    let sck = gpioa.pa5;
    let miso = gpioa.pa11;
    let mosi = gpioa.pa12;
    let mut nss = gpioa.pa2.into_push_pull_output();
    
    // let sck = gpiob.pb3;
    // let miso = gpioa.pa6;
    // let mosi = gpioa.pa7;
    // let mut nss = gpioa.pa15.into_push_pull_output();

    nss.set_high().unwrap();

    // Initialise the SPI peripheral.
    let mut spi = dp
        .SPI1
        .spi((sck, miso, mosi), spi::MODE_0, 100_000.hz(), &mut rcc);
    
    loop {
        let value0 = read_register(&mut nss, &mut spi, 0x01);
        write!(tx, "{} reg x{:x}\r\n",0x01, value0).unwrap();
        let value1 = read_register(&mut nss, &mut spi, 0x06);
        write!(tx, "{} reg x{:x}\r\n",0x06, value1).unwrap();
        let value2 = read_register(&mut nss, &mut spi, 0x07);
        write!(tx, "{} reg x{:x}\r\n",0x07, value2).unwrap();

    }
}

fn read_register(
    nss: &mut stm32l0xx_hal::gpio::gpioa::PA2<Output<PushPull>>,
    spi: &mut hal::spi::Spi<SPI1, (PA5<Input<Floating>>, PA11<Input<Floating>>, PA12<Input<Floating>>)>, 

    //spi: &mut hal::spi::Spi<SPI1, (PB3<Input<Floating>>, PA6<Input<Floating>>, PA7<Input<Floating>>)>, 
    addr: u8) -> u8{


        nss.set_low().unwrap();

        spi.send(addr).unwrap();
        loop {        
        match spi.read() {
                Ok(_byte) => {
                    break;
                },
                _ => {}
            }
        }

        spi.send(0x00).unwrap();
        loop {        
        match spi.read() {
                Ok(byte) => {
                    nss.set_high().unwrap();
                    return byte;
                },
                _ => {}
            }
        }


}