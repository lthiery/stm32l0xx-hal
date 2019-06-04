#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_halt;

use cortex_m_rt::entry;
use stm32l0xx_hal::{pac, prelude::*, rcc::Config};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Configure the clock.
    let mut rcc = dp.RCC.freeze(Config::hsi16());

    // Acquire the GPI0A and GPIOB peripherals. This also enables the clock for
    // GPIOA and GPIOB in the RCC register.
    let gpiob = dp.GPIOB.split(&mut rcc);

    // Configure PB2 as input.
    let button = gpiob.pb2.into_pull_up_input();

    // Configure PB5 as output.
    let mut led = gpiob.pb5.into_push_pull_output();

    loop {
        let wait =  match button.is_high() {
            Ok(true) => 300.ms(),
            Ok(false) => 100.ms(),
            _ => unreachable!(),
        };
        delay.delay(wait);
        led.toggle().unwrap();
    }
}
