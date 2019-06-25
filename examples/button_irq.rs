#![no_main]
#![no_std]

extern crate panic_halt;

use core::cell::RefCell;
use core::ops::DerefMut;

use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use stm32l0xx_hal::{
    exti::TriggerEdge,
    gpio::{self, *},
    pac::{interrupt, Interrupt, EXTI},
    prelude::*,
    rcc::Config,
};

static INT: Mutex<RefCell<Option<EXTI>>> = Mutex::new(RefCell::new(None));
static LED: Mutex<RefCell<Option<gpiob::PB5<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let mut dp = stm32l0xx_hal::stm32::Peripherals::take().unwrap();
    let     cp = cortex_m::Peripherals::take().unwrap();

    // Configure the clock.
    let mut rcc = dp.RCC.freeze(Config::hsi16());

    // Acquire the GPIOB peripheral. This also enables the clock for GPIOB in
    // the RCC register.
    let gpiob = dp.GPIOB.split(&mut rcc);

    // Configure PB6 as output.
    let led = gpiob.pb5.into_push_pull_output();

    // Configure PB2 as input.
    let _ = gpiob.pb2.into_pull_up_input();

    // Configure the external interrupt on the falling edge for the pin 0.
    let exti = dp.EXTI;
    exti.listen(
        &mut rcc,
        &mut dp.SYSCFG_COMP,
        gpio::Port::PB,
        2,
        TriggerEdge::Falling,
    );

    // Store the external interrupt and LED in mutex reffcells to make them
    // available from the interrupt.
    cortex_m::interrupt::free(|cs| {
        *INT.borrow(cs).borrow_mut() = Some(exti);
        *LED.borrow(cs).borrow_mut() = Some(led);
    });

    dp.SYSCFG_COMP.exticr1.write(|mut w|
        unsafe {
            // 1 = B
            w
            .exti2().bits(0b1)
            .exti2().bits(0b1)
            .exti1().bits(0b1)
            .exti0().bits(0b1)

        }
    );

    // Enable the external interrupt in the NVIC.
    let mut nvic = cp.NVIC;
    nvic.enable(Interrupt::EXTI2_3);

    loop {
        asm::wfi();
    }
}

#[interrupt]
fn EXTI2_3() {
    // Keep the LED state.
    static mut STATE: bool = false;

    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut exti) = INT.borrow(cs).borrow_mut().deref_mut() {
            // Clear the interrupt flag.
            exti.clear_irq(0);

            // Change the LED state on each interrupt.
            if let Some(ref mut led) = LED.borrow(cs).borrow_mut().deref_mut() {
                if *STATE {
                    led.set_low().unwrap();
                    *STATE = false;
                } else {
                    led.set_high().unwrap();
                    *STATE = true;
                }
                
            }
        }
    });
}
