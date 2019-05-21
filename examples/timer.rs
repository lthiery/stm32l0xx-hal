#![no_main]
#![no_std]

extern crate panic_halt;

use core::cell::RefCell;
use core::ops::DerefMut;

use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use stm32l0xx_hal::{
    gpio::*,
    pac::{self, Interrupt},
    prelude::*,
    rcc::Config,
    timer::Timer,
};

static LED: Mutex<RefCell<Option<gpioa::PA1<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<Timer<pac::TIM2>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Configure the clock.
    let mut rcc = dp.RCC.freeze(Config::hsi16());

    // Acquire the GPIOA peripheral. This also enables the clock for GPIOA in
    // the RCC register.
    let gpioa = dp.GPIOA.split(&mut rcc);

    // Configure PA1 as output.
    let led = gpioa.pa1.into_push_pull_output();

    // Configure the timer.
    let mut timer = dp.TIM2.timer(1.hz(), &mut rcc);
    timer.listen();

    // Store the LED and timer in mutex refcells to make them available from the
    // timer interrupt.
    cortex_m::interrupt::free(|cs| {
        *LED.borrow(cs).borrow_mut() = Some(led);
        *TIMER.borrow(cs).borrow_mut() = Some(timer);
    });

    // Enable the timer interrupt in the NVIC.
    let mut nvic = cp.NVIC;
    nvic.enable(Interrupt::TIM2);

    loop {
        asm::wfi();
    }
}

fn TIM2() {
    // Keep a state to blink the LED.
    static mut STATE: bool = false;

    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut timer) = TIMER.borrow(cs).borrow_mut().deref_mut() {
            // Clear the interrupt flag.
            timer.clear_irq();

            // Change the LED state on each interrupt.
            if let Some(ref mut led) = LED.borrow(cs).borrow_mut().deref_mut() {
                unsafe {
                    if STATE {
                        led.set_low();
                        STATE = false;
                    } else {
                        led.set_high();
                        STATE = true;
                    }
                }
            }
        }
    });
}
