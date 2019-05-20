// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;
use stm32l0xx_hal as hal;

use hal::{exti::TriggerEdge, gpio::*, pac, prelude::*, rcc::Config};

use embedded_hal::digital::v2::OutputPin;

#[rtfm::app(device = stm32l0xx_hal::pac)]
const APP: () = {
    static mut LED: gpiob::PB5<Output<PushPull>> = ();
    static mut INT: pac::EXTI = ();
    static mut BUTTON: gpiob::PB2<Input<PullUp>> = ();

    #[init]
    fn init() -> init::LateResources {

        // Configure the clock.
        let mut rcc = device.RCC.freeze(Config::hsi16());
        rcc.enable(hal::rcc::Peripheral::SYSCFG);
        // Acquire the GPIOB peripheral. This also enables the clock for GPIOB in
        // the RCC register.
        let gpiob = device.GPIOB.split(&mut rcc);

        // Configure PB5 as output.
        let led = gpiob.pb5.into_push_pull_output();

        // Configure PB2 as input.
        let button = gpiob.pb2.into_pull_up_input();

        // needs to be cleaned up as part of a better EXTI driver
        let syscfg_comp = &device.SYSCFG_COMP;
        syscfg_comp.exticr1.write(|w|
            unsafe {
                w
                .exti2().bits(0b1)
            }
        );

        // Configure the external interrupt on the falling edge for the pin 2.
        let exti = device.EXTI;
        exti.listen(hal::exti::Port::B, 2, TriggerEdge::Falling);

        // Return the initialised resources.
        init::LateResources {
            LED: led,
            INT: exti,
            BUTTON: button,
        }

    }

    #[interrupt(resources = [LED, INT])]
    fn EXTI2_3() {
        static mut STATE: bool = false;

        // Clear the interrupt flag.
        resources.INT.clear_irq(2);
        //unsafe {
            if *STATE {
               resources.LED.set_low().unwrap();
               *STATE = false;
            } else {
                resources.LED.set_high().unwrap();
               *STATE = true;
            }
        //}
        
    }

};
