// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;
use stm32l0xx_hal::{exti::TriggerEdge, gpio::*, pac, prelude::*, rcc::Config, pac::interrupt, pac::Interrupt};

#[rtfm::app(device = stm32l0xx_hal::pac)]
const APP: () = {
    static mut LED: gpiob::PB5<Output<PushPull>> = ();
    static mut INT: pac::EXTI = ();

    #[init]
    fn init() -> init::LateResources {

        // Configure the clock.
        let mut rcc = device.RCC.freeze(Config::hsi16());

        // Acquire the GPIOB peripheral. This also enables the clock for GPIOB in
        // the RCC register.
        let gpiob = device.GPIOB.split(&mut rcc);

        // Configure PB5 as output.
        let mut led = gpiob.pb5.into_push_pull_output();
        led.set_low();

        // Configure PB2 as input.
        let gpioa = device.GPIOA.split(&mut rcc);

        let button0 = gpioa.pa2.into_pull_up_input();
        let button = gpiob.pb2.into_pull_up_input();


        let exticr1 = &device.SYSCFG_COMP.exticr1;

        exticr1.modify(|_, w|
            unsafe {
                w
                .exti2().bits(0b1)
            }
        );

        // Configure the external interrupt on the falling edge for the pin 2.
        let exti = device.EXTI;
        exti.listen(2, TriggerEdge::All);

        //rtfm::enable(interrupt::EXTI2_3);

        // Return the initialised resources.
        init::LateResources {
            LED: led,
            INT: exti,
        }

    }

    // #[idle]
    // fn idle() -> ! {


    //     loop {}
    // }

    #[interrupt(resources = [LED, INT])]
    fn EXTI2_3() {
        static mut STATE: bool = false;

        // Clear the interrupt flag.
        //resources.INT.clear_irq(2);
        //unsafe {
            if *STATE {
               resources.LED.set_low();
               *STATE = false;
            } else {
                resources.LED.set_high();
               *STATE = true;
            }
        //}
        
    }

};
