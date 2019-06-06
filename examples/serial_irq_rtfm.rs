  
//#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_halt;
use stm32l0xx_hal as hal;

use core::fmt::Write;
use hal::{exti::TriggerEdge, gpio::*, pac, prelude::*, rcc::Config, serial};

#[rtfm::app(device = stm32l0xx_hal::pac)]
const APP: () = {
    static mut SERIAL_TX: stm32l0xx_hal::serial::Tx<stm32l0::stm32l0x2::USART2> = ();
    static mut SERIAL_RX: stm32l0xx_hal::serial::Rx<stm32l0::stm32l0x2::USART2> = ();


    #[init]
    fn init() -> init::LateResources {
         // Configure the clock.
        let mut rcc = device.RCC.freeze(Config::hsi16());

        // Acquire the GPIOA peripheral. This also enables the clock for GPIOA in
        // the RCC register.
        let gpioa = device.GPIOA.split(&mut rcc);

        #[cfg(feature = "stm32l0x1")]
        let tx_pin = gpioa.pa9;
        #[cfg(feature = "stm32l0x1")]
        let rx_pin = gpioa.pa10;

        #[cfg(feature = "stm32l0x2")]
        let tx_pin = gpioa.pa2;
        #[cfg(feature = "stm32l0x2")]
        let rx_pin = gpioa.pa3;

        // Configure the serial peripheral.
        let mut serial = device
            .USART2
            .usart((tx_pin, rx_pin), serial::Config::default(), &mut rcc)
            .unwrap();

        serial.listen(serial::Event::Rxne);

        let (mut tx, mut rx) = serial.split();

        // core::fmt::Write is implemented for tx.
        write!(tx, "Start typing: \r\n").unwrap();

        // Return the initialised resources.
        init::LateResources {
            SERIAL_TX: tx,
            SERIAL_RX: rx,
        }
    }

    #[task(capacity = 4, priority = 2, resources = [SERIAL_TX])]
    fn echo(byte: u8) {
        write!(resources.SERIAL_TX, "{}", byte as char).unwrap();
    }

    #[interrupt(priority = 1, resources = [SERIAL_RX], spawn = [echo])]
    fn USART2() {
        if let Ok(byte) = resources.SERIAL_RX.read() {
            spawn.echo(byte);
        }
    }

    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn USART4_USART5();
    }

};

/*
#[rtfm::app(device = stm32l0xx_hal::pac)]
const APP: () = {
    static mut LED: gpiob::PB6<Output<PushPull>> = ();
    static mut INT: pac::EXTI = ();
    static mut BUTTON: gpiob::PB2<Input<PullUp>> = ();

    #[init]
    fn init() -> init::LateResources {

        // Configure the clock.
        let mut rcc = device.RCC.freeze(Config::hsi16());

        // Acquire the GPIOB peripheral. This also enables the clock for GPIOB in
        // the RCC register.
        let gpiob = device.GPIOB.split(&mut rcc);

        // Configure PB5 as output.
        let led = gpiob.pb6.into_push_pull_output();

        // Configure PB2 as input.
        let button = gpiob.pb2.into_pull_up_input();
        #[cfg(feature = "stm32l0x1")]
        let mut syscfg = device.SYSCFG;
        #[cfg(feature = "stm32l0x2")]
        let mut syscfg = device.SYSCFG_COMP;

        // Configure the external interrupt on the falling edge for the pin 0.
        let exti = device.EXTI;
        exti.listen(
            &mut rcc,
            &mut syscfg,
            button.port,
            button.i,
            TriggerEdge::Falling,
        );

        // Return the initialised resources.
        init::LateResources {
            LED: led,
            INT: exti,
            BUTTON: button,
        }

    }

    #[interrupt(resources = [LED, INT, BUTTON])]
    fn EXTI2_3() {
        static mut STATE: bool = false;

        // Clear the interrupt flag.
        resources.INT.clear_irq(resources.BUTTON.i);
        if *STATE {
           resources.LED.set_low().unwrap();
           *STATE = false;
        } else {
            resources.LED.set_high().unwrap();
           *STATE = true;
        }
        
    }

};
*/