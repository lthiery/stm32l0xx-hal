#![no_std]
#![no_main]
extern crate nb;
extern crate panic_halt;

use core::fmt::Write;
use embedded_hal::digital::v2::OutputPin;
use stm32l0xx_hal as hal;
use hal::{
    exti, exti::TriggerEdge, gpio::*, pac, prelude::*, rcc::Config, serial, syscfg};
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use core::ops::DerefMut;

static mut USR_BTN: Mutex<
    RefCell<Option<hal::gpio::gpiob::PB5<hal::gpio::Input<hal::gpio::Floating>>>>,
> = Mutex::new(RefCell::new(None));

#[derive(Debug)]
pub enum AppEvent {
    ReadAdc,
    ButtonPressed,
}


#[rtfm::app(device = stm32l0xx_hal::pac)]
const APP: () = {
    static EXTI: pac::EXTI = ();
    static mut DEBUG_TX: serial::Tx<stm32l0::stm32l0x2::USART2> = ();
    static mut DEBUG_RX: serial::Rx<stm32l0::stm32l0x2::USART2> = ();

    static mut BATT_ANALOG: hal::gpio::gpioa::PA5<hal::gpio::Analog> = ();
    static mut ADC: stm32l0xx_hal::adc::Adc = ();

    #[init]
    fn init() -> init::LateResources {
        // Configure the clock.
        let mut rcc = device.RCC.freeze(Config::hsi16());
        let mut syscfg = syscfg::SYSCFG::new(device.SYSCFG_COMP, &mut rcc);

        let gpioa = device.GPIOA.split(&mut rcc);
        let gpiob = device.GPIOB.split(&mut rcc);

        // Configure the serial peripheral.
        let tx_pin = gpioa.pa2;
        let rx_pin = gpioa.pa3;
        let mut serial = device
            .USART2
            .usart(
                (tx_pin, rx_pin),
                serial::Config::default().baudrate(115_200.bps()),
                &mut rcc,
            )
            .unwrap();

        serial.listen(serial::Event::Rxne);

        let (mut tx, rx) = serial.split();
        writeln!(tx, "Hello, world!\r").unwrap();

        // Configure PB5 as input for rising interrupt
        let usr_btn = gpiob.pb5.into_floating_input();
        let exti = device.EXTI;
        exti.listen(&mut syscfg, usr_btn.port, usr_btn.i, TriggerEdge::Rising);

        let batt_analog = gpioa.pa5.into_analog();
        let adc = device.ADC.constrain(&mut rcc);

        // Store the external interrupt and LED in mutex reffcells to make them
        // available from the interrupt.
        unsafe {
            cortex_m::interrupt::free(|cs| {
                *USR_BTN.borrow(cs).borrow_mut() = Some(usr_btn);
            });
        };

        init::LateResources {
            EXTI: exti,
            DEBUG_TX: tx,
            DEBUG_RX: rx,
            BATT_ANALOG: batt_analog,
            ADC: adc,
        }
    }

    #[task(priority = 1, capacity = 8, resources = [DEBUG_TX, BATT_ANALOG, ADC])]
    fn app_event(event: AppEvent) {

        match event {
            AppEvent::ReadAdc => {
                let mut val: u16 = 0;
                unsafe {
                    cortex_m::interrupt::free(|cs| {
                        if let Some(batt_switch) = USR_BTN.borrow(cs).replace(None)
                        {
                            let mut pin = batt_switch.into_push_pull_output();

                            pin.set_high().unwrap();
                            val =
                                resources.ADC.read(resources.BATT_ANALOG).unwrap();
                            pin.set_low().unwrap();

                            *USR_BTN.borrow(cs).borrow_mut() =
                                Some(pin.into_floating_input());
                        };
                    });
                };
                writeln!(resources.DEBUG_TX, "ADC Reading = {}\r", val).unwrap();
            }
            AppEvent::ButtonPressed => {
                writeln!(resources.DEBUG_TX, "Button has been pressed - fun!\r").unwrap();

            }
        }

    }

    #[interrupt(priority = 2, resources = [EXTI], spawn = [app_event])]
    fn EXTI4_15() {
        let reg = resources.EXTI.get_pending_irq();

        unsafe {
            cortex_m::interrupt::free(|cs| {
                if let Some(usr_btn) = USR_BTN.borrow(cs).borrow_mut().deref_mut() {
                    if exti::line_is_triggered(reg, usr_btn.i) {
                        resources.EXTI.clear_irq(usr_btn.i);
                        spawn.app_event(AppEvent::ButtonPressed).unwrap();
                    }
                }
            });
        };
    }

    #[interrupt(priority = 2, resources = [DEBUG_RX], spawn = [app_event])]
    fn USART2() {
        if let Ok(byte) = resources.DEBUG_RX.read() {
            if byte == b'a' {
                spawn.app_event(AppEvent::ReadAdc).unwrap();
            }
        }
    }

    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn USART4_USART5();
    }
};