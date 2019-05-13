#![feature(prelude_import)]
#![no_std]
#![no_std]
#[prelude_import]
use ::core::prelude::v1::*;
#[macro_use]
extern crate core as core;
#[macro_use]
extern crate compiler_builtins as compiler_builtins;
pub use crate::pac as device;
pub use crate::pac as stm32;
use embedded_hal as hal;
#[cfg(feature = "stm32l0x2")]
pub use stm32l0::stm32l0x2 as pac;
mod bb {
    #![doc = " Bit banding"]
    use core::ptr;
    pub fn clear<T>(register: *const T, bit: u8) {
        write(register, bit, false);
    }
    pub fn set<T>(register: *const T, bit: u8) {
        write(register, bit, true);
    }
    pub fn write<T>(register: *const T, bit: u8, set: bool) {
        let addr = register as usize;
        if !(addr >= 1073741824 && addr <= 1074790400) {
            {
                ::core::panicking::panic(&(
                    "assertion failed: addr >= 1073741824 && addr <= 1074790400",
                    "src/bb.rs",
                    15u32,
                    5u32,
                ))
            }
        };
        if !(bit < 32) {
            {
                ::core::panicking::panic(&("assertion failed: bit < 32", "src/bb.rs", 16u32, 5u32))
            }
        };
        let bit = bit as usize;
        let bb_addr = (1107296256 + (addr - 1073741824) * 32) + 4 * bit;
        unsafe { ptr::write_volatile(bb_addr as *mut u32, if set { 1 } else { 0 }) }
    }
}
pub mod delay {
    #![doc = " Delays"]
    use crate::hal::blocking::delay::{DelayMs, DelayUs};
    use crate::rcc::Clocks;
    use crate::time::MicroSeconds;
    use cast::u32;
    use cortex_m::peripheral::syst::SystClkSource;
    use cortex_m::peripheral::SYST;
    pub trait DelayExt {
        fn delay(self, clocks: Clocks) -> Delay;
    }
    impl DelayExt for SYST {
        fn delay(self, clocks: Clocks) -> Delay {
            Delay::new(self, clocks)
        }
    }
    #[doc = " System timer (SysTick) as a delay provider"]
    pub struct Delay {
        ticks_per_us: u32,
        syst: SYST,
    }
    impl Delay {
        #[doc = " Configures the system timer (SysTick) as a delay provider"]
        pub fn new(mut syst: SYST, clocks: Clocks) -> Self {
            syst.set_clock_source(SystClkSource::Core);
            let freq = clocks.sys_clk().0;
            if !(freq > 1000000u32) {
                {
                    ::core::panicking::panic(&(
                        "assertion failed: freq > 1000000u32",
                        "src/delay.rs",
                        30u32,
                        9u32,
                    ))
                }
            };
            let ticks_per_us = freq / 1000000u32;
            Delay { syst, ticks_per_us }
        }
        pub fn delay<T>(&mut self, delay: T)
        where
            T: Into<MicroSeconds>,
        {
            self.delay_us(delay.into().0)
        }
        #[doc = " Releases the system timer (SysTick) resource"]
        pub fn free(self) -> SYST {
            self.syst
        }
    }
    impl DelayMs<u32> for Delay {
        fn delay_ms(&mut self, ms: u32) {
            self.delay_us(ms * 1000);
        }
    }
    impl DelayMs<u16> for Delay {
        fn delay_ms(&mut self, ms: u16) {
            self.delay_ms(u32(ms));
        }
    }
    impl DelayMs<u8> for Delay {
        fn delay_ms(&mut self, ms: u8) {
            self.delay_ms(u32(ms));
        }
    }
    impl DelayUs<u32> for Delay {
        fn delay_us(&mut self, us: u32) {
            const MAX_RVR: u32 = 16777215;
            let mut total_rvr = self.ticks_per_us * us;
            while total_rvr > 0 {
                let current_rvr = if total_rvr <= MAX_RVR {
                    total_rvr
                } else {
                    MAX_RVR
                };
                self.syst.set_reload(current_rvr);
                self.syst.clear_current();
                self.syst.enable_counter();
                total_rvr -= current_rvr;
                while !self.syst.has_wrapped() {}
                self.syst.disable_counter();
            }
        }
    }
    impl DelayUs<u16> for Delay {
        fn delay_us(&mut self, us: u16) {
            self.delay_us(u32(us))
        }
    }
    impl DelayUs<u8> for Delay {
        fn delay_us(&mut self, us: u8) {
            self.delay_us(u32(us))
        }
    }
}
pub mod exti {
    #![doc = " External interrupt controller"]
    use crate::bb;
    use crate::pac::EXTI;
    pub enum TriggerEdge {
        Rising,
        Falling,
        All,
    }
    pub trait ExtiExt {
        fn listen(&self, line: u8, edge: TriggerEdge);
        fn unlisten(&self, line: u8);
        fn pend_interrupt(&self, line: u8);
        fn clear_irq(&self, line: u8);
    }
    impl ExtiExt for EXTI {
        fn listen(&self, line: u8, edge: TriggerEdge) {
            if !(line < 24) {
                {
                    ::core::panicking::panic(&(
                        "assertion failed: line < 24",
                        "src/exti.rs",
                        20u32,
                        9u32,
                    ))
                }
            };
            match edge {
                TriggerEdge::Rising => bb::set(&self.rtsr, line),
                TriggerEdge::Falling => bb::set(&self.ftsr, line),
                TriggerEdge::All => {
                    bb::set(&self.rtsr, line);
                    bb::set(&self.ftsr, line);
                }
            }
            bb::set(&self.imr, line);
        }
        fn unlisten(&self, line: u8) {
            if !(line < 24) {
                {
                    ::core::panicking::panic(&(
                        "assertion failed: line < 24",
                        "src/exti.rs",
                        33u32,
                        9u32,
                    ))
                }
            };
            bb::clear(&self.rtsr, line);
            bb::clear(&self.ftsr, line);
            bb::clear(&self.imr, line);
        }
        fn pend_interrupt(&self, line: u8) {
            if !(line < 24) {
                {
                    ::core::panicking::panic(&(
                        "assertion failed: line < 24",
                        "src/exti.rs",
                        40u32,
                        9u32,
                    ))
                }
            };
            bb::set(&self.swier, line);
        }
        fn clear_irq(&self, line: u8) {
            if !(line < 24) {
                {
                    ::core::panicking::panic(&(
                        "assertion failed: line < 24",
                        "src/exti.rs",
                        45u32,
                        9u32,
                    ))
                }
            };
            bb::set(&self.pr, line);
        }
    }
}
pub mod gpio {
    #![doc = " General Purpose Input / Output"]
    use crate::rcc::Rcc;
    use core::marker::PhantomData;
    #[doc = " Extension trait to split a GPIO peripheral in independent pins and registers"]
    pub trait GpioExt {
        #[doc = " The parts to split the GPIO into"]
        type Parts;
        #[doc = " Splits the GPIO block into independent pins and registers"]
        fn split(self, rcc: &mut Rcc) -> Self::Parts;
    }
    #[doc = " Input mode (type state)"]
    pub struct Input<MODE> {
        _mode: PhantomData<MODE>,
    }
    #[doc = " Floating input (type state)"]
    pub struct Floating;
    #[doc = " Pulled down input (type state)"]
    pub struct PullDown;
    #[doc = " Pulled up input (type state)"]
    pub struct PullUp;
    #[doc = " Open drain input or output (type state)"]
    pub struct OpenDrain;
    #[doc = " Analog mode (type state)"]
    pub struct Analog;
    #[doc = " Output mode (type state)"]
    pub struct Output<MODE> {
        _mode: PhantomData<MODE>,
    }
    #[doc = " Push pull output (type state)"]
    pub struct PushPull;
    #[doc = " GPIO Pin speed selection"]
    pub enum Speed {
        Low = 0,
        Medium = 1,
        High = 2,
        VeryHigh = 3,
    }
    #[allow(dead_code)]
    pub(crate) enum AltMode {
        AF0 = 0,
        AF1 = 1,
        AF2 = 2,
        AF3 = 3,
        AF4 = 4,
        AF5 = 5,
        AF6 = 6,
        AF7 = 7,
        SYSTEM = 8,
        TIM2 = 9,
        TIM3_5 = 10,
        TIM9_11 = 11,
        I2C = 12,
        SPI1_2 = 13,
        SPI3 = 14,
        USART1_3 = 15,
        UART4_5 = 16,
        USB = 17,
        LCD = 18,
        FSMC = 19,
        RI = 20,
        EVENTOUT = 21,
    }
    #[doc = " GPIO"]
    pub mod gpioa {
        use super::{
            AltMode, Analog, Floating, GpioExt, Input, OpenDrain, Output, PullDown, PullUp,
            PushPull, Speed,
        };
        use crate::hal::digital::{toggleable, InputPin, OutputPin, StatefulOutputPin};
        use crate::pac::GPIOA;
        use crate::rcc::Rcc;
        use core::marker::PhantomData;
        #[doc = " GPIO parts"]
        pub struct Parts {
            #[doc = " Pin"]
            pub pa0: PA0<Input<Floating>>,
            #[doc = " Pin"]
            pub pa1: PA1<Input<Floating>>,
            #[doc = " Pin"]
            pub pa2: PA2<Input<Floating>>,
            #[doc = " Pin"]
            pub pa3: PA3<Input<Floating>>,
            #[doc = " Pin"]
            pub pa4: PA4<Input<Floating>>,
            #[doc = " Pin"]
            pub pa5: PA5<Input<Floating>>,
            #[doc = " Pin"]
            pub pa6: PA6<Input<Floating>>,
            #[doc = " Pin"]
            pub pa7: PA7<Input<Floating>>,
            #[doc = " Pin"]
            pub pa8: PA8<Input<Floating>>,
            #[doc = " Pin"]
            pub pa9: PA9<Input<Floating>>,
            #[doc = " Pin"]
            pub pa10: PA10<Input<Floating>>,
            #[doc = " Pin"]
            pub pa11: PA11<Input<Floating>>,
            #[doc = " Pin"]
            pub pa12: PA12<Input<Floating>>,
            #[doc = " Pin"]
            pub pa13: PA13<Input<Floating>>,
            #[doc = " Pin"]
            pub pa14: PA14<Input<Floating>>,
            #[doc = " Pin"]
            pub pa15: PA15<Input<Floating>>,
        }
        impl GpioExt for GPIOA {
            type Parts = Parts;
            fn split(self, rcc: &mut Rcc) -> Parts {
                rcc.rb.iopenr.modify(|_, w| w.iopaen().set_bit());
                Parts {
                    pa0: PA0 { _mode: PhantomData },
                    pa1: PA1 { _mode: PhantomData },
                    pa2: PA2 { _mode: PhantomData },
                    pa3: PA3 { _mode: PhantomData },
                    pa4: PA4 { _mode: PhantomData },
                    pa5: PA5 { _mode: PhantomData },
                    pa6: PA6 { _mode: PhantomData },
                    pa7: PA7 { _mode: PhantomData },
                    pa8: PA8 { _mode: PhantomData },
                    pa9: PA9 { _mode: PhantomData },
                    pa10: PA10 { _mode: PhantomData },
                    pa11: PA11 { _mode: PhantomData },
                    pa12: PA12 { _mode: PhantomData },
                    pa13: PA13 { _mode: PhantomData },
                    pa14: PA14 { _mode: PhantomData },
                    pa15: PA15 { _mode: PhantomData },
                }
            }
        }
        #[doc = " Partially erased pin"]
        pub struct PA<MODE> {
            i: u8,
            _mode: PhantomData<MODE>,
        }
        impl<MODE> OutputPin for PA<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << self.i)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (self.i + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << self.i) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA<Output<MODE>> {}
        impl<MODE> InputPin for PA<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << self.i) == 0 }
            }
        }
        impl<MODE> InputPin for PA<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << self.i) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PA0<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA0<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PA0<Input<Floating>> {
                let offset = 2 * 0;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA0 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PA0<Input<PullDown>> {
                let offset = 2 * 0;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA0 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PA0<Input<PullUp>> {
                let offset = 2 * 0;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA0 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PA0<Analog> {
                let offset = 2 * 0;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PA0 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PA0<Output<OpenDrain>> {
                let offset = 2 * 0;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 0)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA0 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PA0<Output<PushPull>> {
                let offset = 2 * 0;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 0)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA0 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 0;
                unsafe {
                    &(*GPIOA::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 0;
                let offset2 = 4 * 0;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOA::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOA::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PA0<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Output<MODE>> {
                PA {
                    i: 0,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PA0<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 0)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (0 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA0<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << 0) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA0<Output<MODE>> {}
        impl<MODE> InputPin for PA0<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 0) == 0 }
            }
        }
        impl<MODE> PA0<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Input<MODE>> {
                PA {
                    i: 0,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA0<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 0) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PA1<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA1<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PA1<Input<Floating>> {
                let offset = 2 * 1;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA1 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PA1<Input<PullDown>> {
                let offset = 2 * 1;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA1 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PA1<Input<PullUp>> {
                let offset = 2 * 1;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA1 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PA1<Analog> {
                let offset = 2 * 1;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PA1 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PA1<Output<OpenDrain>> {
                let offset = 2 * 1;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 1)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA1 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PA1<Output<PushPull>> {
                let offset = 2 * 1;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 1)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA1 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 1;
                unsafe {
                    &(*GPIOA::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 1;
                let offset2 = 4 * 1;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOA::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOA::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PA1<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Output<MODE>> {
                PA {
                    i: 1,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PA1<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 1)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (1 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA1<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << 1) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA1<Output<MODE>> {}
        impl<MODE> InputPin for PA1<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 1) == 0 }
            }
        }
        impl<MODE> PA1<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Input<MODE>> {
                PA {
                    i: 1,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA1<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 1) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PA2<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA2<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PA2<Input<Floating>> {
                let offset = 2 * 2;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA2 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PA2<Input<PullDown>> {
                let offset = 2 * 2;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA2 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PA2<Input<PullUp>> {
                let offset = 2 * 2;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA2 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PA2<Analog> {
                let offset = 2 * 2;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PA2 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PA2<Output<OpenDrain>> {
                let offset = 2 * 2;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 2)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA2 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PA2<Output<PushPull>> {
                let offset = 2 * 2;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 2)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA2 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 2;
                unsafe {
                    &(*GPIOA::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 2;
                let offset2 = 4 * 2;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOA::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOA::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PA2<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Output<MODE>> {
                PA {
                    i: 2,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PA2<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 2)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (2 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA2<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << 2) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA2<Output<MODE>> {}
        impl<MODE> InputPin for PA2<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 2) == 0 }
            }
        }
        impl<MODE> PA2<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Input<MODE>> {
                PA {
                    i: 2,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA2<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 2) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PA3<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA3<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PA3<Input<Floating>> {
                let offset = 2 * 3;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA3 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PA3<Input<PullDown>> {
                let offset = 2 * 3;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA3 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PA3<Input<PullUp>> {
                let offset = 2 * 3;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA3 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PA3<Analog> {
                let offset = 2 * 3;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PA3 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PA3<Output<OpenDrain>> {
                let offset = 2 * 3;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 3)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA3 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PA3<Output<PushPull>> {
                let offset = 2 * 3;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 3)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA3 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 3;
                unsafe {
                    &(*GPIOA::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 3;
                let offset2 = 4 * 3;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOA::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOA::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PA3<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Output<MODE>> {
                PA {
                    i: 3,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PA3<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 3)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (3 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA3<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << 3) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA3<Output<MODE>> {}
        impl<MODE> InputPin for PA3<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 3) == 0 }
            }
        }
        impl<MODE> PA3<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Input<MODE>> {
                PA {
                    i: 3,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA3<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 3) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PA4<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA4<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PA4<Input<Floating>> {
                let offset = 2 * 4;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA4 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PA4<Input<PullDown>> {
                let offset = 2 * 4;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA4 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PA4<Input<PullUp>> {
                let offset = 2 * 4;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA4 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PA4<Analog> {
                let offset = 2 * 4;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PA4 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PA4<Output<OpenDrain>> {
                let offset = 2 * 4;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 4)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA4 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PA4<Output<PushPull>> {
                let offset = 2 * 4;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 4)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA4 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 4;
                unsafe {
                    &(*GPIOA::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 4;
                let offset2 = 4 * 4;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOA::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOA::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PA4<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Output<MODE>> {
                PA {
                    i: 4,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PA4<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 4)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (4 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA4<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << 4) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA4<Output<MODE>> {}
        impl<MODE> InputPin for PA4<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 4) == 0 }
            }
        }
        impl<MODE> PA4<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Input<MODE>> {
                PA {
                    i: 4,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA4<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 4) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PA5<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA5<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PA5<Input<Floating>> {
                let offset = 2 * 5;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA5 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PA5<Input<PullDown>> {
                let offset = 2 * 5;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA5 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PA5<Input<PullUp>> {
                let offset = 2 * 5;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA5 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PA5<Analog> {
                let offset = 2 * 5;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PA5 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PA5<Output<OpenDrain>> {
                let offset = 2 * 5;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 5)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA5 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PA5<Output<PushPull>> {
                let offset = 2 * 5;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 5)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA5 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 5;
                unsafe {
                    &(*GPIOA::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 5;
                let offset2 = 4 * 5;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOA::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOA::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PA5<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Output<MODE>> {
                PA {
                    i: 5,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PA5<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 5)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (5 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA5<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << 5) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA5<Output<MODE>> {}
        impl<MODE> InputPin for PA5<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 5) == 0 }
            }
        }
        impl<MODE> PA5<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Input<MODE>> {
                PA {
                    i: 5,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA5<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 5) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PA6<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA6<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PA6<Input<Floating>> {
                let offset = 2 * 6;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA6 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PA6<Input<PullDown>> {
                let offset = 2 * 6;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA6 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PA6<Input<PullUp>> {
                let offset = 2 * 6;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA6 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PA6<Analog> {
                let offset = 2 * 6;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PA6 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PA6<Output<OpenDrain>> {
                let offset = 2 * 6;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 6)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA6 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PA6<Output<PushPull>> {
                let offset = 2 * 6;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 6)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA6 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 6;
                unsafe {
                    &(*GPIOA::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 6;
                let offset2 = 4 * 6;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOA::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOA::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PA6<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Output<MODE>> {
                PA {
                    i: 6,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PA6<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 6)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (6 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA6<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << 6) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA6<Output<MODE>> {}
        impl<MODE> InputPin for PA6<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 6) == 0 }
            }
        }
        impl<MODE> PA6<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Input<MODE>> {
                PA {
                    i: 6,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA6<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 6) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PA7<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA7<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PA7<Input<Floating>> {
                let offset = 2 * 7;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA7 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PA7<Input<PullDown>> {
                let offset = 2 * 7;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA7 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PA7<Input<PullUp>> {
                let offset = 2 * 7;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA7 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PA7<Analog> {
                let offset = 2 * 7;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PA7 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PA7<Output<OpenDrain>> {
                let offset = 2 * 7;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 7)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA7 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PA7<Output<PushPull>> {
                let offset = 2 * 7;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 7)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA7 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 7;
                unsafe {
                    &(*GPIOA::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 7;
                let offset2 = 4 * 7;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOA::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOA::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PA7<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Output<MODE>> {
                PA {
                    i: 7,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PA7<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 7)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (7 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA7<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << 7) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA7<Output<MODE>> {}
        impl<MODE> InputPin for PA7<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 7) == 0 }
            }
        }
        impl<MODE> PA7<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Input<MODE>> {
                PA {
                    i: 7,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA7<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 7) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PA8<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA8<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PA8<Input<Floating>> {
                let offset = 2 * 8;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA8 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PA8<Input<PullDown>> {
                let offset = 2 * 8;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA8 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PA8<Input<PullUp>> {
                let offset = 2 * 8;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA8 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PA8<Analog> {
                let offset = 2 * 8;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PA8 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PA8<Output<OpenDrain>> {
                let offset = 2 * 8;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 8)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA8 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PA8<Output<PushPull>> {
                let offset = 2 * 8;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 8)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA8 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 8;
                unsafe {
                    &(*GPIOA::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 8;
                let offset2 = 4 * 8;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOA::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOA::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PA8<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Output<MODE>> {
                PA {
                    i: 8,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PA8<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 8)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (8 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA8<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << 8) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA8<Output<MODE>> {}
        impl<MODE> InputPin for PA8<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 8) == 0 }
            }
        }
        impl<MODE> PA8<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Input<MODE>> {
                PA {
                    i: 8,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA8<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 8) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PA9<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA9<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PA9<Input<Floating>> {
                let offset = 2 * 9;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA9 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PA9<Input<PullDown>> {
                let offset = 2 * 9;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA9 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PA9<Input<PullUp>> {
                let offset = 2 * 9;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA9 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PA9<Analog> {
                let offset = 2 * 9;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PA9 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PA9<Output<OpenDrain>> {
                let offset = 2 * 9;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 9)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA9 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PA9<Output<PushPull>> {
                let offset = 2 * 9;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 9)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA9 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 9;
                unsafe {
                    &(*GPIOA::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 9;
                let offset2 = 4 * 9;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOA::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOA::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PA9<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Output<MODE>> {
                PA {
                    i: 9,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PA9<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 9)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (9 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA9<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << 9) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA9<Output<MODE>> {}
        impl<MODE> InputPin for PA9<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 9) == 0 }
            }
        }
        impl<MODE> PA9<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Input<MODE>> {
                PA {
                    i: 9,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA9<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 9) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PA10<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA10<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PA10<Input<Floating>> {
                let offset = 2 * 10;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA10 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PA10<Input<PullDown>> {
                let offset = 2 * 10;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA10 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PA10<Input<PullUp>> {
                let offset = 2 * 10;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA10 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PA10<Analog> {
                let offset = 2 * 10;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PA10 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PA10<Output<OpenDrain>> {
                let offset = 2 * 10;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 10)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA10 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PA10<Output<PushPull>> {
                let offset = 2 * 10;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 10)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA10 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 10;
                unsafe {
                    &(*GPIOA::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 10;
                let offset2 = 4 * 10;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOA::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOA::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PA10<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Output<MODE>> {
                PA {
                    i: 10,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PA10<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 10)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (10 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA10<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << 10) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA10<Output<MODE>> {}
        impl<MODE> InputPin for PA10<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 10) == 0 }
            }
        }
        impl<MODE> PA10<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Input<MODE>> {
                PA {
                    i: 10,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA10<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 10) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PA11<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA11<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PA11<Input<Floating>> {
                let offset = 2 * 11;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA11 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PA11<Input<PullDown>> {
                let offset = 2 * 11;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA11 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PA11<Input<PullUp>> {
                let offset = 2 * 11;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA11 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PA11<Analog> {
                let offset = 2 * 11;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PA11 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PA11<Output<OpenDrain>> {
                let offset = 2 * 11;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 11)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA11 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PA11<Output<PushPull>> {
                let offset = 2 * 11;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 11)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA11 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 11;
                unsafe {
                    &(*GPIOA::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 11;
                let offset2 = 4 * 11;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOA::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOA::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PA11<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Output<MODE>> {
                PA {
                    i: 11,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PA11<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 11)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (11 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA11<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << 11) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA11<Output<MODE>> {}
        impl<MODE> InputPin for PA11<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 11) == 0 }
            }
        }
        impl<MODE> PA11<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Input<MODE>> {
                PA {
                    i: 11,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA11<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 11) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PA12<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA12<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PA12<Input<Floating>> {
                let offset = 2 * 12;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA12 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PA12<Input<PullDown>> {
                let offset = 2 * 12;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA12 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PA12<Input<PullUp>> {
                let offset = 2 * 12;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA12 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PA12<Analog> {
                let offset = 2 * 12;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PA12 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PA12<Output<OpenDrain>> {
                let offset = 2 * 12;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 12)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA12 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PA12<Output<PushPull>> {
                let offset = 2 * 12;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 12)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA12 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 12;
                unsafe {
                    &(*GPIOA::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 12;
                let offset2 = 4 * 12;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOA::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOA::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PA12<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Output<MODE>> {
                PA {
                    i: 12,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PA12<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 12)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (12 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA12<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << 12) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA12<Output<MODE>> {}
        impl<MODE> InputPin for PA12<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 12) == 0 }
            }
        }
        impl<MODE> PA12<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Input<MODE>> {
                PA {
                    i: 12,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA12<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 12) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PA13<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA13<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PA13<Input<Floating>> {
                let offset = 2 * 13;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA13 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PA13<Input<PullDown>> {
                let offset = 2 * 13;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA13 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PA13<Input<PullUp>> {
                let offset = 2 * 13;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA13 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PA13<Analog> {
                let offset = 2 * 13;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PA13 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PA13<Output<OpenDrain>> {
                let offset = 2 * 13;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 13)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA13 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PA13<Output<PushPull>> {
                let offset = 2 * 13;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 13)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA13 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 13;
                unsafe {
                    &(*GPIOA::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 13;
                let offset2 = 4 * 13;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOA::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOA::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PA13<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Output<MODE>> {
                PA {
                    i: 13,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PA13<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 13)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (13 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA13<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << 13) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA13<Output<MODE>> {}
        impl<MODE> InputPin for PA13<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 13) == 0 }
            }
        }
        impl<MODE> PA13<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Input<MODE>> {
                PA {
                    i: 13,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA13<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 13) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PA14<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA14<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PA14<Input<Floating>> {
                let offset = 2 * 14;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA14 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PA14<Input<PullDown>> {
                let offset = 2 * 14;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA14 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PA14<Input<PullUp>> {
                let offset = 2 * 14;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA14 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PA14<Analog> {
                let offset = 2 * 14;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PA14 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PA14<Output<OpenDrain>> {
                let offset = 2 * 14;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 14)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA14 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PA14<Output<PushPull>> {
                let offset = 2 * 14;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 14)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA14 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 14;
                unsafe {
                    &(*GPIOA::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 14;
                let offset2 = 4 * 14;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOA::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOA::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PA14<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Output<MODE>> {
                PA {
                    i: 14,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PA14<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 14)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (14 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA14<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << 14) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA14<Output<MODE>> {}
        impl<MODE> InputPin for PA14<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 14) == 0 }
            }
        }
        impl<MODE> PA14<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Input<MODE>> {
                PA {
                    i: 14,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA14<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 14) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PA15<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA15<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PA15<Input<Floating>> {
                let offset = 2 * 15;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA15 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PA15<Input<PullDown>> {
                let offset = 2 * 15;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA15 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PA15<Input<PullUp>> {
                let offset = 2 * 15;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PA15 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PA15<Analog> {
                let offset = 2 * 15;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PA15 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PA15<Output<OpenDrain>> {
                let offset = 2 * 15;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 15)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA15 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PA15<Output<PushPull>> {
                let offset = 2 * 15;
                unsafe {
                    &(*GPIOA::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOA::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 15)));
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PA15 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 15;
                unsafe {
                    &(*GPIOA::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 15;
                let offset2 = 4 * 15;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOA::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOA::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOA::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PA15<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Output<MODE>> {
                PA {
                    i: 15,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PA15<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 15)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (15 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PA15<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).odr.read().bits() & (1 << 15) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PA15<Output<MODE>> {}
        impl<MODE> InputPin for PA15<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 15) == 0 }
            }
        }
        impl<MODE> PA15<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PA<Input<MODE>> {
                PA {
                    i: 15,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA15<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOA::ptr()).idr.read().bits() & (1 << 15) == 0 }
            }
        }
        impl<TYPE> PA<TYPE> {
            pub fn get_id(&self) -> u8 {
                self.i
            }
        }
    }
    #[doc = " GPIO"]
    pub mod gpiob {
        use super::{
            AltMode, Analog, Floating, GpioExt, Input, OpenDrain, Output, PullDown, PullUp,
            PushPull, Speed,
        };
        use crate::hal::digital::{toggleable, InputPin, OutputPin, StatefulOutputPin};
        use crate::pac::GPIOB;
        use crate::rcc::Rcc;
        use core::marker::PhantomData;
        #[doc = " GPIO parts"]
        pub struct Parts {
            #[doc = " Pin"]
            pub pb0: PB0<Input<Floating>>,
            #[doc = " Pin"]
            pub pb1: PB1<Input<Floating>>,
            #[doc = " Pin"]
            pub pb2: PB2<Input<Floating>>,
            #[doc = " Pin"]
            pub pb3: PB3<Input<Floating>>,
            #[doc = " Pin"]
            pub pb4: PB4<Input<Floating>>,
            #[doc = " Pin"]
            pub pb5: PB5<Input<Floating>>,
            #[doc = " Pin"]
            pub pb6: PB6<Input<Floating>>,
            #[doc = " Pin"]
            pub pb7: PB7<Input<Floating>>,
            #[doc = " Pin"]
            pub pb8: PB8<Input<Floating>>,
            #[doc = " Pin"]
            pub pb9: PB9<Input<Floating>>,
            #[doc = " Pin"]
            pub pb10: PB10<Input<Floating>>,
            #[doc = " Pin"]
            pub pb11: PB11<Input<Floating>>,
            #[doc = " Pin"]
            pub pb12: PB12<Input<Floating>>,
            #[doc = " Pin"]
            pub pb13: PB13<Input<Floating>>,
            #[doc = " Pin"]
            pub pb14: PB14<Input<Floating>>,
            #[doc = " Pin"]
            pub pb15: PB15<Input<Floating>>,
        }
        impl GpioExt for GPIOB {
            type Parts = Parts;
            fn split(self, rcc: &mut Rcc) -> Parts {
                rcc.rb.iopenr.modify(|_, w| w.iopben().set_bit());
                Parts {
                    pb0: PB0 { _mode: PhantomData },
                    pb1: PB1 { _mode: PhantomData },
                    pb2: PB2 { _mode: PhantomData },
                    pb3: PB3 { _mode: PhantomData },
                    pb4: PB4 { _mode: PhantomData },
                    pb5: PB5 { _mode: PhantomData },
                    pb6: PB6 { _mode: PhantomData },
                    pb7: PB7 { _mode: PhantomData },
                    pb8: PB8 { _mode: PhantomData },
                    pb9: PB9 { _mode: PhantomData },
                    pb10: PB10 { _mode: PhantomData },
                    pb11: PB11 { _mode: PhantomData },
                    pb12: PB12 { _mode: PhantomData },
                    pb13: PB13 { _mode: PhantomData },
                    pb14: PB14 { _mode: PhantomData },
                    pb15: PB15 { _mode: PhantomData },
                }
            }
        }
        #[doc = " Partially erased pin"]
        pub struct PB<MODE> {
            i: u8,
            _mode: PhantomData<MODE>,
        }
        impl<MODE> OutputPin for PB<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << self.i)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (self.i + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << self.i) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB<Output<MODE>> {}
        impl<MODE> InputPin for PB<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << self.i) == 0 }
            }
        }
        impl<MODE> InputPin for PB<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << self.i) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PB0<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB0<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PB0<Input<Floating>> {
                let offset = 2 * 0;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB0 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PB0<Input<PullDown>> {
                let offset = 2 * 0;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB0 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PB0<Input<PullUp>> {
                let offset = 2 * 0;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB0 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PB0<Analog> {
                let offset = 2 * 0;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PB0 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PB0<Output<OpenDrain>> {
                let offset = 2 * 0;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 0)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB0 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PB0<Output<PushPull>> {
                let offset = 2 * 0;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 0)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB0 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 0;
                unsafe {
                    &(*GPIOB::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 0;
                let offset2 = 4 * 0;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOB::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOB::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PB0<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Output<MODE>> {
                PB {
                    i: 0,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PB0<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 0)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (0 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB0<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << 0) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB0<Output<MODE>> {}
        impl<MODE> InputPin for PB0<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 0) == 0 }
            }
        }
        impl<MODE> PB0<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Input<MODE>> {
                PB {
                    i: 0,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB0<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 0) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PB1<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB1<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PB1<Input<Floating>> {
                let offset = 2 * 1;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB1 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PB1<Input<PullDown>> {
                let offset = 2 * 1;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB1 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PB1<Input<PullUp>> {
                let offset = 2 * 1;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB1 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PB1<Analog> {
                let offset = 2 * 1;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PB1 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PB1<Output<OpenDrain>> {
                let offset = 2 * 1;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 1)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB1 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PB1<Output<PushPull>> {
                let offset = 2 * 1;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 1)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB1 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 1;
                unsafe {
                    &(*GPIOB::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 1;
                let offset2 = 4 * 1;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOB::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOB::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PB1<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Output<MODE>> {
                PB {
                    i: 1,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PB1<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 1)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (1 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB1<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << 1) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB1<Output<MODE>> {}
        impl<MODE> InputPin for PB1<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 1) == 0 }
            }
        }
        impl<MODE> PB1<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Input<MODE>> {
                PB {
                    i: 1,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB1<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 1) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PB2<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB2<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PB2<Input<Floating>> {
                let offset = 2 * 2;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB2 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PB2<Input<PullDown>> {
                let offset = 2 * 2;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB2 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PB2<Input<PullUp>> {
                let offset = 2 * 2;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB2 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PB2<Analog> {
                let offset = 2 * 2;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PB2 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PB2<Output<OpenDrain>> {
                let offset = 2 * 2;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 2)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB2 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PB2<Output<PushPull>> {
                let offset = 2 * 2;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 2)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB2 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 2;
                unsafe {
                    &(*GPIOB::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 2;
                let offset2 = 4 * 2;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOB::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOB::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PB2<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Output<MODE>> {
                PB {
                    i: 2,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PB2<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 2)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (2 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB2<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << 2) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB2<Output<MODE>> {}
        impl<MODE> InputPin for PB2<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 2) == 0 }
            }
        }
        impl<MODE> PB2<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Input<MODE>> {
                PB {
                    i: 2,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB2<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 2) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PB3<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB3<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PB3<Input<Floating>> {
                let offset = 2 * 3;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB3 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PB3<Input<PullDown>> {
                let offset = 2 * 3;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB3 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PB3<Input<PullUp>> {
                let offset = 2 * 3;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB3 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PB3<Analog> {
                let offset = 2 * 3;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PB3 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PB3<Output<OpenDrain>> {
                let offset = 2 * 3;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 3)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB3 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PB3<Output<PushPull>> {
                let offset = 2 * 3;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 3)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB3 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 3;
                unsafe {
                    &(*GPIOB::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 3;
                let offset2 = 4 * 3;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOB::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOB::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PB3<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Output<MODE>> {
                PB {
                    i: 3,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PB3<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 3)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (3 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB3<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << 3) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB3<Output<MODE>> {}
        impl<MODE> InputPin for PB3<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 3) == 0 }
            }
        }
        impl<MODE> PB3<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Input<MODE>> {
                PB {
                    i: 3,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB3<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 3) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PB4<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB4<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PB4<Input<Floating>> {
                let offset = 2 * 4;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB4 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PB4<Input<PullDown>> {
                let offset = 2 * 4;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB4 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PB4<Input<PullUp>> {
                let offset = 2 * 4;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB4 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PB4<Analog> {
                let offset = 2 * 4;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PB4 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PB4<Output<OpenDrain>> {
                let offset = 2 * 4;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 4)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB4 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PB4<Output<PushPull>> {
                let offset = 2 * 4;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 4)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB4 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 4;
                unsafe {
                    &(*GPIOB::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 4;
                let offset2 = 4 * 4;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOB::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOB::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PB4<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Output<MODE>> {
                PB {
                    i: 4,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PB4<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 4)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (4 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB4<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << 4) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB4<Output<MODE>> {}
        impl<MODE> InputPin for PB4<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 4) == 0 }
            }
        }
        impl<MODE> PB4<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Input<MODE>> {
                PB {
                    i: 4,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB4<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 4) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PB5<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB5<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PB5<Input<Floating>> {
                let offset = 2 * 5;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB5 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PB5<Input<PullDown>> {
                let offset = 2 * 5;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB5 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PB5<Input<PullUp>> {
                let offset = 2 * 5;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB5 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PB5<Analog> {
                let offset = 2 * 5;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PB5 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PB5<Output<OpenDrain>> {
                let offset = 2 * 5;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 5)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB5 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PB5<Output<PushPull>> {
                let offset = 2 * 5;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 5)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB5 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 5;
                unsafe {
                    &(*GPIOB::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 5;
                let offset2 = 4 * 5;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOB::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOB::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PB5<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Output<MODE>> {
                PB {
                    i: 5,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PB5<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 5)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (5 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB5<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << 5) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB5<Output<MODE>> {}
        impl<MODE> InputPin for PB5<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 5) == 0 }
            }
        }
        impl<MODE> PB5<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Input<MODE>> {
                PB {
                    i: 5,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB5<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 5) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PB6<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB6<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PB6<Input<Floating>> {
                let offset = 2 * 6;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB6 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PB6<Input<PullDown>> {
                let offset = 2 * 6;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB6 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PB6<Input<PullUp>> {
                let offset = 2 * 6;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB6 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PB6<Analog> {
                let offset = 2 * 6;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PB6 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PB6<Output<OpenDrain>> {
                let offset = 2 * 6;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 6)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB6 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PB6<Output<PushPull>> {
                let offset = 2 * 6;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 6)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB6 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 6;
                unsafe {
                    &(*GPIOB::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 6;
                let offset2 = 4 * 6;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOB::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOB::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PB6<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Output<MODE>> {
                PB {
                    i: 6,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PB6<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 6)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (6 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB6<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << 6) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB6<Output<MODE>> {}
        impl<MODE> InputPin for PB6<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 6) == 0 }
            }
        }
        impl<MODE> PB6<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Input<MODE>> {
                PB {
                    i: 6,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB6<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 6) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PB7<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB7<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PB7<Input<Floating>> {
                let offset = 2 * 7;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB7 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PB7<Input<PullDown>> {
                let offset = 2 * 7;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB7 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PB7<Input<PullUp>> {
                let offset = 2 * 7;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB7 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PB7<Analog> {
                let offset = 2 * 7;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PB7 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PB7<Output<OpenDrain>> {
                let offset = 2 * 7;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 7)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB7 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PB7<Output<PushPull>> {
                let offset = 2 * 7;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 7)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB7 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 7;
                unsafe {
                    &(*GPIOB::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 7;
                let offset2 = 4 * 7;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOB::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOB::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PB7<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Output<MODE>> {
                PB {
                    i: 7,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PB7<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 7)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (7 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB7<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << 7) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB7<Output<MODE>> {}
        impl<MODE> InputPin for PB7<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 7) == 0 }
            }
        }
        impl<MODE> PB7<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Input<MODE>> {
                PB {
                    i: 7,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB7<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 7) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PB8<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB8<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PB8<Input<Floating>> {
                let offset = 2 * 8;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB8 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PB8<Input<PullDown>> {
                let offset = 2 * 8;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB8 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PB8<Input<PullUp>> {
                let offset = 2 * 8;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB8 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PB8<Analog> {
                let offset = 2 * 8;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PB8 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PB8<Output<OpenDrain>> {
                let offset = 2 * 8;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 8)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB8 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PB8<Output<PushPull>> {
                let offset = 2 * 8;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 8)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB8 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 8;
                unsafe {
                    &(*GPIOB::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 8;
                let offset2 = 4 * 8;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOB::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOB::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PB8<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Output<MODE>> {
                PB {
                    i: 8,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PB8<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 8)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (8 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB8<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << 8) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB8<Output<MODE>> {}
        impl<MODE> InputPin for PB8<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 8) == 0 }
            }
        }
        impl<MODE> PB8<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Input<MODE>> {
                PB {
                    i: 8,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB8<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 8) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PB9<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB9<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PB9<Input<Floating>> {
                let offset = 2 * 9;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB9 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PB9<Input<PullDown>> {
                let offset = 2 * 9;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB9 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PB9<Input<PullUp>> {
                let offset = 2 * 9;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB9 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PB9<Analog> {
                let offset = 2 * 9;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PB9 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PB9<Output<OpenDrain>> {
                let offset = 2 * 9;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 9)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB9 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PB9<Output<PushPull>> {
                let offset = 2 * 9;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 9)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB9 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 9;
                unsafe {
                    &(*GPIOB::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 9;
                let offset2 = 4 * 9;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOB::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOB::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PB9<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Output<MODE>> {
                PB {
                    i: 9,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PB9<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 9)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (9 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB9<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << 9) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB9<Output<MODE>> {}
        impl<MODE> InputPin for PB9<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 9) == 0 }
            }
        }
        impl<MODE> PB9<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Input<MODE>> {
                PB {
                    i: 9,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB9<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 9) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PB10<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB10<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PB10<Input<Floating>> {
                let offset = 2 * 10;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB10 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PB10<Input<PullDown>> {
                let offset = 2 * 10;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB10 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PB10<Input<PullUp>> {
                let offset = 2 * 10;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB10 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PB10<Analog> {
                let offset = 2 * 10;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PB10 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PB10<Output<OpenDrain>> {
                let offset = 2 * 10;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 10)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB10 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PB10<Output<PushPull>> {
                let offset = 2 * 10;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 10)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB10 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 10;
                unsafe {
                    &(*GPIOB::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 10;
                let offset2 = 4 * 10;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOB::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOB::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PB10<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Output<MODE>> {
                PB {
                    i: 10,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PB10<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 10)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (10 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB10<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << 10) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB10<Output<MODE>> {}
        impl<MODE> InputPin for PB10<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 10) == 0 }
            }
        }
        impl<MODE> PB10<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Input<MODE>> {
                PB {
                    i: 10,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB10<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 10) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PB11<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB11<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PB11<Input<Floating>> {
                let offset = 2 * 11;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB11 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PB11<Input<PullDown>> {
                let offset = 2 * 11;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB11 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PB11<Input<PullUp>> {
                let offset = 2 * 11;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB11 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PB11<Analog> {
                let offset = 2 * 11;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PB11 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PB11<Output<OpenDrain>> {
                let offset = 2 * 11;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 11)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB11 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PB11<Output<PushPull>> {
                let offset = 2 * 11;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 11)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB11 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 11;
                unsafe {
                    &(*GPIOB::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 11;
                let offset2 = 4 * 11;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOB::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOB::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PB11<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Output<MODE>> {
                PB {
                    i: 11,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PB11<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 11)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (11 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB11<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << 11) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB11<Output<MODE>> {}
        impl<MODE> InputPin for PB11<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 11) == 0 }
            }
        }
        impl<MODE> PB11<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Input<MODE>> {
                PB {
                    i: 11,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB11<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 11) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PB12<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB12<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PB12<Input<Floating>> {
                let offset = 2 * 12;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB12 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PB12<Input<PullDown>> {
                let offset = 2 * 12;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB12 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PB12<Input<PullUp>> {
                let offset = 2 * 12;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB12 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PB12<Analog> {
                let offset = 2 * 12;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PB12 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PB12<Output<OpenDrain>> {
                let offset = 2 * 12;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 12)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB12 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PB12<Output<PushPull>> {
                let offset = 2 * 12;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 12)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB12 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 12;
                unsafe {
                    &(*GPIOB::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 12;
                let offset2 = 4 * 12;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOB::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOB::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PB12<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Output<MODE>> {
                PB {
                    i: 12,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PB12<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 12)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (12 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB12<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << 12) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB12<Output<MODE>> {}
        impl<MODE> InputPin for PB12<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 12) == 0 }
            }
        }
        impl<MODE> PB12<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Input<MODE>> {
                PB {
                    i: 12,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB12<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 12) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PB13<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB13<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PB13<Input<Floating>> {
                let offset = 2 * 13;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB13 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PB13<Input<PullDown>> {
                let offset = 2 * 13;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB13 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PB13<Input<PullUp>> {
                let offset = 2 * 13;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB13 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PB13<Analog> {
                let offset = 2 * 13;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PB13 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PB13<Output<OpenDrain>> {
                let offset = 2 * 13;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 13)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB13 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PB13<Output<PushPull>> {
                let offset = 2 * 13;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 13)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB13 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 13;
                unsafe {
                    &(*GPIOB::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 13;
                let offset2 = 4 * 13;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOB::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOB::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PB13<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Output<MODE>> {
                PB {
                    i: 13,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PB13<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 13)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (13 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB13<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << 13) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB13<Output<MODE>> {}
        impl<MODE> InputPin for PB13<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 13) == 0 }
            }
        }
        impl<MODE> PB13<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Input<MODE>> {
                PB {
                    i: 13,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB13<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 13) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PB14<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB14<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PB14<Input<Floating>> {
                let offset = 2 * 14;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB14 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PB14<Input<PullDown>> {
                let offset = 2 * 14;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB14 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PB14<Input<PullUp>> {
                let offset = 2 * 14;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB14 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PB14<Analog> {
                let offset = 2 * 14;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PB14 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PB14<Output<OpenDrain>> {
                let offset = 2 * 14;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 14)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB14 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PB14<Output<PushPull>> {
                let offset = 2 * 14;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 14)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB14 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 14;
                unsafe {
                    &(*GPIOB::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 14;
                let offset2 = 4 * 14;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOB::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOB::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PB14<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Output<MODE>> {
                PB {
                    i: 14,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PB14<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 14)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (14 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB14<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << 14) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB14<Output<MODE>> {}
        impl<MODE> InputPin for PB14<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 14) == 0 }
            }
        }
        impl<MODE> PB14<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Input<MODE>> {
                PB {
                    i: 14,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB14<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 14) == 0 }
            }
        }
        #[doc = " Pin"]
        pub struct PB15<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB15<MODE> {
            #[doc = " Configures the pin to operate as a floating input pin"]
            pub fn into_floating_input(self) -> PB15<Input<Floating>> {
                let offset = 2 * 15;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB15 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled down input pin"]
            pub fn into_pull_down_input(self) -> PB15<Input<PullDown>> {
                let offset = 2 * 15;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB15 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as a pulled up input pin"]
            pub fn into_pull_up_input(self) -> PB15<Input<PullUp>> {
                let offset = 2 * 15;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)))
                };
                PB15 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an analog pin"]
            pub fn into_analog(self) -> PB15<Analog> {
                let offset = 2 * 15;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (3 << offset)));
                }
                PB15 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an open drain output pin"]
            pub fn into_open_drain_output(self) -> PB15<Output<OpenDrain>> {
                let offset = 2 * 15;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (1 << 15)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB15 { _mode: PhantomData }
            }
            #[doc = " Configures the pin to operate as an push pull output pin"]
            pub fn into_push_pull_output(self) -> PB15<Output<PushPull>> {
                let offset = 2 * 15;
                unsafe {
                    &(*GPIOB::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (0 << offset)));
                    &(*GPIOB::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(1 << 15)));
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (1 << offset)))
                };
                PB15 { _mode: PhantomData }
            }
            #[doc = " Set pin speed"]
            pub fn set_speed(self, speed: Speed) -> Self {
                let offset = 2 * 15;
                unsafe {
                    &(*GPIOB::ptr()).ospeedr.modify(|r, w| {
                        w.bits((r.bits() & !(3 << offset)) | ((speed as u32) << offset))
                    })
                };
                self
            }
            #[allow(dead_code)]
            pub(crate) fn set_alt_mode(&self, mode: AltMode) {
                let mode = mode as u32;
                let offset = 2 * 15;
                let offset2 = 4 * 15;
                unsafe {
                    if offset2 < 32 {
                        &(*GPIOB::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        &(*GPIOB::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(15 << offset2)) | (mode << offset2))
                        });
                    }
                    &(*GPIOB::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & !(3 << offset)) | (2 << offset)));
                }
            }
        }
        impl<MODE> PB15<Output<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Output<MODE>> {
                PB {
                    i: 15,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for PB15<Output<MODE>> {
            fn set_high(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << 15)) }
            }
            fn set_low(&mut self) {
                unsafe { (*GPIOB::ptr()).bsrr.write(|w| w.bits(1 << (15 + 16))) }
            }
        }
        impl<MODE> StatefulOutputPin for PB15<Output<MODE>> {
            fn is_set_high(&self) -> bool {
                !self.is_set_low()
            }
            fn is_set_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).odr.read().bits() & (1 << 15) == 0 }
            }
        }
        impl<MODE> toggleable::Default for PB15<Output<MODE>> {}
        impl<MODE> InputPin for PB15<Output<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 15) == 0 }
            }
        }
        impl<MODE> PB15<Input<MODE>> {
            #[doc = " Erases the pin number from the type"]
            #[doc = ""]
            #[doc = " This is useful when you want to collect the pins into an array where you"]
            #[doc = " need all the elements to have the same type"]
            pub fn downgrade(self) -> PB<Input<MODE>> {
                PB {
                    i: 15,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB15<Input<MODE>> {
            fn is_high(&self) -> bool {
                !self.is_low()
            }
            fn is_low(&self) -> bool {
                unsafe { (*GPIOB::ptr()).idr.read().bits() & (1 << 15) == 0 }
            }
        }
        impl<TYPE> PB<TYPE> {
            pub fn get_id(&self) -> u8 {
                self.i
            }
        }
    }
}
pub mod i2c {
    #![doc = " I2C"]
    use crate::gpio::gpiob::{PB6, PB7};
    use crate::gpio::{AltMode, OpenDrain, Output};
    use crate::hal::blocking::i2c::{Read, Write, WriteRead};
    use crate::pac::I2C1;
    use crate::prelude::*;
    use crate::rcc::Rcc;
    use crate::time::Hertz;
    #[doc = " I2C abstraction"]
    pub struct I2c<I2C, PINS> {
        i2c: I2C,
        pins: PINS,
    }
    pub trait Pins<I2c> {
        fn setup(&self);
    }
    impl Pins<I2C1> for (PB6<Output<OpenDrain>>, PB7<Output<OpenDrain>>) {
        fn setup(&self) {
            self.0.set_alt_mode(AltMode::I2C);
            self.1.set_alt_mode(AltMode::I2C);
        }
    }
    pub enum Error {
        OVERRUN,
        NACK,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for Error {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match (&*self,) {
                (&Error::OVERRUN,) => {
                    let mut debug_trait_builder = f.debug_tuple("OVERRUN");
                    debug_trait_builder.finish()
                }
                (&Error::NACK,) => {
                    let mut debug_trait_builder = f.debug_tuple("NACK");
                    debug_trait_builder.finish()
                }
            }
        }
    }
    impl<PINS> I2c<I2C1, PINS> {
        pub fn i2c1(i2c: I2C1, pins: PINS, speed: Hertz, rcc: &mut Rcc) -> Self
        where
            PINS: Pins<I2C1>,
        {
            pins.setup();
            i2c.cr1.modify(|_, w| w.pe().set_bit());
            I2c { i2c, pins }
        }
        pub fn release(self) -> (I2C1, PINS) {
            (self.i2c, self.pins)
        }
        fn send_byte(&self, byte: u8) -> Result<(), Error> {
            while self.i2c.isr.read().txe().bit_is_clear() {}
            self.i2c.txdr.write(|w| unsafe { w.txdata().bits(byte) });
            while {
                let sr1 = self.i2c.isr.read();
                if sr1.nackf().bit_is_set() {
                    return Err(Error::NACK);
                }
                sr1.tcr().bit_is_clear()
            } {}
            Ok(())
        }
        fn recv_byte(&self) -> Result<u8, Error> {
            while self.i2c.isr.read().rxne().bit_is_clear() {}
            Ok(0)
        }
    }
    impl<PINS> WriteRead for I2c<I2C1, PINS> {
        type Error = Error;
        fn write_read(
            &mut self,
            addr: u8,
            bytes: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), Self::Error> {
            self.write(addr, bytes)?;
            self.read(addr, buffer)?;
            Ok(())
        }
    }
    impl<PINS> Write for I2c<I2C1, PINS> {
        type Error = Error;
        fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            self.i2c.cr2.modify(|_, w| w.start().set_bit());
            Ok(())
        }
    }
    impl<PINS> Read for I2c<I2C1, PINS> {
        type Error = Error;
        fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
            Ok(())
        }
    }
    pub trait I2c1Ext {
        fn i2c<PINS, T>(self, pins: PINS, speed: T, rcc: &mut Rcc) -> I2c<I2C1, PINS>
        where
            PINS: Pins<I2C1>,
            T: Into<Hertz>;
    }
    impl I2c1Ext for I2C1 {
        fn i2c<PINS, T>(self, pins: PINS, speed: T, rcc: &mut Rcc) -> I2c<I2C1, PINS>
        where
            PINS: Pins<I2C1>,
            T: Into<Hertz>,
        {
            I2c::i2c1(self, pins, speed.into(), rcc)
        }
    }
}
pub mod prelude {
    pub use crate::delay::DelayExt as _stm32l0xx_hal_delay_DelayExt;
    pub use crate::exti::ExtiExt as _stm32l0xx_hal_exti_ExtiExt;
    pub use crate::gpio::GpioExt as _stm32l0xx_hal_gpio_GpioExt;
    pub use crate::hal::adc::OneShot as _hal_adc_OneShot;
    pub use crate::hal::watchdog::Watchdog as _hal_watchdog_Watchdog;
    pub use crate::hal::watchdog::WatchdogEnable as _hal_watchdog_WatchdogEnable;
    pub use crate::i2c::I2c1Ext as _stm32l0xx_hal_i2c_I2c1Ext;
    pub use crate::rcc::RccExt as _stm32l0xx_hal_rcc_RccExt;
    pub use crate::serial::Serial1Ext as _stm32l0xx_hal_serial_Serial1Ext;
    pub use crate::serial::Serial2Ext as _stm32l0xx_hal_serial_Serial2Ext;
    pub use crate::spi::SpiExt as _stm32l0xx_hal_spi_SpiExt;
    pub use crate::time::U32Ext as _stm32l0xx_hal_time_U32Ext;
    pub use crate::timer::TimerExt as _stm32l0xx_hal_timer_TimerExt;
    pub use crate::watchdog::IndependedWatchdogExt as _stm32l0xx_hal_watchdog_IndependedWatchdogExt;
    pub use crate::watchdog::WindowWatchdogExt as _stm32l0xx_hal_watchdog_WindowWatchdogExt;
    pub use embedded_hal::prelude::*;
}
pub mod rcc {
    use crate::pac::RCC;
    use crate::time::{Hertz, U32Ext};
    #[doc = " System clock mux source"]
    pub enum ClockSrc {
        MSI(MSIRange),
        PLL(PLLSource, PLLMul, PLLDiv),
        HSE(Hertz),
        HSI16,
    }
    #[doc = " MSI Range"]
    #[rustc_copy_clone_marker]
    pub enum MSIRange {
        Range0 = 0,
        Range1 = 1,
        Range2 = 2,
        Range3 = 3,
        Range4 = 4,
        Range5 = 5,
        Range6 = 6,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for MSIRange {
        #[inline]
        fn clone(&self) -> MSIRange {
            {
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for MSIRange {}
    impl Default for MSIRange {
        fn default() -> MSIRange {
            MSIRange::Range5
        }
    }
    #[doc = " PLL divider"]
    #[rustc_copy_clone_marker]
    pub enum PLLDiv {
        Div2 = 1,
        Div3 = 2,
        Div4 = 3,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for PLLDiv {
        #[inline]
        fn clone(&self) -> PLLDiv {
            {
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for PLLDiv {}
    #[doc = " PLL multiplier"]
    #[rustc_copy_clone_marker]
    pub enum PLLMul {
        Mul3 = 0,
        Mul4 = 1,
        Mul6 = 2,
        Mul8 = 3,
        Mul12 = 4,
        Mul16 = 5,
        Mul24 = 6,
        Mul32 = 7,
        Mul48 = 8,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for PLLMul {
        #[inline]
        fn clone(&self) -> PLLMul {
            {
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for PLLMul {}
    #[doc = " AHB prescaler"]
    #[rustc_copy_clone_marker]
    pub enum AHBPrescaler {
        NotDivided = 0,
        Div2 = 8,
        Div4 = 9,
        Div8 = 10,
        Div16 = 11,
        Div64 = 12,
        Div128 = 13,
        Div256 = 14,
        Div512 = 15,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for AHBPrescaler {
        #[inline]
        fn clone(&self) -> AHBPrescaler {
            {
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for AHBPrescaler {}
    #[doc = " APB prescaler"]
    #[rustc_copy_clone_marker]
    pub enum APBPrescaler {
        NotDivided = 0,
        Div2 = 4,
        Div4 = 5,
        Div8 = 6,
        Div16 = 7,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for APBPrescaler {
        #[inline]
        fn clone(&self) -> APBPrescaler {
            {
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for APBPrescaler {}
    #[doc = " PLL clock input source"]
    #[rustc_copy_clone_marker]
    pub enum PLLSource {
        HSI16,
        HSE(Hertz),
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for PLLSource {
        #[inline]
        fn clone(&self) -> PLLSource {
            {
                let _: ::core::clone::AssertParamIsClone<Hertz>;
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for PLLSource {}
    #[doc = " HSI speed"]
    pub const HSI_FREQ: u32 = 16000000;
    #[doc = " Clocks configutation"]
    pub struct Config {
        mux: ClockSrc,
        ahb_pre: AHBPrescaler,
        apb1_pre: APBPrescaler,
        apb2_pre: APBPrescaler,
    }
    impl Default for Config {
        fn default() -> Config {
            Config {
                mux: ClockSrc::MSI(MSIRange::default()),
                ahb_pre: AHBPrescaler::NotDivided,
                apb1_pre: APBPrescaler::NotDivided,
                apb2_pre: APBPrescaler::NotDivided,
            }
        }
    }
    impl Config {
        pub fn clock_src(mut self, mux: ClockSrc) -> Self {
            self.mux = mux;
            self
        }
        pub fn ahb_pre(mut self, pre: AHBPrescaler) -> Self {
            self.ahb_pre = pre;
            self
        }
        pub fn apb1_pre(mut self, pre: APBPrescaler) -> Self {
            self.apb1_pre = pre;
            self
        }
        pub fn apb2_pre(mut self, pre: APBPrescaler) -> Self {
            self.apb2_pre = pre;
            self
        }
        pub fn hsi16() -> Config {
            Config {
                mux: ClockSrc::HSI16,
                ahb_pre: AHBPrescaler::NotDivided,
                apb1_pre: APBPrescaler::NotDivided,
                apb2_pre: APBPrescaler::NotDivided,
            }
        }
        pub fn msi(range: MSIRange) -> Config {
            Config {
                mux: ClockSrc::MSI(range),
                ahb_pre: AHBPrescaler::NotDivided,
                apb1_pre: APBPrescaler::NotDivided,
                apb2_pre: APBPrescaler::NotDivided,
            }
        }
        pub fn pll(pll_src: PLLSource, pll_mul: PLLMul, pll_div: PLLDiv) -> Config {
            Config {
                mux: ClockSrc::PLL(pll_src, pll_mul, pll_div),
                ahb_pre: AHBPrescaler::NotDivided,
                apb1_pre: APBPrescaler::NotDivided,
                apb2_pre: APBPrescaler::NotDivided,
            }
        }
        pub fn hse<T>(freq: T) -> Config
        where
            T: Into<Hertz>,
        {
            Config {
                mux: ClockSrc::HSE(freq.into()),
                ahb_pre: AHBPrescaler::NotDivided,
                apb1_pre: APBPrescaler::NotDivided,
                apb2_pre: APBPrescaler::NotDivided,
            }
        }
    }
    #[doc = " RCC peripheral"]
    pub struct Rcc {
        pub clocks: Clocks,
        pub(crate) rb: RCC,
    }
    #[doc = " Extension trait that freezes the `RCC` peripheral with provided clocks configuration"]
    pub trait RccExt {
        fn freeze(self, config: Config) -> Rcc;
    }
    impl RccExt for RCC {
        fn freeze(self, cfgr: Config) -> Rcc {
            let (sys_clk, sw_bits) = match cfgr.mux {
                ClockSrc::MSI(range) => {
                    let range = range as u8;
                    self.icscr.write(|w| unsafe { w.msirange().bits(range) });
                    self.cr.write(|w| w.msion().set_bit());
                    while self.cr.read().msirdy().bit_is_clear() {}
                    let freq = 32768 * (1 << (range + 1));
                    (freq, 0)
                }
                ClockSrc::HSI16 => {
                    self.cr.write(|w| w.hsi16on().set_bit());
                    while self.cr.read().hsi16rdyf().bit_is_clear() {}
                    (HSI_FREQ, 1)
                }
                ClockSrc::HSE(freq) => {
                    self.cr.write(|w| w.hseon().set_bit());
                    while self.cr.read().hserdy().bit_is_clear() {}
                    (freq.0, 2)
                }
                ClockSrc::PLL(src, mul, div) => {
                    let (src_bit, freq) = match src {
                        PLLSource::HSE(freq) => {
                            self.cr.write(|w| w.hseon().set_bit());
                            while self.cr.read().hserdy().bit_is_clear() {}
                            (true, freq.0)
                        }
                        PLLSource::HSI16 => {
                            self.cr.write(|w| w.hsi16on().set_bit());
                            while self.cr.read().hsi16rdyf().bit_is_clear() {}
                            (false, HSI_FREQ)
                        }
                    };
                    self.cr.write(|w| w.pllon().clear_bit());
                    while self.cr.read().pllrdy().bit_is_set() {}
                    let mul_bytes = mul as u8;
                    let div_bytes = div as u8;
                    let freq = match mul {
                        PLLMul::Mul3 => freq * 3,
                        PLLMul::Mul4 => freq * 4,
                        PLLMul::Mul6 => freq * 6,
                        PLLMul::Mul8 => freq * 8,
                        PLLMul::Mul12 => freq * 12,
                        PLLMul::Mul16 => freq * 16,
                        PLLMul::Mul24 => freq * 24,
                        PLLMul::Mul32 => freq * 32,
                        PLLMul::Mul48 => freq * 48,
                    };
                    let freq = match div {
                        PLLDiv::Div2 => freq / 2,
                        PLLDiv::Div3 => freq / 3,
                        PLLDiv::Div4 => freq / 4,
                    };
                    if !(freq <= 24.mhz().0) {
                        {
                            ::core::panicking::panic(&(
                                "assertion failed: freq <= 24.mhz().0",
                                "src/rcc.rs",
                                246u32,
                                17u32,
                            ))
                        }
                    };
                    self.cfgr.write(move |w| unsafe {
                        w.pllmul()
                            .bits(mul_bytes)
                            .plldiv()
                            .bits(div_bytes)
                            .pllsrc()
                            .bit(src_bit)
                    });
                    self.cr.write(|w| w.pllon().set_bit());
                    while self.cr.read().pllrdy().bit_is_clear() {}
                    (freq, 3)
                }
            };
            self.cfgr.modify(|_, w| unsafe {
                w.sw()
                    .bits(sw_bits)
                    .hpre()
                    .bits(cfgr.ahb_pre as u8)
                    .ppre1()
                    .bits(cfgr.apb1_pre as u8)
                    .ppre2()
                    .bits(cfgr.apb2_pre as u8)
            });
            let ahb_freq = match cfgr.ahb_pre {
                AHBPrescaler::NotDivided => sys_clk,
                pre => sys_clk / (1 << (pre as u8 - 7)),
            };
            let (apb1_freq, apb1_tim_freq) = match cfgr.apb1_pre {
                APBPrescaler::NotDivided => (ahb_freq, ahb_freq),
                pre => {
                    let freq = ahb_freq / (1 << (pre as u8 - 3));
                    (freq, freq * 2)
                }
            };
            let (apb2_freq, apb2_tim_freq) = match cfgr.apb2_pre {
                APBPrescaler::NotDivided => (ahb_freq, ahb_freq),
                pre => {
                    let freq = ahb_freq / (1 << (pre as u8 - 3));
                    (freq, freq * 2)
                }
            };
            let clocks = Clocks {
                sys_clk: sys_clk.hz(),
                ahb_clk: ahb_freq.hz(),
                apb1_clk: apb1_freq.hz(),
                apb2_clk: apb2_freq.hz(),
                apb1_tim_clk: apb1_tim_freq.hz(),
                apb2_tim_clk: apb2_tim_freq.hz(),
            };
            Rcc { rb: self, clocks }
        }
    }
    #[doc = " Frozen clock frequencies"]
    #[doc = ""]
    #[doc = " The existence of this value indicates that the clock configuration can no longer be changed"]
    #[rustc_copy_clone_marker]
    pub struct Clocks {
        sys_clk: Hertz,
        ahb_clk: Hertz,
        apb1_clk: Hertz,
        apb1_tim_clk: Hertz,
        apb2_clk: Hertz,
        apb2_tim_clk: Hertz,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for Clocks {
        #[inline]
        fn clone(&self) -> Clocks {
            {
                let _: ::core::clone::AssertParamIsClone<Hertz>;
                let _: ::core::clone::AssertParamIsClone<Hertz>;
                let _: ::core::clone::AssertParamIsClone<Hertz>;
                let _: ::core::clone::AssertParamIsClone<Hertz>;
                let _: ::core::clone::AssertParamIsClone<Hertz>;
                let _: ::core::clone::AssertParamIsClone<Hertz>;
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for Clocks {}
    impl Clocks {
        #[doc = " Returns the system (core) frequency"]
        pub fn sys_clk(&self) -> Hertz {
            self.sys_clk
        }
        #[doc = " Returns the frequency of the AHB"]
        pub fn ahb_clk(&self) -> Hertz {
            self.ahb_clk
        }
        #[doc = " Returns the frequency of the APB1"]
        pub fn apb1_clk(&self) -> Hertz {
            self.apb1_clk
        }
        #[doc = " Returns the frequency of the APB1 timers"]
        pub fn apb1_tim_clk(&self) -> Hertz {
            self.apb1_tim_clk
        }
        #[doc = " Returns the frequency of the APB2"]
        pub fn apb2_clk(&self) -> Hertz {
            self.apb2_clk
        }
        #[doc = " Returns the frequency of the APB2 timers"]
        pub fn apb2_tim_clk(&self) -> Hertz {
            self.apb2_tim_clk
        }
    }
}
pub mod serial {
    use crate::gpio::gpioa::{PA10, PA14, PA15, PA2, PA3, PA9};
    use crate::gpio::{AltMode, Floating, Input};
    use crate::hal;
    use crate::hal::prelude::*;
    use crate::pac::{USART1, USART2};
    use crate::rcc::Rcc;
    use core::fmt;
    use core::marker::PhantomData;
    use core::ptr;
    use nb::block;
    #[doc = " Serial error"]
    pub enum Error {
        #[doc = " Framing error"]
        Framing,
        #[doc = " Noise error"]
        Noise,
        #[doc = " RX buffer overrun"]
        Overrun,
        #[doc = " Parity check error"]
        Parity,
        #[doc(hidden)]
        _Extensible,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for Error {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match (&*self,) {
                (&Error::Framing,) => {
                    let mut debug_trait_builder = f.debug_tuple("Framing");
                    debug_trait_builder.finish()
                }
                (&Error::Noise,) => {
                    let mut debug_trait_builder = f.debug_tuple("Noise");
                    debug_trait_builder.finish()
                }
                (&Error::Overrun,) => {
                    let mut debug_trait_builder = f.debug_tuple("Overrun");
                    debug_trait_builder.finish()
                }
                (&Error::Parity,) => {
                    let mut debug_trait_builder = f.debug_tuple("Parity");
                    debug_trait_builder.finish()
                }
                (&Error::_Extensible,) => {
                    let mut debug_trait_builder = f.debug_tuple("_Extensible");
                    debug_trait_builder.finish()
                }
            }
        }
    }
    #[doc = " Interrupt event"]
    pub enum Event {
        #[doc = " New data has been received"]
        Rxne,
        #[doc = " New data can be sent"]
        Txe,
        #[doc = " Idle line state detected"]
        Idle,
    }
    use crate::time::Bps;
    use crate::time::U32Ext;
    pub enum WordLength {
        DataBits8,
        DataBits9,
    }
    pub enum Parity {
        ParityNone,
        ParityEven,
        ParityOdd,
    }
    pub enum StopBits {
        #[doc = "1 stop bit"]
        STOP1,
        #[doc = "0.5 stop bits"]
        STOP0P5,
        #[doc = "2 stop bits"]
        STOP2,
        #[doc = "1.5 stop bits"]
        STOP1P5,
    }
    pub struct Config {
        pub baudrate: Bps,
        pub wordlength: WordLength,
        pub parity: Parity,
        pub stopbits: StopBits,
    }
    impl Config {
        pub fn baudrate(mut self, baudrate: Bps) -> Self {
            self.baudrate = baudrate;
            self
        }
        pub fn parity_none(mut self) -> Self {
            self.parity = Parity::ParityNone;
            self
        }
        pub fn parity_even(mut self) -> Self {
            self.parity = Parity::ParityEven;
            self
        }
        pub fn parity_odd(mut self) -> Self {
            self.parity = Parity::ParityOdd;
            self
        }
        pub fn wordlength_8(mut self) -> Self {
            self.wordlength = WordLength::DataBits8;
            self
        }
        pub fn wordlength_9(mut self) -> Self {
            self.wordlength = WordLength::DataBits9;
            self
        }
        pub fn stopbits(mut self, stopbits: StopBits) -> Self {
            self.stopbits = stopbits;
            self
        }
    }
    pub struct InvalidConfig;
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for InvalidConfig {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                InvalidConfig => {
                    let mut debug_trait_builder = f.debug_tuple("InvalidConfig");
                    debug_trait_builder.finish()
                }
            }
        }
    }
    impl Default for Config {
        fn default() -> Config {
            let baudrate = 9600u32.bps();
            Config {
                baudrate,
                wordlength: WordLength::DataBits8,
                parity: Parity::ParityNone,
                stopbits: StopBits::STOP1,
            }
        }
    }
    pub trait Pins<USART> {
        fn setup(&self);
    }
    #[cfg(feature = "stm32l0x2")]
    impl Pins<USART1> for (PA9<Input<Floating>>, PA10<Input<Floating>>) {
        fn setup(&self) {
            self.0.set_alt_mode(AltMode::AF4);
            self.1.set_alt_mode(AltMode::AF4);
        }
    }
    #[cfg(feature = "stm32l0x2")]
    impl Pins<USART2> for (PA14<Input<Floating>>, PA15<Input<Floating>>) {
        fn setup(&self) {
            self.0.set_alt_mode(AltMode::AF4);
            self.1.set_alt_mode(AltMode::AF4);
        }
    }
    #[doc = " Serial abstraction"]
    pub struct Serial<USART> {
        usart: USART,
        rx: Rx<USART>,
        tx: Tx<USART>,
    }
    #[doc = " Serial receiver"]
    pub struct Rx<USART> {
        _usart: PhantomData<USART>,
    }
    #[doc = " Serial transmitter"]
    pub struct Tx<USART> {
        _usart: PhantomData<USART>,
    }
    pub trait Serial1Ext<PINS> {
        fn usart(
            self,
            pins: PINS,
            config: Config,
            rcc: &mut Rcc,
        ) -> Result<Serial<USART1>, InvalidConfig>;
    }
    impl<PINS> Serial1Ext<PINS> for USART1
    where
        PINS: Pins<USART1>,
    {
        fn usart(
            self,
            pins: PINS,
            config: Config,
            rcc: &mut Rcc,
        ) -> Result<Serial<USART1>, InvalidConfig> {
            Serial::usart1(self, pins, config, rcc)
        }
    }
    impl Serial<USART1> {
        pub fn usart1<PINS>(
            usart: USART1,
            pins: PINS,
            config: Config,
            rcc: &mut Rcc,
        ) -> Result<Self, InvalidConfig>
        where
            PINS: Pins<USART1>,
        {
            pins.setup();
            rcc.rb.apb2enr.modify(|_, w| w.usart1en().set_bit());
            let div = (rcc.clocks.apb2_clk().0 * 25) / (4 * config.baudrate.0);
            let mantissa = div / 100;
            let fraction = ((div - mantissa * 100) * 16 + 50) / 100;
            usart
                .brr
                .write(|w| unsafe { w.bits(mantissa << 4 | fraction) });
            usart.cr2.reset();
            usart.cr3.reset();
            usart.cr1.write(|w| {
                w.ue()
                    .set_bit()
                    .te()
                    .set_bit()
                    .re()
                    .set_bit()
                    .m0()
                    .bit(match config.wordlength {
                        WordLength::DataBits8 => false,
                        WordLength::DataBits9 => true,
                    })
                    .pce()
                    .bit(match config.parity {
                        Parity::ParityNone => false,
                        _ => true,
                    })
                    .ps()
                    .bit(match config.parity {
                        Parity::ParityOdd => true,
                        _ => false,
                    })
            });
            usart.cr2.write(|w| unsafe {
                w.stop().bits(match config.stopbits {
                    StopBits::STOP1 => 0,
                    StopBits::STOP0P5 => 1,
                    StopBits::STOP2 => 2,
                    StopBits::STOP1P5 => 3,
                })
            });
            Ok(Serial {
                usart,
                tx: Tx {
                    _usart: PhantomData,
                },
                rx: Rx {
                    _usart: PhantomData,
                },
            })
        }
        #[doc = " Starts listening for an interrupt event"]
        pub fn listen(&mut self, event: Event) {
            match event {
                Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().set_bit()),
                Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().set_bit()),
                Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().set_bit()),
            }
        }
        #[doc = " Stop listening for an interrupt event"]
        pub fn unlisten(&mut self, event: Event) {
            match event {
                Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().clear_bit()),
                Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().clear_bit()),
                Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().clear_bit()),
            }
        }
        #[doc = " Clears interrupt flag"]
        pub fn clear_irq(&mut self, event: Event) {
            if let Event::Rxne = event {
                self.usart.rqr.write(|w| unsafe { w.rxfrq().discard() })
            }
        }
        pub fn split(self) -> (Tx<USART1>, Rx<USART1>) {
            (self.tx, self.rx)
        }
        pub fn release(self) -> USART1 {
            self.usart
        }
    }
    impl hal::serial::Read<u8> for Serial<USART1> {
        type Error = Error;
        fn read(&mut self) -> nb::Result<u8, Error> {
            self.rx.read()
        }
    }
    impl hal::serial::Write<u8> for Serial<USART1> {
        type Error = Error;
        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            self.tx.flush()
        }
        fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
            self.tx.write(byte)
        }
    }
    impl hal::serial::Read<u8> for Rx<USART1> {
        type Error = Error;
        fn read(&mut self) -> nb::Result<u8, Error> {
            let isr = unsafe { (*USART1::ptr()).isr.read() };
            let err = if isr.pe().bit_is_set() {
                Some(Error::Parity)
            } else if isr.fe().bit_is_set() {
                Some(Error::Framing)
            } else if isr.nf().bit_is_set() {
                Some(Error::Noise)
            } else if isr.ore().bit_is_set() {
                Some(Error::Overrun)
            } else {
                None
            };
            if let Some(err) = err {
                unsafe {
                    (*USART1::ptr()).icr.write(|w| w.pecf().set_bit());
                    (*USART1::ptr()).icr.write(|w| w.fecf().set_bit());
                    (*USART1::ptr()).icr.write(|w| w.ncf().set_bit());
                    (*USART1::ptr()).icr.write(|w| w.orecf().set_bit());
                }
                Err(nb::Error::Other(err))
            } else {
                if isr.rxne().bit_is_set() {
                    Ok(
                        unsafe {
                            ptr::read_volatile(&(*USART1::ptr()).rdr as *const _ as *const _)
                        },
                    )
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
        }
    }
    impl hal::serial::Write<u8> for Tx<USART1> {
        type Error = Error;
        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            let isr = unsafe { (*USART1::ptr()).isr.read() };
            unsafe {
                (*USART1::ptr()).icr.write(|w| w.tccf().set_bit());
            }
            if isr.tc().bit_is_set() {
                Ok(())
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
        fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
            let isr = unsafe { (*USART1::ptr()).isr.read() };
            if isr.txe().bit_is_set() {
                unsafe { ptr::write_volatile(&(*USART1::ptr()).tdr as *const _ as *mut _, byte) }
                Ok(())
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }
    pub trait Serial2Ext<PINS> {
        fn usart(
            self,
            pins: PINS,
            config: Config,
            rcc: &mut Rcc,
        ) -> Result<Serial<USART2>, InvalidConfig>;
    }
    impl<PINS> Serial2Ext<PINS> for USART2
    where
        PINS: Pins<USART2>,
    {
        fn usart(
            self,
            pins: PINS,
            config: Config,
            rcc: &mut Rcc,
        ) -> Result<Serial<USART2>, InvalidConfig> {
            Serial::usart2(self, pins, config, rcc)
        }
    }
    impl Serial<USART2> {
        pub fn usart2<PINS>(
            usart: USART2,
            pins: PINS,
            config: Config,
            rcc: &mut Rcc,
        ) -> Result<Self, InvalidConfig>
        where
            PINS: Pins<USART2>,
        {
            pins.setup();
            rcc.rb.apb1enr.modify(|_, w| w.usart2en().set_bit());
            let div = (rcc.clocks.apb1_clk().0 * 25) / (4 * config.baudrate.0);
            let mantissa = div / 100;
            let fraction = ((div - mantissa * 100) * 16 + 50) / 100;
            usart
                .brr
                .write(|w| unsafe { w.bits(mantissa << 4 | fraction) });
            usart.cr2.reset();
            usart.cr3.reset();
            usart.cr1.write(|w| {
                w.ue()
                    .set_bit()
                    .te()
                    .set_bit()
                    .re()
                    .set_bit()
                    .m0()
                    .bit(match config.wordlength {
                        WordLength::DataBits8 => false,
                        WordLength::DataBits9 => true,
                    })
                    .pce()
                    .bit(match config.parity {
                        Parity::ParityNone => false,
                        _ => true,
                    })
                    .ps()
                    .bit(match config.parity {
                        Parity::ParityOdd => true,
                        _ => false,
                    })
            });
            usart.cr2.write(|w| unsafe {
                w.stop().bits(match config.stopbits {
                    StopBits::STOP1 => 0,
                    StopBits::STOP0P5 => 1,
                    StopBits::STOP2 => 2,
                    StopBits::STOP1P5 => 3,
                })
            });
            Ok(Serial {
                usart,
                tx: Tx {
                    _usart: PhantomData,
                },
                rx: Rx {
                    _usart: PhantomData,
                },
            })
        }
        #[doc = " Starts listening for an interrupt event"]
        pub fn listen(&mut self, event: Event) {
            match event {
                Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().set_bit()),
                Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().set_bit()),
                Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().set_bit()),
            }
        }
        #[doc = " Stop listening for an interrupt event"]
        pub fn unlisten(&mut self, event: Event) {
            match event {
                Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().clear_bit()),
                Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().clear_bit()),
                Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().clear_bit()),
            }
        }
        #[doc = " Clears interrupt flag"]
        pub fn clear_irq(&mut self, event: Event) {
            if let Event::Rxne = event {
                self.usart.rqr.write(|w| unsafe { w.rxfrq().discard() })
            }
        }
        pub fn split(self) -> (Tx<USART2>, Rx<USART2>) {
            (self.tx, self.rx)
        }
        pub fn release(self) -> USART2 {
            self.usart
        }
    }
    impl hal::serial::Read<u8> for Serial<USART2> {
        type Error = Error;
        fn read(&mut self) -> nb::Result<u8, Error> {
            self.rx.read()
        }
    }
    impl hal::serial::Write<u8> for Serial<USART2> {
        type Error = Error;
        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            self.tx.flush()
        }
        fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
            self.tx.write(byte)
        }
    }
    impl hal::serial::Read<u8> for Rx<USART2> {
        type Error = Error;
        fn read(&mut self) -> nb::Result<u8, Error> {
            let isr = unsafe { (*USART2::ptr()).isr.read() };
            let err = if isr.pe().bit_is_set() {
                Some(Error::Parity)
            } else if isr.fe().bit_is_set() {
                Some(Error::Framing)
            } else if isr.nf().bit_is_set() {
                Some(Error::Noise)
            } else if isr.ore().bit_is_set() {
                Some(Error::Overrun)
            } else {
                None
            };
            if let Some(err) = err {
                unsafe {
                    (*USART2::ptr()).icr.write(|w| w.pecf().set_bit());
                    (*USART2::ptr()).icr.write(|w| w.fecf().set_bit());
                    (*USART2::ptr()).icr.write(|w| w.ncf().set_bit());
                    (*USART2::ptr()).icr.write(|w| w.orecf().set_bit());
                }
                Err(nb::Error::Other(err))
            } else {
                if isr.rxne().bit_is_set() {
                    Ok(
                        unsafe {
                            ptr::read_volatile(&(*USART2::ptr()).rdr as *const _ as *const _)
                        },
                    )
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
        }
    }
    impl hal::serial::Write<u8> for Tx<USART2> {
        type Error = Error;
        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            let isr = unsafe { (*USART2::ptr()).isr.read() };
            unsafe {
                (*USART2::ptr()).icr.write(|w| w.tccf().set_bit());
            }
            if isr.tc().bit_is_set() {
                Ok(())
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
        fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
            let isr = unsafe { (*USART2::ptr()).isr.read() };
            if isr.txe().bit_is_set() {
                unsafe { ptr::write_volatile(&(*USART2::ptr()).tdr as *const _ as *mut _, byte) }
                Ok(())
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }
    impl<USART> fmt::Write for Serial<USART>
    where
        Serial<USART>: hal::serial::Write<u8>,
    {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            let _ = s
                .as_bytes()
                .into_iter()
                .map(|c| loop {
                    #[allow(unreachable_patterns)]
                    match self.write(*c) {
                        Err(::nb::Error::Other(e)) => {
                            #[allow(unreachable_code)]
                            break Err(e)
                        }
                        Err(::nb::Error::WouldBlock) => {}
                        Ok(x) => break Ok(x),
                    }
                })
                .last();
            self.flush();
            Ok(())
        }
    }
    impl<USART> fmt::Write for Tx<USART>
    where
        Tx<USART>: hal::serial::Write<u8>,
    {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            let _ = s
                .as_bytes()
                .into_iter()
                .map(|c| loop {
                    #[allow(unreachable_patterns)]
                    match self.write(*c) {
                        Err(::nb::Error::Other(e)) => {
                            #[allow(unreachable_code)]
                            break Err(e)
                        }
                        Err(::nb::Error::WouldBlock) => {}
                        Ok(x) => break Ok(x),
                    }
                })
                .last();
            self.flush();
            Ok(())
        }
    }
}
pub mod spi {
    #[cfg(feature = "stm32l0x2")]
    use crate::gpio::gpioa::{PA6, PA7};
    #[cfg(feature = "stm32l0x2")]
    use crate::gpio::gpiob::PB3;
    use crate::gpio::{Floating, Input};
    use crate::hal;
    use crate::pac::SPI1;
    use crate::rcc::Rcc;
    use crate::time::Hertz;
    use core::ptr;
    pub use hal::spi::{Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3};
    use nb;
    #[doc = " SPI error"]
    pub enum Error {
        #[doc = " Overrun occurred"]
        Overrun,
        #[doc = " Mode fault occurred"]
        ModeFault,
        #[doc = " CRC error"]
        Crc,
        #[doc(hidden)]
        _Extensible,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for Error {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match (&*self,) {
                (&Error::Overrun,) => {
                    let mut debug_trait_builder = f.debug_tuple("Overrun");
                    debug_trait_builder.finish()
                }
                (&Error::ModeFault,) => {
                    let mut debug_trait_builder = f.debug_tuple("ModeFault");
                    debug_trait_builder.finish()
                }
                (&Error::Crc,) => {
                    let mut debug_trait_builder = f.debug_tuple("Crc");
                    debug_trait_builder.finish()
                }
                (&Error::_Extensible,) => {
                    let mut debug_trait_builder = f.debug_tuple("_Extensible");
                    debug_trait_builder.finish()
                }
            }
        }
    }
    pub trait Pins<SPI> {}
    pub trait PinSck<SPI> {}
    pub trait PinMiso<SPI> {}
    pub trait PinMosi<SPI> {}
    impl<SPI, SCK, MISO, MOSI> Pins<SPI> for (SCK, MISO, MOSI)
    where
        SCK: PinSck<SPI>,
        MISO: PinMiso<SPI>,
        MOSI: PinMosi<SPI>,
    {
    }
    #[doc = " A filler type for when the SCK pin is unnecessary"]
    pub struct NoSck;
    #[doc = " A filler type for when the Miso pin is unnecessary"]
    pub struct NoMiso;
    #[doc = " A filler type for when the Mosi pin is unnecessary"]
    pub struct NoMosi;
    impl PinSck<SPI1> for NoSck {}
    impl PinSck<SPI1> for PB3<Input<Floating>> {}
    impl PinMiso<SPI1> for NoMiso {}
    impl PinMiso<SPI1> for PA6<Input<Floating>> {}
    impl PinMosi<SPI1> for NoMosi {}
    impl PinMosi<SPI1> for PA7<Input<Floating>> {}
    pub struct Spi<SPI, PINS> {
        spi: SPI,
        pins: PINS,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl<SPI: ::core::fmt::Debug, PINS: ::core::fmt::Debug> ::core::fmt::Debug for Spi<SPI, PINS> {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                Spi {
                    spi: ref __self_0_0,
                    pins: ref __self_0_1,
                } => {
                    let mut debug_trait_builder = f.debug_struct("Spi");
                    let _ = debug_trait_builder.field("spi", &&(*__self_0_0));
                    let _ = debug_trait_builder.field("pins", &&(*__self_0_1));
                    debug_trait_builder.finish()
                }
            }
        }
    }
    pub trait SpiExt<SPI>: Sized {
        fn spi<PINS, T>(self, pins: PINS, mode: Mode, freq: T, rcc: &mut Rcc) -> Spi<SPI, PINS>
        where
            PINS: Pins<SPI>,
            T: Into<Hertz>;
    }
    impl<PINS> Spi<SPI1, PINS> {
        pub fn spi1<T>(spi: SPI1, pins: PINS, mode: Mode, freq: T, rcc: &mut Rcc) -> Self
        where
            PINS: Pins<SPI1>,
            T: Into<Hertz>,
        {
            rcc.rb.apb2enr.modify(|_, w| w.spi1en().set_bit());
            spi.cr2.write(|w| w.ssoe().clear_bit());
            let spi_freq = freq.into().0;
            let apb_freq = rcc.clocks.apb2_clk().0;
            let br = match apb_freq / spi_freq {
                0 => ::core::panicking::panic(&(
                    "internal error: entered unreachable code",
                    "src/spi.rs",
                    254u32,
                    1u32,
                )),
                1...2 => 0,
                3...5 => 1,
                6...11 => 2,
                12...23 => 3,
                24...47 => 4,
                48...95 => 5,
                96...191 => 6,
                _ => 7,
            };
            #[allow(unused)]
            spi.cr1.write(|w| unsafe {
                w.cpha()
                    .bit(mode.phase == Phase::CaptureOnSecondTransition)
                    .cpol()
                    .bit(mode.polarity == Polarity::IdleHigh)
                    .mstr()
                    .set_bit()
                    .br()
                    .bits(br)
                    .lsbfirst()
                    .clear_bit()
                    .ssm()
                    .set_bit()
                    .ssi()
                    .set_bit()
                    .rxonly()
                    .clear_bit()
                    .dff()
                    .clear_bit()
                    .bidimode()
                    .clear_bit()
                    .spe()
                    .set_bit()
            });
            Spi { spi, pins }
        }
        pub fn free(self) -> (SPI1, PINS) {
            (self.spi, self.pins)
        }
    }
    impl SpiExt<SPI1> for SPI1 {
        fn spi<PINS, T>(self, pins: PINS, mode: Mode, freq: T, rcc: &mut Rcc) -> Spi<SPI1, PINS>
        where
            PINS: Pins<SPI1>,
            T: Into<Hertz>,
        {
            Spi::spi1(self, pins, mode, freq, rcc)
        }
    }
    impl<PINS> hal::spi::FullDuplex<u8> for Spi<SPI1, PINS> {
        type Error = Error;
        fn read(&mut self) -> nb::Result<u8, Error> {
            let sr = self.spi.sr.read();
            Err(if sr.ovr().bit_is_set() {
                nb::Error::Other(Error::Overrun)
            } else if sr.modf().bit_is_set() {
                nb::Error::Other(Error::ModeFault)
            } else if sr.crcerr().bit_is_set() {
                nb::Error::Other(Error::Crc)
            } else if sr.rxne().bit_is_set() {
                return Ok(unsafe { ptr::read_volatile(&self.spi.dr as *const _ as *const u8) });
            } else {
                nb::Error::WouldBlock
            })
        }
        fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
            let sr = self.spi.sr.read();
            Err(if sr.ovr().bit_is_set() {
                nb::Error::Other(Error::Overrun)
            } else if sr.modf().bit_is_set() {
                nb::Error::Other(Error::ModeFault)
            } else if sr.crcerr().bit_is_set() {
                nb::Error::Other(Error::Crc)
            } else if sr.txe().bit_is_set() {
                unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, byte) }
                return Ok(());
            } else {
                nb::Error::WouldBlock
            })
        }
    }
    impl<PINS> crate::hal::blocking::spi::transfer::Default<u8> for Spi<SPI1, PINS> {}
    impl<PINS> crate::hal::blocking::spi::write::Default<u8> for Spi<SPI1, PINS> {}
}
pub mod time {
    use cortex_m::peripheral::DWT;
    #[rustc_copy_clone_marker]
    pub struct Bps(pub u32);
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialEq for Bps {
        #[inline]
        fn eq(&self, other: &Bps) -> bool {
            match *other {
                Bps(ref __self_1_0) => match *self {
                    Bps(ref __self_0_0) => (*__self_0_0) == (*__self_1_0),
                },
            }
        }
        #[inline]
        fn ne(&self, other: &Bps) -> bool {
            match *other {
                Bps(ref __self_1_0) => match *self {
                    Bps(ref __self_0_0) => (*__self_0_0) != (*__self_1_0),
                },
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialOrd for Bps {
        #[inline]
        fn partial_cmp(&self, other: &Bps) -> ::core::option::Option<::core::cmp::Ordering> {
            match *other {
                Bps(ref __self_1_0) => match *self {
                    Bps(ref __self_0_0) => {
                        match ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)) {
                            ::core::option::Option::Some(::core::cmp::Ordering::Equal) => {
                                ::core::option::Option::Some(::core::cmp::Ordering::Equal)
                            }
                            cmp => cmp,
                        }
                    }
                },
            }
        }
        #[inline]
        fn lt(&self, other: &Bps) -> bool {
            match *other {
                Bps(ref __self_1_0) => match *self {
                    Bps(ref __self_0_0) => {
                        ::core::option::Option::unwrap_or(
                            ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)),
                            ::core::cmp::Ordering::Greater,
                        ) == ::core::cmp::Ordering::Less
                    }
                },
            }
        }
        #[inline]
        fn le(&self, other: &Bps) -> bool {
            match *other {
                Bps(ref __self_1_0) => match *self {
                    Bps(ref __self_0_0) => {
                        ::core::option::Option::unwrap_or(
                            ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)),
                            ::core::cmp::Ordering::Greater,
                        ) != ::core::cmp::Ordering::Greater
                    }
                },
            }
        }
        #[inline]
        fn gt(&self, other: &Bps) -> bool {
            match *other {
                Bps(ref __self_1_0) => match *self {
                    Bps(ref __self_0_0) => {
                        ::core::option::Option::unwrap_or(
                            ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)),
                            ::core::cmp::Ordering::Less,
                        ) == ::core::cmp::Ordering::Greater
                    }
                },
            }
        }
        #[inline]
        fn ge(&self, other: &Bps) -> bool {
            match *other {
                Bps(ref __self_1_0) => match *self {
                    Bps(ref __self_0_0) => {
                        ::core::option::Option::unwrap_or(
                            ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)),
                            ::core::cmp::Ordering::Less,
                        ) != ::core::cmp::Ordering::Less
                    }
                },
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for Bps {
        #[inline]
        fn clone(&self) -> Bps {
            {
                let _: ::core::clone::AssertParamIsClone<u32>;
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for Bps {}
    #[rustc_copy_clone_marker]
    pub struct Hertz(pub u32);
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialEq for Hertz {
        #[inline]
        fn eq(&self, other: &Hertz) -> bool {
            match *other {
                Hertz(ref __self_1_0) => match *self {
                    Hertz(ref __self_0_0) => (*__self_0_0) == (*__self_1_0),
                },
            }
        }
        #[inline]
        fn ne(&self, other: &Hertz) -> bool {
            match *other {
                Hertz(ref __self_1_0) => match *self {
                    Hertz(ref __self_0_0) => (*__self_0_0) != (*__self_1_0),
                },
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialOrd for Hertz {
        #[inline]
        fn partial_cmp(&self, other: &Hertz) -> ::core::option::Option<::core::cmp::Ordering> {
            match *other {
                Hertz(ref __self_1_0) => match *self {
                    Hertz(ref __self_0_0) => {
                        match ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)) {
                            ::core::option::Option::Some(::core::cmp::Ordering::Equal) => {
                                ::core::option::Option::Some(::core::cmp::Ordering::Equal)
                            }
                            cmp => cmp,
                        }
                    }
                },
            }
        }
        #[inline]
        fn lt(&self, other: &Hertz) -> bool {
            match *other {
                Hertz(ref __self_1_0) => match *self {
                    Hertz(ref __self_0_0) => {
                        ::core::option::Option::unwrap_or(
                            ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)),
                            ::core::cmp::Ordering::Greater,
                        ) == ::core::cmp::Ordering::Less
                    }
                },
            }
        }
        #[inline]
        fn le(&self, other: &Hertz) -> bool {
            match *other {
                Hertz(ref __self_1_0) => match *self {
                    Hertz(ref __self_0_0) => {
                        ::core::option::Option::unwrap_or(
                            ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)),
                            ::core::cmp::Ordering::Greater,
                        ) != ::core::cmp::Ordering::Greater
                    }
                },
            }
        }
        #[inline]
        fn gt(&self, other: &Hertz) -> bool {
            match *other {
                Hertz(ref __self_1_0) => match *self {
                    Hertz(ref __self_0_0) => {
                        ::core::option::Option::unwrap_or(
                            ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)),
                            ::core::cmp::Ordering::Less,
                        ) == ::core::cmp::Ordering::Greater
                    }
                },
            }
        }
        #[inline]
        fn ge(&self, other: &Hertz) -> bool {
            match *other {
                Hertz(ref __self_1_0) => match *self {
                    Hertz(ref __self_0_0) => {
                        ::core::option::Option::unwrap_or(
                            ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)),
                            ::core::cmp::Ordering::Less,
                        ) != ::core::cmp::Ordering::Less
                    }
                },
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for Hertz {
        #[inline]
        fn clone(&self) -> Hertz {
            {
                let _: ::core::clone::AssertParamIsClone<u32>;
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for Hertz {}
    #[rustc_copy_clone_marker]
    pub struct MicroSeconds(pub u32);
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialEq for MicroSeconds {
        #[inline]
        fn eq(&self, other: &MicroSeconds) -> bool {
            match *other {
                MicroSeconds(ref __self_1_0) => match *self {
                    MicroSeconds(ref __self_0_0) => (*__self_0_0) == (*__self_1_0),
                },
            }
        }
        #[inline]
        fn ne(&self, other: &MicroSeconds) -> bool {
            match *other {
                MicroSeconds(ref __self_1_0) => match *self {
                    MicroSeconds(ref __self_0_0) => (*__self_0_0) != (*__self_1_0),
                },
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialOrd for MicroSeconds {
        #[inline]
        fn partial_cmp(
            &self,
            other: &MicroSeconds,
        ) -> ::core::option::Option<::core::cmp::Ordering> {
            match *other {
                MicroSeconds(ref __self_1_0) => match *self {
                    MicroSeconds(ref __self_0_0) => {
                        match ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)) {
                            ::core::option::Option::Some(::core::cmp::Ordering::Equal) => {
                                ::core::option::Option::Some(::core::cmp::Ordering::Equal)
                            }
                            cmp => cmp,
                        }
                    }
                },
            }
        }
        #[inline]
        fn lt(&self, other: &MicroSeconds) -> bool {
            match *other {
                MicroSeconds(ref __self_1_0) => match *self {
                    MicroSeconds(ref __self_0_0) => {
                        ::core::option::Option::unwrap_or(
                            ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)),
                            ::core::cmp::Ordering::Greater,
                        ) == ::core::cmp::Ordering::Less
                    }
                },
            }
        }
        #[inline]
        fn le(&self, other: &MicroSeconds) -> bool {
            match *other {
                MicroSeconds(ref __self_1_0) => match *self {
                    MicroSeconds(ref __self_0_0) => {
                        ::core::option::Option::unwrap_or(
                            ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)),
                            ::core::cmp::Ordering::Greater,
                        ) != ::core::cmp::Ordering::Greater
                    }
                },
            }
        }
        #[inline]
        fn gt(&self, other: &MicroSeconds) -> bool {
            match *other {
                MicroSeconds(ref __self_1_0) => match *self {
                    MicroSeconds(ref __self_0_0) => {
                        ::core::option::Option::unwrap_or(
                            ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)),
                            ::core::cmp::Ordering::Less,
                        ) == ::core::cmp::Ordering::Greater
                    }
                },
            }
        }
        #[inline]
        fn ge(&self, other: &MicroSeconds) -> bool {
            match *other {
                MicroSeconds(ref __self_1_0) => match *self {
                    MicroSeconds(ref __self_0_0) => {
                        ::core::option::Option::unwrap_or(
                            ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)),
                            ::core::cmp::Ordering::Less,
                        ) != ::core::cmp::Ordering::Less
                    }
                },
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for MicroSeconds {
        #[inline]
        fn clone(&self) -> MicroSeconds {
            {
                let _: ::core::clone::AssertParamIsClone<u32>;
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for MicroSeconds {}
    #[doc = " Extension trait that adds convenience methods to the `u32` type"]
    pub trait U32Ext {
        #[doc = " Wrap in `Bps`"]
        fn bps(self) -> Bps;
        #[doc = " Wrap in `Hertz`"]
        fn hz(self) -> Hertz;
        #[doc = " Wrap in `Hertz`"]
        fn khz(self) -> Hertz;
        #[doc = " Wrap in `Hertz`"]
        fn mhz(self) -> Hertz;
        #[doc = " Wrap in `MicroSeconds`"]
        fn us(self) -> MicroSeconds;
        #[doc = " Wrap in `MicroSeconds`"]
        fn ms(self) -> MicroSeconds;
    }
    impl U32Ext for u32 {
        fn bps(self) -> Bps {
            Bps(self)
        }
        fn hz(self) -> Hertz {
            Hertz(self)
        }
        fn khz(self) -> Hertz {
            Hertz(self * 1000)
        }
        fn mhz(self) -> Hertz {
            Hertz(self * 1000000)
        }
        fn ms(self) -> MicroSeconds {
            MicroSeconds(self * 1000)
        }
        fn us(self) -> MicroSeconds {
            MicroSeconds(self)
        }
    }
    impl Into<MicroSeconds> for Hertz {
        fn into(self) -> MicroSeconds {
            let freq = self.0;
            if !(freq != 0 && freq <= 1000000) {
                {
                    ::core::panicking::panic(&(
                        "assertion failed: freq != 0 && freq <= 1000000",
                        "src/time.rs",
                        62u32,
                        9u32,
                    ))
                }
            };
            MicroSeconds(1000000 / freq)
        }
    }
    impl Into<Hertz> for MicroSeconds {
        fn into(self) -> Hertz {
            let period = self.0;
            if !(period != 0 && period <= 1000000) {
                {
                    ::core::panicking::panic(&(
                        "assertion failed: period != 0 && period <= 1000000",
                        "src/time.rs",
                        70u32,
                        9u32,
                    ))
                }
            };
            Hertz(1000000 / period)
        }
    }
}
pub mod timer {
    #![doc = " Timers"]
    use crate::hal::timer::{CountDown, Periodic};
    use crate::pac::{TIM2, TIM21, TIM22, TIM3};
    use crate::rcc::{Clocks, Rcc};
    use crate::time::Hertz;
    use cast::{u16, u32};
    use cortex_m::peripheral::syst::SystClkSource;
    use cortex_m::peripheral::SYST;
    use nb;
    use void::Void;
    pub trait TimerExt<TIM> {
        fn timer<T>(self, timeout: T, rcc: &mut Rcc) -> Timer<TIM>
        where
            T: Into<Hertz>;
    }
    #[doc = " Hardware timers"]
    pub struct Timer<TIM> {
        clocks: Clocks,
        tim: TIM,
    }
    impl Timer<SYST> {
        #[doc = " Configures the SYST clock as a periodic count down timer"]
        pub fn syst<T>(mut syst: SYST, timeout: T, rcc: &mut Rcc) -> Self
        where
            T: Into<Hertz>,
        {
            syst.set_clock_source(SystClkSource::Core);
            let mut timer = Timer {
                tim: syst,
                clocks: rcc.clocks,
            };
            timer.start(timeout);
            timer
        }
        #[doc = " Starts listening"]
        pub fn listen(&mut self) {
            self.tim.enable_interrupt()
        }
        #[doc = " Stops listening"]
        pub fn unlisten(&mut self) {
            self.tim.disable_interrupt()
        }
    }
    impl CountDown for Timer<SYST> {
        type Time = Hertz;
        fn start<T>(&mut self, timeout: T)
        where
            T: Into<Hertz>,
        {
            let rvr = self.clocks.sys_clk().0 / timeout.into().0 - 1;
            if !(rvr < (1 << 24)) {
                {
                    ::core::panicking::panic(&(
                        "assertion failed: rvr < (1 << 24)",
                        "src/timer.rs",
                        59u32,
                        9u32,
                    ))
                }
            };
            self.tim.set_reload(rvr);
            self.tim.clear_current();
            self.tim.enable_counter();
        }
        fn wait(&mut self) -> nb::Result<(), Void> {
            if self.tim.has_wrapped() {
                Ok(())
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }
    impl TimerExt<SYST> for SYST {
        fn timer<T>(self, timeout: T, rcc: &mut Rcc) -> Timer<SYST>
        where
            T: Into<Hertz>,
        {
            Timer::syst(self, timeout, rcc)
        }
    }
    impl Periodic for Timer<SYST> {}
    impl TimerExt<TIM2> for TIM2 {
        fn timer<T>(self, timeout: T, rcc: &mut Rcc) -> Timer<TIM2>
        where
            T: Into<Hertz>,
        {
            Timer::tim2(self, timeout, rcc)
        }
    }
    impl Timer<TIM2> {
        #[doc = " Configures a TIM peripheral as a periodic count down timer"]
        pub fn tim2<T>(tim: TIM2, timeout: T, rcc: &mut Rcc) -> Self
        where
            T: Into<Hertz>,
        {
            rcc.rb.apb1enr.modify(|_, w| w.tim2en().set_bit());
            rcc.rb.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
            rcc.rb.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());
            let mut timer = Timer {
                tim,
                clocks: rcc.clocks,
            };
            timer.start(timeout);
            timer
        }
        #[doc = " Starts listening"]
        pub fn listen(&mut self) {
            self.tim.dier.write(|w| w.uie().set_bit());
        }
        #[doc = " Stops listening"]
        pub fn unlisten(&mut self) {
            self.tim.dier.write(|w| w.uie().clear_bit());
        }
        #[doc = " Clears interrupt flag"]
        pub fn clear_irq(&mut self) {
            self.tim.sr.write(|w| w.uif().clear_bit());
        }
        #[doc = " Releases the TIM peripheral"]
        pub fn release(self) -> TIM2 {
            self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            self.tim
        }
    }
    impl CountDown for Timer<TIM2> {
        type Time = Hertz;
        fn start<T>(&mut self, timeout: T)
        where
            T: Into<Hertz>,
        {
            self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            self.tim.cnt.reset();
            let freq = timeout.into().0;
            let ticks = self.clocks.apb1_tim_clk().0 / freq;
            let psc = u16((ticks - 1) / (1 << 16)).unwrap();
            self.tim.psc.write(|w| unsafe { w.psc().bits(psc) });
            #[cfg(feature = "stm32l0x2")]
            self.tim
                .arr
                .write(|w| unsafe { w.arr().bits(u16(ticks / u32(psc + 1)).unwrap().into()) });
            self.tim.cr1.modify(|_, w| w.urs().set_bit());
            self.tim.cr1.modify(|_, w| w.cen().set_bit());
        }
        fn wait(&mut self) -> nb::Result<(), Void> {
            if self.tim.sr.read().uif().bit_is_clear() {
                Err(nb::Error::WouldBlock)
            } else {
                self.tim.sr.modify(|_, w| w.uif().clear_bit());
                Ok(())
            }
        }
    }
    impl Periodic for Timer<TIM2> {}
    impl TimerExt<TIM3> for TIM3 {
        fn timer<T>(self, timeout: T, rcc: &mut Rcc) -> Timer<TIM3>
        where
            T: Into<Hertz>,
        {
            Timer::tim3(self, timeout, rcc)
        }
    }
    impl Timer<TIM3> {
        #[doc = " Configures a TIM peripheral as a periodic count down timer"]
        pub fn tim3<T>(tim: TIM3, timeout: T, rcc: &mut Rcc) -> Self
        where
            T: Into<Hertz>,
        {
            rcc.rb.apb1enr.modify(|_, w| w.tim3en().set_bit());
            rcc.rb.apb1rstr.modify(|_, w| w.tim3rst().set_bit());
            rcc.rb.apb1rstr.modify(|_, w| w.tim3rst().clear_bit());
            let mut timer = Timer {
                tim,
                clocks: rcc.clocks,
            };
            timer.start(timeout);
            timer
        }
        #[doc = " Starts listening"]
        pub fn listen(&mut self) {
            self.tim.dier.write(|w| w.uie().set_bit());
        }
        #[doc = " Stops listening"]
        pub fn unlisten(&mut self) {
            self.tim.dier.write(|w| w.uie().clear_bit());
        }
        #[doc = " Clears interrupt flag"]
        pub fn clear_irq(&mut self) {
            self.tim.sr.write(|w| w.uif().clear_bit());
        }
        #[doc = " Releases the TIM peripheral"]
        pub fn release(self) -> TIM3 {
            self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            self.tim
        }
    }
    impl CountDown for Timer<TIM3> {
        type Time = Hertz;
        fn start<T>(&mut self, timeout: T)
        where
            T: Into<Hertz>,
        {
            self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            self.tim.cnt.reset();
            let freq = timeout.into().0;
            let ticks = self.clocks.apb1_tim_clk().0 / freq;
            let psc = u16((ticks - 1) / (1 << 16)).unwrap();
            self.tim.psc.write(|w| unsafe { w.psc().bits(psc) });
            #[cfg(feature = "stm32l0x2")]
            self.tim
                .arr
                .write(|w| unsafe { w.arr().bits(u16(ticks / u32(psc + 1)).unwrap().into()) });
            self.tim.cr1.modify(|_, w| w.urs().set_bit());
            self.tim.cr1.modify(|_, w| w.cen().set_bit());
        }
        fn wait(&mut self) -> nb::Result<(), Void> {
            if self.tim.sr.read().uif().bit_is_clear() {
                Err(nb::Error::WouldBlock)
            } else {
                self.tim.sr.modify(|_, w| w.uif().clear_bit());
                Ok(())
            }
        }
    }
    impl Periodic for Timer<TIM3> {}
    impl TimerExt<TIM21> for TIM21 {
        fn timer<T>(self, timeout: T, rcc: &mut Rcc) -> Timer<TIM21>
        where
            T: Into<Hertz>,
        {
            Timer::tim21(self, timeout, rcc)
        }
    }
    impl Timer<TIM21> {
        #[doc = " Configures a TIM peripheral as a periodic count down timer"]
        pub fn tim21<T>(tim: TIM21, timeout: T, rcc: &mut Rcc) -> Self
        where
            T: Into<Hertz>,
        {
            rcc.rb.apb2enr.modify(|_, w| w.tim21en().set_bit());
            rcc.rb.apb2rstr.modify(|_, w| w.tim21rst().set_bit());
            rcc.rb.apb2rstr.modify(|_, w| w.tim21rst().clear_bit());
            let mut timer = Timer {
                tim,
                clocks: rcc.clocks,
            };
            timer.start(timeout);
            timer
        }
        #[doc = " Starts listening"]
        pub fn listen(&mut self) {
            self.tim.dier.write(|w| w.uie().set_bit());
        }
        #[doc = " Stops listening"]
        pub fn unlisten(&mut self) {
            self.tim.dier.write(|w| w.uie().clear_bit());
        }
        #[doc = " Clears interrupt flag"]
        pub fn clear_irq(&mut self) {
            self.tim.sr.write(|w| w.uif().clear_bit());
        }
        #[doc = " Releases the TIM peripheral"]
        pub fn release(self) -> TIM21 {
            self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            self.tim
        }
    }
    impl CountDown for Timer<TIM21> {
        type Time = Hertz;
        fn start<T>(&mut self, timeout: T)
        where
            T: Into<Hertz>,
        {
            self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            self.tim.cnt.reset();
            let freq = timeout.into().0;
            let ticks = self.clocks.apb2_tim_clk().0 / freq;
            let psc = u16((ticks - 1) / (1 << 16)).unwrap();
            self.tim.psc.write(|w| unsafe { w.psc().bits(psc) });
            #[cfg(feature = "stm32l0x2")]
            self.tim
                .arr
                .write(|w| unsafe { w.arr().bits(u16(ticks / u32(psc + 1)).unwrap().into()) });
            self.tim.cr1.modify(|_, w| w.urs().set_bit());
            self.tim.cr1.modify(|_, w| w.cen().set_bit());
        }
        fn wait(&mut self) -> nb::Result<(), Void> {
            if self.tim.sr.read().uif().bit_is_clear() {
                Err(nb::Error::WouldBlock)
            } else {
                self.tim.sr.modify(|_, w| w.uif().clear_bit());
                Ok(())
            }
        }
    }
    impl Periodic for Timer<TIM21> {}
}
pub mod watchdog {
    use crate::hal::watchdog;
    use crate::pac::{IWDG, WWDG};
    use crate::rcc::Rcc;
    use crate::time::Hertz;
    pub struct IndependedWatchdog {
        iwdg: IWDG,
    }
    impl IndependedWatchdog {
        pub fn set_config(&mut self, pre: u8, reload: u16) {
            self.iwdg.kr.write(|w| w.key().reset());
            self.iwdg.kr.write(|w| w.key().enable());
            while self.iwdg.sr.read().pvu().bit() {}
            self.iwdg.pr.write(|w| w.pr().bits(pre));
            while self.iwdg.sr.read().rvu().bit() {}
            self.iwdg.rlr.write(|w| w.rl().bits(reload));
            self.iwdg.kr.write(|w| w.key().start());
            self.iwdg.kr.write(|w| w.key().reset());
        }
    }
    impl watchdog::Watchdog for IndependedWatchdog {
        fn feed(&mut self) {
            self.iwdg.kr.write(|w| w.key().reset());
        }
    }
    impl watchdog::WatchdogEnable for IndependedWatchdog {
        type Time = Hertz;
        fn start<T>(&mut self, period: T)
        where
            T: Into<Hertz>,
        {
            const LSI_CLOCK: u32 = 38000u32;
            let freq = period.into().0;
            let mut timeout = LSI_CLOCK / freq / 4;
            let mut pre = 0;
            let mut reload = 0;
            while pre < 7 {
                reload = timeout;
                if reload <= 4095 {
                    break;
                }
                pre += 1;
                timeout /= 2;
            }
            self.set_config(pre, reload as u16);
        }
    }
    pub trait IndependedWatchdogExt {
        fn watchdog(self) -> IndependedWatchdog;
    }
    impl IndependedWatchdogExt for IWDG {
        fn watchdog(self) -> IndependedWatchdog {
            IndependedWatchdog { iwdg: self }
        }
    }
    pub struct WindowWatchdog {
        wwdg: WWDG,
        clk: u32,
    }
    impl watchdog::Watchdog for WindowWatchdog {
        fn feed(&mut self) {
            self.wwdg.cr.write(|w| w.t().bits(255));
        }
    }
    impl WindowWatchdog {
        pub fn set_window<T>(&mut self, window: T)
        where
            T: Into<Hertz>,
        {
            let freq = window.into().0;
            let mut ticks = self.clk / freq;
            let mut pre = 0;
            let mut threshold = 0;
            while pre < 3 {
                threshold = ticks;
                if threshold <= 127 {
                    break;
                }
                pre += 1;
                ticks /= 2;
            }
            let window_bits = if threshold >= 255 {
                0
            } else {
                255 - threshold as u8
            };
            self.wwdg
                .cfr
                .write(|w| w.wdgtb().bits(pre).w().bits(window_bits));
        }
        pub fn listen(&mut self) {
            self.wwdg.cfr.write(|w| w.ewi().set_bit());
        }
    }
    impl watchdog::WatchdogEnable for WindowWatchdog {
        type Time = Hertz;
        fn start<T>(&mut self, period: T)
        where
            T: Into<Hertz>,
        {
            self.set_window(period);
            self.wwdg.cr.write(|w| w.wdga().set_bit().t().bits(255));
        }
    }
    pub trait WindowWatchdogExt {
        fn watchdog(self, rcc: &mut Rcc) -> WindowWatchdog;
    }
    impl WindowWatchdogExt for WWDG {
        fn watchdog(self, rcc: &mut Rcc) -> WindowWatchdog {
            rcc.rb.apb1enr.modify(|_, w| w.wwdgen().set_bit());
            WindowWatchdog {
                wwdg: self,
                clk: rcc.clocks.apb1_clk().0 / 4096,
            }
        }
    }
}
