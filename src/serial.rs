use core::fmt;
use core::marker::PhantomData;
use core::ptr;

use crate::gpio::gpioa::*;
use crate::gpio::AltMode;
use crate::hal;
use crate::hal::prelude::*;
pub use crate::pac::USART2;
use crate::rcc::Rcc;
use nb::block;

#[cfg(feature = "stm32l0x1")]
pub use crate::pac::LPUART1;

#[cfg(feature = "stm32l0x2")]
use core::{
    ops::{Deref, DerefMut},
    pin::Pin,
};

#[cfg(feature = "stm32l0x2")]
use as_slice::{AsMutSlice, AsSlice};

#[cfg(feature = "stm32l0x2")]
pub use crate::{
    dma,
    gpio::gpiob::{PB6, PB7},
    pac::USART1,
};

/// Serial error
#[derive(Debug)]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
    #[doc(hidden)]
    _Extensible,
}

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
    /// Idle line state detected
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

#[derive(Debug)]
pub struct InvalidConfig;

impl Default for Config {
    fn default() -> Config {
        let baudrate = 9_600_u32.bps();
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

macro_rules! impl_pins {
    ($($instance:ty, $tx:ident, $rx:ident, $alt:ident;)*) => {
        $(
            impl<Tx, Rx> Pins<$instance> for ($tx<Tx>, $rx<Rx>) {
                fn setup(&self) {
                    self.0.set_alt_mode(AltMode::$alt);
                    self.1.set_alt_mode(AltMode::$alt);
                }
            }
        )*
    }
}

#[cfg(feature = "stm32l0x1")]
impl_pins!(
    LPUART1, PA2, PA3,  AF6;
    USART2,  PA9, PA10, AF4;
);

#[cfg(feature = "stm32l0x2")]
impl_pins!(
    USART1, PA9,  PA10, AF4;
    USART1, PB6,  PB7,  AF0;
    USART2, PA2,  PA3,  AF4;
    USART2, PA14, PA15, AF4;
);

/// Serial abstraction
pub struct Serial<USART> {
    usart: USART,
    rx: Rx<USART>,
    tx: Tx<USART>,
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

macro_rules! usart {
    ($(
        $USARTX:ident: ($usartX:ident, $apbXenr:ident, $usartXen:ident, $pclkX:ident, $SerialExt:ident),
    )+) => {
        $(
            pub trait $SerialExt<PINS> {
                fn usart(self, pins: PINS, config: Config, rcc: &mut Rcc) -> Result<Serial<$USARTX>, InvalidConfig>;
            }

            impl<PINS> $SerialExt<PINS> for $USARTX
                where
                    PINS: Pins<$USARTX>,
            {
                fn usart(self, pins: PINS, config: Config, rcc: &mut Rcc) -> Result<Serial<$USARTX>, InvalidConfig> {
                    Serial::$usartX(self, pins, config, rcc)
                }
            }

            impl Serial<$USARTX> {
                pub fn $usartX<PINS>(
                    usart: $USARTX,
                    pins: PINS,
                    config: Config,
                    rcc: &mut Rcc,
                ) -> Result<Self, InvalidConfig>
                where
                    PINS: Pins<$USARTX>,
                {
                    pins.setup();

                    // Enable clock for USART
                    rcc.rb.$apbXenr.modify(|_, w| w.$usartXen().set_bit());

                    // Calculate correct baudrate divisor on the fly
                    let div = (rcc.clocks.$pclkX().0 * 25) / (4 * config.baudrate.0);
                    let mantissa = div / 100;
                    let fraction = ((div - mantissa * 100) * 16 + 50) / 100;
                    let mut brr = mantissa << 4 | fraction;

                    if stringify!($usartX) == "lpuart1" {
                        brr = brr*256
                    }

                    usart
                        .brr
                        .write(|w| unsafe { w.bits(brr) });

                    // Reset other registers to disable advanced USART features
                    usart.cr2.reset();

                    // Enable DMA
                    usart.cr3.write(|w|
                        w
                            // Stop DMA transfer on reception error
                            .ddre().disabled()
                            // Enable DMA
                            .dmat().enabled()
                            .dmar().enabled()
                    );

                    // Enable transmission and receiving
                    // and configure frame
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
                            }).pce()
                            .bit(match config.parity {
                                Parity::ParityNone => false,
                                _ => true,
                            }).ps()
                            .bit(match config.parity {
                                Parity::ParityOdd => true,
                                _ => false,
                            })
                    });

                    usart.cr2.write(|w|
                        w.stop().bits(match config.stopbits {
                            StopBits::STOP1 => 0b00,
                            StopBits::STOP0P5 => 0b01,
                            StopBits::STOP2 => 0b10,
                            StopBits::STOP1P5 => 0b11,
                        })
                    );
                    Ok(Serial {
                        usart,
                        tx: Tx { _usart: PhantomData },
                        rx: Rx { _usart: PhantomData },
                    })
                }

                /// Starts listening for an interrupt event
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Rxne => {
                            self.usart.cr1.modify(|_, w| w.rxneie().set_bit())
                        },
                        Event::Txe => {
                            self.usart.cr1.modify(|_, w| w.txeie().set_bit())
                        },
                        Event::Idle => {
                            self.usart.cr1.modify(|_, w| w.idleie().set_bit())
                        },
                    }
                }

                /// Stop listening for an interrupt event
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Rxne => {
                            self.usart.cr1.modify(|_, w| w.rxneie().clear_bit())
                        },
                        Event::Txe => {
                            self.usart.cr1.modify(|_, w| w.txeie().clear_bit())
                        },
                        Event::Idle => {
                            self.usart.cr1.modify(|_, w| w.idleie().clear_bit())
                        },
                    }
                }

                /// Clears interrupt flag
                pub fn clear_irq(&mut self, event: Event) {
                    if let Event::Rxne = event {
                        self.usart.rqr.write(|w| w.rxfrq().discard())
                    }
                }

                pub fn split(self) -> (Tx<$USARTX>, Rx<$USARTX>) {
                    (self.tx, self.rx)
                }

                pub fn release(self) -> $USARTX {
                    self.usart
                }
            }

            impl hal::serial::Read<u8> for Serial<$USARTX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    self.rx.read()
                }
            }

             impl hal::serial::Write<u8> for  Serial<$USARTX> {
                type Error = Error;

                fn flush(&mut self) -> nb::Result<(), Self::Error> {
                    self.tx.flush()
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                    self.tx.write(byte)
                }
            }

            #[cfg(feature = "stm32l0x2")]
            impl Rx<$USARTX> {
                pub fn read_all<Buffer, Channel>(self,
                    dma:     &mut dma::Handle,
                    buffer:  Pin<Buffer>,
                    channel: Channel,
                )
                    -> dma::Transfer<Self, Channel, Buffer, dma::Ready>
                    where
                        Self:           dma::Target<Channel>,
                        Buffer:         DerefMut + 'static,
                        Buffer::Target: AsMutSlice<Element=u8>,
                        Channel:        dma::Channel,
                {
                    // Safe, because we're only taking the address of a
                    // register.
                    let address =
                        &unsafe { &*$USARTX::ptr() }.rdr as *const _ as u32;

                    // Safe, because the trait bounds of this method guarantee
                    // that the buffer can be written to.
                    unsafe {
                        dma::Transfer::new(
                            dma,
                            self,
                            channel,
                            buffer,
                            address,
                            dma::Priority::high(),
                            dma::Direction::peripheral_to_memory(),
                        )
                    }
                }
            }

            impl hal::serial::Read<u8> for Rx<$USARTX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    // Check for any errors
                    let _err = if isr.pe().bit_is_set() {
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

                    if let Some(_err) = _err {
                        // Some error occured. Clear the error flags by writing to ICR
                        // followed by a read from the rdr register
                        // NOTE(read_volatile) see `write_volatile` below
                        unsafe {
                            (*$USARTX::ptr()).icr.write(|w| {w.pecf().set_bit()});
                            (*$USARTX::ptr()).icr.write(|w| {w.fecf().set_bit()});
                            (*$USARTX::ptr()).icr.write(|w| {w.ncf().set_bit()});
                            (*$USARTX::ptr()).icr.write(|w| {w.orecf().set_bit()});
                        }
                        Err(nb::Error::WouldBlock)
                    } else {
                        // Check if a byte is available
                        if isr.rxne().bit_is_set() {
                            // Read the received byte
                            // NOTE(read_volatile) see `write_volatile` below
                            Ok(unsafe {
                                ptr::read_volatile(&(*$USARTX::ptr()).rdr as *const _ as *const _)
                            })
                        } else {
                            Err(nb::Error::WouldBlock)
                        }
                    }
                }
            }

            impl hal::serial::Write<u8> for Tx<$USARTX> {
                type Error = Error;

                fn flush(&mut self) -> nb::Result<(), Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    // Frame complete, set the TC Clear Flag
                    unsafe {
                        (*$USARTX::ptr()).icr.write(|w| {w.tccf().set_bit()});
                    }

                    // Check TC bit on ISR
                    if isr.tc().bit_is_set() {
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    if isr.txe().bit_is_set() {
                        // NOTE(unsafe) atomic write to stateless register
                        // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
                        unsafe { ptr::write_volatile(&(*$USARTX::ptr()).tdr as *const _ as *mut _, byte) }

                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }
            }

            #[cfg(feature = "stm32l0x2")]
            impl Tx<$USARTX> {
                pub fn write_all<Buffer, Channel>(self,
                    dma:     &mut dma::Handle,
                    buffer:  Pin<Buffer>,
                    channel: Channel,
                )
                    -> dma::Transfer<Self, Channel, Buffer, dma::Ready>
                    where
                        Self:           dma::Target<Channel>,
                        Buffer:         Deref + 'static,
                        Buffer::Target: AsSlice<Element=u8>,
                        Channel:        dma::Channel,
                {
                    // Safe, because we're only taking the address of a
                    // register.
                    let address =
                        &unsafe { &*$USARTX::ptr() }.tdr as *const _ as u32;

                    // Safe, because the trait bounds of this method guarantee
                    // that the buffer can be read from.
                    unsafe {
                        dma::Transfer::new(
                            dma,
                            self,
                            channel,
                            buffer,
                            address,
                            dma::Priority::high(),
                            dma::Direction::memory_to_peripheral(),
                        )
                    }
                }
            }
        )+
    }
}

#[cfg(feature = "stm32l0x1")]
usart! {
    LPUART1: (lpuart1, apb1enr, lpuart1en, apb1_clk, Serial1Ext),
    USART2: (usart2, apb1enr, usart2en, apb1_clk, Serial2Ext),
}

#[cfg(feature = "stm32l0x2")]
usart! {
    USART1: (usart1, apb2enr, usart1en, apb1_clk, Serial1Ext),
    USART2: (usart2, apb1enr, usart2en, apb1_clk, Serial2Ext),
}

impl<USART> fmt::Write for Serial<USART>
where
    Serial<USART>: hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = s
            .as_bytes()
            .into_iter()
            .map(|c| block!(self.write(*c)))
            .last();

        //self.flush().map_err(|_| fmt::Error)?;

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
            .map(|c| block!(self.write(*c)))
            .last();

        //self.flush().map_err(|_| fmt::Error)?;

        Ok(())
    }
}
