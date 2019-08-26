pub use embedded_hal::{
    adc::OneShot as _,
    digital::v2::*,
    prelude::*,
    watchdog::{Watchdog as _, WatchdogEnable as _},
};

pub use crate::{
    adc::AdcExt as _,
    delay::DelayExt as _,
    exti::ExtiExt as _,
    gpio::GpioExt as _,
    i2c::I2cExt as _,
    pwr::PowerMode as _,
    rcc::RccExt as _,
    serial::{Serial1Ext as _, Serial2Ext as _},
    spi::SpiExt as _,
    time::U32Ext as _,
    timer::TimerExt as _,
    watchdog::{IndependedWatchdogExt as _, WindowWatchdogExt as _},
};
