#![no_main]
#![no_std]

extern crate panic_halt;

use core::pin::Pin;
use core::fmt::Write;

use cortex_m::{asm, interrupt};
use cortex_m_rt::entry;
use stm32l0xx_hal::{
    dma::{self, DMA},
    pac::{self, Interrupt},
    prelude::*,
    rcc::Config,
    serial,
};

#[entry]
fn main() -> ! {
    let mut cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(Config::hsi16());
    let mut dma = DMA::new(dp.DMA1, &mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);

    let mut rx_channel = dma.channels.channel5;

    let (mut tx, mut rx) = dp
        .USART2
        .usart(
            (gpioa.pa2, gpioa.pa3),
            serial::Config::default().baudrate(115_200.bps()),
            &mut rcc,
        )
        .unwrap()
        .split();

    // Create the buffer we're going to use for DMA.
    // This is safe, since this is the main function, and it's only executed
    // once. This means there is no other code accessing this `static`.
    static mut BUFFER: [u8; 1] = [0; 1];
    let mut buffer = Some(Pin::new(unsafe { &mut BUFFER }));
    let mut maybe_rx = Some(rx);
    let mut maybe_channel = Some(rx_channel);

    let mut transfer = None;
    loop {
        // Prepare read transfer
        if let Some(buf) = buffer {
            if let Some(rx) = maybe_rx{
                if let Some(rx_channel) = maybe_channel {
                    let mut xfer = rx.read_all(&mut dma.handle, buf, rx_channel);
                    xfer.enable_interrupts(dma::Interrupts {
                        transfer_error: true,
                        transfer_complete: true,
                        ..dma::Interrupts::default()
                    });

                    transfer = Some(xfer.start());
                    maybe_channel = None;
                }
                maybe_rx = None;
            }
            buffer = None;
        }

        transfer = match transfer {
            None => None,
            Some(xfer) => {
                let res = xfer.wait().unwrap();

                // Re-assign reception resources to their variables, so they're
                // available again in the next loop iteration.
                maybe_rx = Some(res.target);
                maybe_channel = Some(res.channel);
                write!(tx, "{}", res.buffer.as_ref()[0] as char);
                buffer = Some(res.buffer);
                None
            }
        }
       

        // // Prepare write transfer
        // let mut transfer = tx.write_all(&mut dma.handle, buffer, tx_channel);

        // // Start DMA transfer and wait for it to finish
        // let res = interrupt::free(|_| {
        //     cp.NVIC.enable(Interrupt::DMA1_CHANNEL4_7);

        //     transfer.enable_interrupts(dma::Interrupts {
        //         transfer_error: true,
        //         transfer_complete: true,
        //         ..dma::Interrupts::default()
        //     });

        //     let transfer = transfer.start();

        //     asm::wfi();
        //     let res = transfer.wait().unwrap();
        //     cp.NVIC.disable(Interrupt::DMA1_CHANNEL4_7);

        //     res
        // });

        // // Re-assign transmission resources to their variables, so they're
        // // available again in the next loop iteration.
        // tx = res.target;
        // tx_channel = res.channel;
        // buffer = res.buffer;
    }
}
