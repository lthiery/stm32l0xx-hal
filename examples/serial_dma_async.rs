#![no_main]
#![no_std]


extern crate panic_halt;
use cortex_m::interrupt::Mutex;

use core::pin::Pin;

use cortex_m_rt::entry;
use stm32l0xx_hal::{
    prelude::*,
    dma::{
        self,
        DMA,
    },
    pac::{
        self,
        Interrupt,
    },
    rcc::Config,
    serial,
};

enum State {
    READY,
    RECEIVING,
    RECEIVED,
    SENDING,
    SENT
}

static mut BUFFER: [u8; 1] = [0;1];
static STATE: Mutex<State> = Mutex::new(State::READY);


#[entry]
fn main() -> ! {
    let mut cp = pac::CorePeripherals::take().unwrap();
    let     dp = pac::Peripherals::take().unwrap();

    let mut rcc   = dp.RCC.freeze(Config::hsi16());
    let mut dma   = DMA::new(dp.DMA1, &mut rcc);
    let     gpioa = dp.GPIOA.split(&mut rcc);

    // let mut rx_channel = 

    let (mut tx, mut rx) = dp
        .USART2
        .usart(
            (gpioa.pa2, gpioa.pa3),
            serial::Config::default().baudrate(115_200.bps()),
            &mut rcc,
        )
        .unwrap()
        .split();

    //let maybe_tx = Some(tx);
    let mut rx_stash = Some(rx);
    //let mut dma_stash = Some(dma);
    let mut transfer_stash = None;

    loop {
        cortex_m::interrupt::free(|cs| {
            let mut state = *STATE.borrow(cs).borrow_mut();
            match state {
                State::READY => {
                    if let Some(uart_rx) = rx_stash.take() {
                
                            // Prepare read transfer
                            let mut pin_buffer = Pin::new(unsafe { &mut BUFFER});
                            let mut transfer = uart_rx.read_all(
                                &mut dma.handle,
                                pin_buffer,
                                dma.channels.channel5,
                            );

                            cp.NVIC.enable(Interrupt::DMA1_CHANNEL4_7);

                            transfer.enable_interrupts(dma::Interrupts {
                                transfer_error:    true,
                                transfer_complete: true,
                                .. dma::Interrupts::default()
                            });
                           
                            transfer_stash = Some(transfer.start());
                            state = &State::RECEIVING;
                            rx_stash = None;
                        
                    } else {
                        panic!("Rx was not put back!");
                    }

                }
                State::RECEIVED => {
                    if let Some(transfer) = transfer_stash {
                        let res = transfer.wait().unwrap();
                        rx_stash = Some(res.target);
                        dma.channels.channel5 = res.channel;
                        //dma_stash = 
        
                    }
                    // transfer_stash = None;

                }
                State::SENT => {
                    
                }
                State::RECEIVING | State::SENDING => {},

            }
        });


        // let res = transfer.wait().unwrap();
        // cp.NVIC.disable(Interrupt::DMA1_CHANNEL4_7);

    
            


        // // Re-assign reception resources to their variables, so they're
        // // available again in the next loop iteration.
        // rx         = res.target;
        // rx_channel = res.channel;
        // buffer     = res.buffer;

        // // Prepare write transfer
        // let mut transfer = tx.write_all(
        //     &mut dma.handle,
        //     buffer,
        //     tx_channel,
        // );

        // // Start DMA transfer and wait for it to finish
        // let res = interrupt::free(|_| {
        //     cp.NVIC.enable(Interrupt::DMA1_CHANNEL4_7);

        //     transfer.enable_interrupts(dma::Interrupts {
        //         transfer_error:    true,
        //         transfer_complete: true,
        //         .. dma::Interrupts::default()
        //     });

        //     let transfer = transfer.start();

        //     asm::wfi();
        //     let res = transfer.wait().unwrap();
        //     cp.NVIC.disable(Interrupt::DMA1_CHANNEL4_7);

        //     res
        // });

        // // Re-assign transmission resources to their variables, so they're
        // // available again in the next loop iteration.
        // tx         = res.target;
        // tx_channel = res.channel;
        // buffer     = res.buffer;
    }
}

fn DMA1_CHANNEL4_7() {
    cortex_m::interrupt::free(|cs| {
         let mut state = *STATE.borrow(cs).borrow_mut();
         match state {
            State::READY | State::RECEIVED | State::SENT  =>
                panic!("Should not interrupt in this state!"),
            State::RECEIVING => state = &State::RECEIVED,
            State::SENDING => state = &State::RECEIVED,
         }
    });


   
    // // Start DMA transfer and wait for it to finish
    // let res = interrupt::free(|_| {
    //     cp.NVIC.enable(Interrupt::DMA1_CHANNEL4_7);

    //     transfer.enable_interrupts(dma::Interrupts {
    //         transfer_error:    true,
    //         transfer_complete: true,
    //         .. dma::Interrupts::default()
    //     });

    //     let transfer = transfer.start();

    //     asm::wfi();
    //     let res = transfer.wait().unwrap();
    //     cp.NVIC.disable(Interrupt::DMA1_CHANNEL4_7);

    //     res
    // });
}