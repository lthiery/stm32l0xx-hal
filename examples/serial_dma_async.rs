#![no_main]
#![no_std]


extern crate panic_halt;
use cortex_m::interrupt::Mutex;

use core::{
    cell::RefCell,
    ops::DerefMut,
    pin::Pin,
};

use cortex_m_rt::entry;
use replace_with::replace_with;
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
    RECEIVING(RxTransfer),
    RECEIVED(RxTransfer),
    SENDING(TxTransfer),
    SENT(TxTransfer),
}

type RxTransfer = dma::Transfer<
    serial::Rx<serial::USART2>,
    dma::Channel5,
    &'static mut [u8; 1],
    dma::Started,
>;

// Placeholder. Currently no sending is done in this example, so I didn't see
// the need to figure out the correct type for the transmission channel right
// now.
type TxTransfer = ();


static mut BUFFER: [u8; 1] = [0;1];
static STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(State::READY));


#[entry]
fn main() -> ! {
    let mut cp = pac::CorePeripherals::take().unwrap();
    let     dp = pac::Peripherals::take().unwrap();

    let mut rcc   = dp.RCC.freeze(Config::hsi16());
    let mut dma   = DMA::new(dp.DMA1, &mut rcc);
    let     gpioa = dp.GPIOA.split(&mut rcc);

    // let mut rx_channel = 

    let (_tx, rx) = dp
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

    loop {
        cortex_m::interrupt::free(|cs| {
            let mut state = STATE.borrow(cs).borrow_mut();

            // This function allows us to move out of `state` which is of type
            // `&mut State`, then replace it with a different state once we're
            // done. If we tried to do this on our own, we'd run into all kinds
            // of trouble with the compiler. `replace_with` solves this problem
            // nicely, with a bit of `unsafe` under the hood.
            replace_with(
                state.deref_mut(),
                || {
                    // While we've moved a value out of `state`, the memory
                    // `state` points to is uninitialized. This usually isn't
                    // going to be a problem, because we're moving something
                    // back there once we're done.
                    //
                    // However, if we panic while `state` is in this invalid
                    // state, this closure is called. We could provide some
                    // default value here to assign back to `state` and recover
                    // from the panic, but I don't think this is feasible here.
                    // Let's just give up and panic.
                    panic!("Panic while working with transitioning state");
                },
                |state| {
                    // This closure is the important part of our `replace_with`
                    // call. We have an owned `state` here, and can do with it
                    // what we want, as long as we return a new state from the
                    // closure.
                    match state {
                        State::READY => {
                            if let Some(uart_rx) = rx_stash.take() {
                                // Prepare read transfer
                                let pin_buffer = Pin::new(
                                    unsafe { &mut BUFFER}
                                );
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

                                rx_stash = None;
                                State::RECEIVING(transfer.start())
                            } else {
                                panic!("Rx was not put back!");
                            }
                        }
                        State::RECEIVED(transfer) => {
                            let res = transfer.wait().unwrap();
                            rx_stash = Some(res.target);
                            dma.channels.channel5 = res.channel;

                            State::READY
                        }
                        state => {
                            // Currently we don't handle any other states. Let's
                            // just return the current state.
                            state
                        }

                        // State::SENT(_) => {

                        // }
                        // State::RECEIVING(_) | State::SENDING(_) => {},
                    }
                }
            );
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
         let mut state = STATE.borrow(cs).borrow_mut();

         // See other use of `replace_with` above for explanation.
         replace_with(
            state.deref_mut(),
            || panic!("Panic during state transition"),
            |state| {
                match state {
                    State::READY | State::RECEIVED(_) | State::SENT(_)  =>
                        panic!("Should not interrupt in this state!"),
                    State::RECEIVING(transfer) => State::RECEIVED(transfer),
                    State::SENDING(transfer)   => State::SENT(transfer),
                 }
            },
        );
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