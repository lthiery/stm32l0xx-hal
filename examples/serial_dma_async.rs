#![no_main]
#![no_std]

extern crate panic_halt;

use core::pin::Pin;

use cortex_m_rt::entry;
use stm32l0xx_hal::{
    dma::{self, DMA},
    pac::{
        self,
    },
    prelude::*,
    rcc::Config,
    serial,
};

type RxTarget = serial::Rx<serial::USART2>;
type RxChannel = dma::Channel5;
type DmaBuffer = &'static mut [u8; 1];
type RxTransfer = dma::Transfer<RxTarget, RxChannel, DmaBuffer, dma::Started>;

enum RxState {
    READY(RxTarget, RxChannel),
    RECEIVING(RxTransfer),
}

type TxTarget = serial::Tx<serial::USART2>;
type TxChannel = dma::Channel4;
type TxTransfer = dma::Transfer<TxTarget, TxChannel, DmaBuffer, dma::Started>;

enum TxState {
    READY(TxTarget, TxChannel),
    TRANSMITTING(TxTransfer),
}

use heapless::consts::*;
use heapless::spsc::Queue;

static mut BUFFER_1: [u8; 1] = [0; 1];
static mut BUFFER_2: [u8; 1] = [0; 1];


#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(Config::hsi16());
    let mut dma = DMA::new(dp.DMA1, &mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);

    let mut rx_buffers: Queue<Pin<DmaBuffer>, U4> = Queue::new();
    // putting the same pointer in here twice would be a big mistake
    unsafe {
        rx_buffers.enqueue(Pin::new(&mut BUFFER_1)).unwrap();;
        rx_buffers.enqueue(Pin::new(&mut BUFFER_2)).unwrap();;
    }

    // since we echo out received buffers from DMA, they are already pinned
    let mut tx_buffers: Queue<Pin<DmaBuffer>, U4> = Queue::new();

    let (tx, rx) = dp
        .USART2
        .usart(
            (gpioa.pa2, gpioa.pa3),
            serial::Config::default().baudrate(115_200.bps()),
            &mut rcc,
        )
        .unwrap()
        .split();

    let dma_handle = &mut dma.handle;
    let rx_channel = dma.channels.channel5;
    let tx_channel = dma.channels.channel4;

    let mut rx_state = RxState::READY(rx, rx_channel);
    let mut tx_state = TxState::READY(tx, tx_channel);

    loop {
        rx_state = match rx_state {
            RxState::READY(rx, channel) => {
                if let Some(buffer) = rx_buffers.dequeue() {
                    // prepare transfer transaction
                    let mut transfer = rx.read_all(dma_handle, buffer, channel);

                    transfer.enable_interrupts(dma::Interrupts {
                        transfer_error: true,
                        transfer_complete: true,
                        ..dma::Interrupts::default()
                    });

                    RxState::RECEIVING(transfer.start())
                } else {
                    panic!("Not enough buffers allocated");
                }
            }
            RxState::RECEIVING(transfer) => {
                if !transfer.is_active(){
                    let res = transfer.wait().unwrap();
                    tx_buffers.enqueue(res.buffer).unwrap();
                    RxState::READY(res.target, res.channel)
                } else {
                    RxState::RECEIVING(transfer)
                }
            }
        };

        tx_state = match tx_state {
            TxState::READY(tx, channel) => {
                if let Some(buffer) = tx_buffers.dequeue() {
                    let mut transfer = tx.write_all(dma_handle, buffer, channel);

                    transfer.enable_interrupts(dma::Interrupts {
                        transfer_error: true,
                        transfer_complete: true,
                        ..dma::Interrupts::default()
                    });
                    TxState::TRANSMITTING(transfer.start())
                } else {
                    TxState::READY(tx, channel)
                }
            }
            TxState::TRANSMITTING(transfer) => {
                if !transfer.is_active(){
                    let res = transfer.wait().unwrap();
                    // give the buffer back to the rx_buffer queue
                    rx_buffers.enqueue(res.buffer).unwrap();
                    TxState::READY(res.target, res.channel)
                } else {
                    TxState::TRANSMITTING(transfer)
                }
            }
        };
    }
    
}
