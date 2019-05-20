//! External interrupt controller
use crate::bb;
use crate::pac::EXTI;

pub enum Port {
    A,
    B,
    C
}


pub enum TriggerEdge {
    Rising,
    Falling,
    All,
}

pub trait ExtiExt {
    fn listen(&self, port: Port, line: u8, edge: TriggerEdge);
    fn unlisten(&self, line: u8);
    fn pend_interrupt(&self, line: u8);
    fn clear_irq(&self, line: u8);
}

impl ExtiExt for EXTI {
    fn listen(&self, port: Port, line: u8, edge: TriggerEdge) {
        #[cfg(feature = "stm32l0x1")]
        assert!(line < 24);
        #[cfg(feature = "stm32l0x2")]
        assert!(line < 17);

        let bm: u32 = 0xFFFF;//0b1<<line;


        unsafe {
            match edge {
                TriggerEdge::Rising => self.rtsr.write(|mut w|
                    w.bits(bm)
                ),
                TriggerEdge::Falling => self.ftsr.write(|mut w|
                    w.bits(bm)
                ),
                TriggerEdge::All => {
                    self.rtsr.write(| mut w|
                        w.bits(bm)
                    );
                    self.ftsr.write(|mut w|
                        w.bits(bm)
                    );
                    }
            }

            self.imr.modify(|_, mut w|
                w.bits(bm)
            );
        }
       

    }

    fn unlisten(&self, line: u8) {
        assert!(line < 24);
        bb::clear(&self.rtsr, line);
        bb::clear(&self.ftsr, line);
        bb::clear(&self.imr, line);
    }

    fn pend_interrupt(&self, line: u8) {
        assert!(line < 24);
        bb::set(&self.swier, line);
    }

    fn clear_irq(&self, line: u8) {
        assert!(line < 24);

        self.pr.write(|mut w|
            unsafe{
                w.bits(0b1<<line)
            }
        );
    }
}
