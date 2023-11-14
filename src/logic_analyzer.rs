//! Simple "logic analyzer" for debugging
use cortex_m::singleton;
use defmt::*;

use crate::bsp::hal::{dma, pio};

pub(crate) fn capture_trace<P, SMI, CH>(
    pio: &mut pio::PIO<P>,
    sm: pio::UninitStateMachine<(P, SMI)>,
    dma_chan: CH,
) where
    P: pio::PIOExt,
    SMI: pio::StateMachineIndex,
    CH: dma::SingleChannel,
{
    let program = pio_proc::pio_asm!(
        "wait 1 gpio 4",
        "wait 0 gpio 4",
        ".wrap_target",
        "in pins, 8",
        ".wrap",
    );

    let installed = pio.install(&program.program).unwrap();
    let (sm, rx, _) = pio::PIOBuilder::from_program(installed)
        // Hardware pinout:
        // 3: DO
        // 4: CS
        // 5: CLK
        // 6: DI
        .in_pin_base(3)
        .autopull(true)
        .autopush(true)
        .build(sm);
    sm.start();

    let buf = singleton!(: [u32; 1024 / 4] = [0; 1024 / 4]).unwrap();
    let mut config = dma::single_buffer::Config::new(dma_chan, rx, buf);
    config.bswap(true);
    let transfer = config.start();
    let (_, _, buf) = transfer.wait();
    let (_, buf, _) = unsafe { buf.align_to::<u8>() };
    debug!("{}", buf);
}
