//! Simple "logic analyzer" for debugging
use cortex_m::singleton;

use crate::bsp::hal::{dma, pio};

#[allow(dead_code)]
pub(crate) fn capture_trace<P, SMI, CH>(
    pio: &mut pio::PIO<P>,
    sm: pio::UninitStateMachine<(P, SMI)>,
    dma_chan: CH,
) where
    P: pio::PIOExt,
    SMI: pio::StateMachineIndex,
    CH: dma::SingleChannel,
{
    let pin_cs = 4;

    let mut a = ::pio::Assembler::<{ ::pio::RP2040_MAX_PROGRAM_SIZE }>::new();
    let mut wrap_source = a.label();
    let mut wrap_target = a.label();
    {
        use ::pio::*;
        a.wait(1, WaitSource::GPIO, pin_cs, false);
        a.wait(0, WaitSource::GPIO, pin_cs, false);
        a.bind(&mut wrap_target);
        a.r#in(InSource::PINS, 8);
        a.bind(&mut wrap_source);
    }
    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    let installed = pio.install(&program).unwrap();
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
    defmt::debug!("{}", buf);
}
