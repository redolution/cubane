//! # Flipper External Interface slave
//!
//! This module implements an EXI slave in PIO.
//! EXI is basically SPI mode 0 (CPHA=0, CPOL=0):
//! - CS is active-low;
//! - the clock idles low;
//! - the first bit is shifted out before the first clock pulse;
//! - data is latched on the rising edge of the clock on both ends;[^sample_edge]
//! - data is clocked out MSB-first, bytes in big-endian order;
//! - maximum data rate of 27Mbaud.
//!
//! [^sample_edge]: The Flipper datasheet actually contradicts itself here.  
//! On page 9:
//! "EXI Data In 0 Channel 0: EXI0DI0 is an input signal.
//! EXI0DI0 receives the serial data from the slave device,
//! the MSB is received first.
//! The data is latched on the *falling edge* of the EXI0CLK0."
//! **This is a lie.** Or most likely a mistake. Whatever.  
//! The timing diagram on page 20 is correct:
//! Flipper latches input data on the rising edge.
//! This was verified empirically by capturing a trace of a read from RTC-DOL.
//! RTC-DOL shifts bits out shortly after the rising edge of the bus clock.
//! The data only decodes correctly when setting the protocol decoder to SPI mode 0.
//!
//! We have to use PIO, because the hardware SPI block in the RP2040 is too slow:
//! section *4.4.3.4. Clock ratios* of the datasheet
//! says *SYSCLK* needs to be at least 12 times faster than the bus clock, which would be 324MHz;
//! RP2040 overclocks very well, but that is too high for comfort.
//!
//! Lastly, this module requires overclocking the microcontroller to meet DI (MISO) timings.  
//! The simplest PIO program to detect an edge on an input and toggle a GPIO takes 4-5 cycles to
//! react.
//! At the default setting of 125MHz, the *SYSCLK* to bus clock ratio is ~4.6.
//! This means that it's a coin flip whether we'll make it before the next rising edge.  
//! Instead, we target a *SYSCLK* rate of 200MHz, resulting in a clock ratio of ~7.4.
//! In the worst case, this leaves about 2 cycles (10ns) of headroom,
//! which exceeds the 8.93ns setup time specified by the Flipper datasheet.

use core::{marker, task};

use futures_util::future;

use crate::bsp::hal::{dma, gpio, pio};

/// Simple container type for a PIO state machine and its FIFOs
struct PIOStateMachine<P: pio::PIOExt, SMI: pio::StateMachineIndex> {
    sm: pio::StateMachine<(P, SMI), pio::Running>,
    rx: pio::Rx<(P, SMI)>,
    tx: pio::Tx<(P, SMI)>,
}

/// The actual EXI implementation
pub(crate) struct EXI<
    P: pio::PIOExt,
    CS: pio::StateMachineIndex,
    RX: pio::StateMachineIndex,
    TX: pio::StateMachineIndex,
    CHI: dma::ChannelIndex,
    PioIRQ: pio::IRQIndex,
    const CS_IRQ: u8,
> {
    /// The chip-select monitor
    ///
    /// Watches for CS changes, triggers interrupts and disables the DI output.
    sm_cs: PIOStateMachine<P, CS>,

    /// The data receive state machine
    sm_rx: PIOStateMachine<P, RX>,

    /// The data transmit state machine
    sm_tx: PIOStateMachine<P, TX>,

    /// A single DMA channel ought to be enough for anybody
    dma: Option<dma::Channel<CHI>>,

    _pio_irq: marker::PhantomData<PioIRQ>,
}

/// ZST to encapsulate the ISR implementation
pub(crate) struct EXIIntHandler<P: pio::PIOExt, PioIRQ: pio::IRQIndex, const CS_IRQ: u8> {
    _pio: marker::PhantomData<P>,
    _pio_irq: marker::PhantomData<PioIRQ>,
}

impl<P, CS, RX, TX, CHI, PioIRQ: pio::IRQIndex, const CS_IRQ: u8>
    EXI<P, CS, RX, TX, CHI, PioIRQ, CS_IRQ>
where
    P: pio::PIOExt,
    CS: pio::StateMachineIndex,
    RX: pio::StateMachineIndex,
    TX: pio::StateMachineIndex,
    CHI: dma::ChannelIndex,
{
    /// Initialize EXI with all the resources it requires
    pub(crate) fn new<PinCs, PinClk, PinDo, PinDi>(
        pio: &mut pio::PIO<P>,
        sm_cs: pio::UninitStateMachine<(P, CS)>,
        sm_rx: pio::UninitStateMachine<(P, RX)>,
        sm_tx: pio::UninitStateMachine<(P, TX)>,

        pin_cs: PinCs,
        pin_clk: PinClk,
        pin_do: PinDo,
        pin_di: PinDi,

        dma: dma::Channel<CHI>,
    ) -> (Self, EXIIntHandler<P, PioIRQ, CS_IRQ>)
    where
        PinCs: gpio::AnyPin,
        PinCs::Id: gpio::ValidFunction<P::PinFunction>,
        PinClk: gpio::AnyPin,
        PinClk::Id: gpio::ValidFunction<P::PinFunction>,
        PinDo: gpio::AnyPin,
        PinDo::Id: gpio::ValidFunction<P::PinFunction>,
        PinDi: gpio::AnyPin,
        PinDi::Id: gpio::ValidFunction<P::PinFunction>,
    {
        let pin_cs = pin_cs
            .into()
            .into_function()
            .into_pull_type::<gpio::PullUp>();
        let pin_clk = pin_clk.into().into_function();
        let pin_do = pin_do.into().into_function();
        let mut pin_di = pin_di.into().into_function();
        // Required to outdrive RTC-DOL
        pin_di.set_drive_strength(gpio::OutputDriveStrength::EightMilliAmps);

        let sm_cs = init_sm_cs(pio, sm_cs, CS_IRQ, &pin_cs, &pin_di);
        let sm_rx = init_sm_rx(pio, sm_rx, &pin_cs, &pin_clk, &pin_do);
        let sm_tx = init_sm_tx(pio, sm_tx, &pin_clk, &pin_di);

        (
            Self {
                sm_cs,
                sm_rx,
                sm_tx,
                dma: Some(dma),
                _pio_irq: marker::PhantomData {},
            },
            EXIIntHandler {
                _pio: marker::PhantomData {},
                _pio_irq: marker::PhantomData {},
            },
        )
    }

    /// Wait for the next rising edge on CS
    pub(crate) async fn transaction_end<PIO, W>(&self, pio: &mut PIO, cs_waker: &mut W)
    where
        PIO: rtic::Mutex<T = pio::PIO<P>>,
        W: rtic::Mutex<T = Option<task::Waker>>,
    {
        pio.lock(|pio| {
            pio.clear_irq(1 << CS_IRQ);
        });
        future::poll_fn(|poll_ctx| {
            pio.lock(|pio| {
                if pio.get_irq_raw() & (1 << CS_IRQ) != 0 {
                    pio.clear_irq(1 << CS_IRQ);
                    return core::task::Poll::Ready(());
                }

                cs_waker.lock(|w| {
                    w.replace(poll_ctx.waker().clone());
                });
                pio.irq::<PioIRQ>().enable_sm_interrupt(CS_IRQ);
                core::task::Poll::Pending
            })
        })
        .await;
    }
}

impl<P, PioIRQ: pio::IRQIndex, const CS_IRQ: u8> EXIIntHandler<P, PioIRQ, CS_IRQ>
where
    P: pio::PIOExt,
{
    /// The real ISR implementation
    pub(crate) fn on_pio_interrupt(&self, pio: &pio::PIO<P>, cs_waker: &mut Option<task::Waker>) {
        let irq = pio.irq::<PioIRQ>();
        if pio.get_irq_raw() & (1 << CS_IRQ) != 0 {
            irq.disable_sm_interrupt(CS_IRQ);
            cs_waker.take().map(|w| w.wake());
        }
    }
}

/// The CS monitor state machine
///
/// Responsible for disabling the DI output and providing an interrupt when CS is deasserted.
fn init_sm_cs<P, SMI, PinCs, PinDi>(
    pio: &mut pio::PIO<P>,
    sm: pio::UninitStateMachine<(P, SMI)>,
    irq_cs: u8,
    pin_cs: &PinCs,
    pin_di: &PinDi,
) -> PIOStateMachine<P, SMI>
where
    P: pio::PIOExt,
    SMI: pio::StateMachineIndex,
    PinCs: gpio::AnyPin<Function = P::PinFunction>,
    PinDi: gpio::AnyPin<Function = P::PinFunction>,
{
    let pin_cs = pin_cs.borrow().id().num;
    let pin_di = pin_di.borrow().id().num;

    let mut a = ::pio::Assembler::<{ ::pio::RP2040_MAX_PROGRAM_SIZE }>::new();
    let mut wrap_source = a.label();
    let mut wrap_target = a.label();
    let mut cs_high = a.label();
    {
        use ::pio::*;
        a.jmp(JmpCondition::PinHigh, &mut cs_high);

        a.bind(&mut wrap_target);
        a.wait(0, WaitSource::GPIO, pin_cs, false);

        a.wait(1, WaitSource::GPIO, pin_cs, false);
        a.bind(&mut cs_high);
        a.irq(false, false, irq_cs, false);
        a.set(SetDestination::PINDIRS, 0);
        a.bind(&mut wrap_source);
    }
    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    let installed = pio.install(&program).unwrap();
    let (sm, rx, tx) = pio::PIOBuilder::from_program(installed)
        .jmp_pin(pin_cs)
        .set_pins(pin_di, 1)
        .build(sm);
    let sm = sm.start();

    PIOStateMachine { sm, rx, tx }
}

/// The RX state machine
fn init_sm_rx<P, SMI, PinCs, PinClk, PinDo>(
    pio: &mut pio::PIO<P>,
    sm: pio::UninitStateMachine<(P, SMI)>,
    pin_cs: &PinCs,
    pin_clk: &PinClk,
    pin_do: &PinDo,
) -> PIOStateMachine<P, SMI>
where
    P: pio::PIOExt,
    SMI: pio::StateMachineIndex,
    PinCs: gpio::AnyPin<Function = P::PinFunction>,
    PinClk: gpio::AnyPin<Function = P::PinFunction>,
    PinDo: gpio::AnyPin<Function = P::PinFunction>,
{
    let pin_cs = pin_cs.borrow().id().num;
    let pin_clk = pin_clk.borrow().id().num;
    let pin_do = pin_do.borrow().id().num;

    let mut a = ::pio::Assembler::<{ ::pio::RP2040_MAX_PROGRAM_SIZE }>::new();
    let mut wrap_source = a.label();
    let mut wrap_target = a.label();
    {
        use ::pio::*;
        a.bind(&mut wrap_target);
        a.wait(0, WaitSource::GPIO, pin_cs, false);
        a.wait(0, WaitSource::GPIO, pin_clk, false);
        a.wait(1, WaitSource::GPIO, pin_clk, false);
        a.r#in(InSource::PINS, 1);
        a.bind(&mut wrap_source);
    }
    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    let installed = pio.install(&program).unwrap();
    let (sm, rx, tx) = pio::PIOBuilder::from_program(installed)
        .in_pin_base(pin_do)
        .autopush(true)
        .build(sm);
    let sm = sm.start();

    PIOStateMachine { sm, rx, tx }
}

/// The TX state machine
fn init_sm_tx<P, SMI, PinClk, PinDi>(
    pio: &mut pio::PIO<P>,
    sm: pio::UninitStateMachine<(P, SMI)>,
    pin_clk: &PinClk,
    pin_di: &PinDi,
) -> PIOStateMachine<P, SMI>
where
    P: pio::PIOExt,
    SMI: pio::StateMachineIndex,
    PinClk: gpio::AnyPin<Function = P::PinFunction>,
    PinDi: gpio::AnyPin<Function = P::PinFunction>,
{
    let pin_clk = pin_clk.borrow().id().num;
    let pin_di = pin_di.borrow().id().num;

    let mut a = ::pio::Assembler::<{ ::pio::RP2040_MAX_PROGRAM_SIZE }>::new();
    let mut wrap_source = a.label();
    let mut wrap_target = a.label();
    {
        use ::pio::*;
        a.bind(&mut wrap_target);
        a.out(OutDestination::PINS, 1);
        a.wait(0, WaitSource::GPIO, pin_clk, false);
        a.wait(1, WaitSource::GPIO, pin_clk, false);
        a.bind(&mut wrap_source);
    }
    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    let installed = pio.install(&program).unwrap();
    let (sm, rx, tx) = pio::PIOBuilder::from_program(installed)
        .out_pins(pin_di, 1)
        .autopull(true)
        .build(sm);
    let sm = sm.start();

    PIOStateMachine { sm, rx, tx }
}

/*
pub(crate) fn install_di_driver<P, SMI, CH>(
    pio: &mut pio::PIO<P>,
    sm: pio::UninitStateMachine<(P, SMI)>,
    cs_pin_id: u8,
    clk_pin_id: u8,
    di_pin_id: u8,
    dma_chan: CH,
) where
    P: pio::PIOExt,
    SMI: pio::StateMachineIndex,
    CH: dma::SingleChannel,
{
    let program = pio_proc::pio_asm!(
        ".wrap_target",
        "out pins, 1",
        "wait 0 gpio 5",
        "wait 1 gpio 5",
        ".wrap",
    );

    let installed = pio.install(&program.program).unwrap();
    let (mut sm, _, tx) = pio::PIOBuilder::from_program(installed)
        .set_pins(di_pin_id, 1)
        .out_pins(di_pin_id, 1)
        .in_pin_base(cs_pin_id)
        .autopull(true)
        .build(sm);
    sm.set_pindirs([
        (cs_pin_id, pio::PinDir::Input),
        (clk_pin_id, pio::PinDir::Input),
        (di_pin_id, pio::PinDir::Output),
    ]);
    sm.start();

    let payload = unsafe { &crate::_payload };
    assert_eq!(u32::from_be(payload[0]), 0x49504C42); // "IPLB"
    assert_eq!(u32::from_be(payload[1]), 0x4F4F5420); // "OOT "
    let size = u32::from_be(payload[2]) as usize / core::mem::size_of_val(&payload[0]);
    assert!(size <= payload.len());
    let payload = &payload[..size];
    assert_eq!(u32::from_be(payload[size - 1]), 0x5049434F); // "PICO"

    let block = &payload[..256 / 4];
    let (mut dma_chan, mut tx) = (dma_chan, tx);
    loop {
        while tx.is_full() {}
        tx.write(0); // Dummy word for the command/address
        let mut config = dma::single_buffer::Config::new(dma_chan, block, tx);
        config.bswap(true);
        let transfer = config.start();
        (dma_chan, _, tx) = transfer.wait();
    }
}
*/
