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
struct PioStateMachine<P: pio::PIOExt, SMI: pio::StateMachineIndex> {
    sm: pio::StateMachine<(P, SMI), pio::Running>,
    rx: Option<pio::Rx<(P, SMI)>>,
    tx: Option<pio::Tx<(P, SMI)>>,
}

/// The actual EXI implementation
pub(crate) struct Exi<
    P: pio::PIOExt,
    CS: pio::StateMachineIndex,
    RX: pio::StateMachineIndex,
    TX: pio::StateMachineIndex,
    CHI: dma::ChannelIndex,
    const PIO_IRQ: usize,
    const CS_IRQ: u8,
> {
    /// The chip-select monitor
    ///
    /// Watches for CS changes, triggers interrupts and disables the DI output.
    sm_cs: PioStateMachine<P, CS>,

    /// The data receive state machine
    sm_rx: PioStateMachine<P, RX>,

    /// The data transmit state machine
    sm_tx: PioStateMachine<P, TX>,

    /// A single DMA channel ought to be enough for anybody
    dma: Option<dma::Channel<CHI>>,
}

/// ZST to encapsulate the ISR implementation
pub(crate) struct ExiIntHandler<P: pio::PIOExt, const PIO_IRQ: usize, const CS_IRQ: u8> {
    _pio: marker::PhantomData<P>,
}

/// Pin mapping
pub(crate) struct ExiPins<CS: gpio::AnyPin, CLK: gpio::AnyPin, DO: gpio::AnyPin, DI: gpio::AnyPin> {
    pub cs: CS,
    pub clk: CLK,
    pub r#do: DO,
    pub di: DI,
}

impl<P, CS, RX, TX, CHI, const PIO_IRQ: usize, const CS_IRQ: u8>
    Exi<P, CS, RX, TX, CHI, PIO_IRQ, CS_IRQ>
where
    P: pio::PIOExt,
    CS: pio::StateMachineIndex,
    RX: pio::StateMachineIndex,
    TX: pio::StateMachineIndex,
    CHI: dma::ChannelIndex,
{
    /// Initialize EXI with all the resources it requires
    #[allow(clippy::type_complexity)] // This lint doesn't like the scary tuple
    pub(crate) fn new<PinCs, PinClk, PinDo, PinDi>(
        pio: &mut pio::PIO<P>,
        state_machines: (
            pio::UninitStateMachine<(P, CS)>,
            pio::UninitStateMachine<(P, RX)>,
            pio::UninitStateMachine<(P, TX)>,
        ),
        pins: ExiPins<PinCs, PinClk, PinDo, PinDi>,
        dma: dma::Channel<CHI>,
    ) -> (Self, ExiIntHandler<P, PIO_IRQ, CS_IRQ>)
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
        let pin_cs = pins
            .cs
            .into()
            .into_function()
            .into_pull_type::<gpio::PullUp>();
        let pin_clk = pins.clk.into().into_function();
        let pin_do = pins.r#do.into().into_function();
        let mut pin_di = pins.di.into().into_function();
        // Required to outdrive RTC-DOL
        pin_di.set_drive_strength(gpio::OutputDriveStrength::EightMilliAmps);

        let (sm_cs, sm_rx, sm_tx) = state_machines;
        let sm_cs = init_sm_cs(pio, sm_cs, CS_IRQ, &pin_cs, &pin_di);
        let sm_rx = init_sm_rx(pio, sm_rx, &pin_cs, &pin_clk, &pin_do);
        let sm_tx = init_sm_tx(pio, sm_tx, &pin_clk, &pin_di);

        (
            Self {
                sm_cs,
                sm_rx,
                sm_tx,
                dma: Some(dma),
            },
            ExiIntHandler {
                _pio: marker::PhantomData {},
            },
        )
    }

    /// Wait for CS to be deasserted (edge-triggered)
    pub(crate) async fn transaction_end<PIO, W>(&self, pio: &mut PIO, cs_waker: &mut W)
    where
        PIO: rtic::Mutex<T = pio::PIO<P>>,
        W: rtic::Mutex<T = Option<task::Waker>>,
    {
        pio.lock(|pio| {
            pio.clear_irq(1 << CS_IRQ);
        });
        self.bus_idle(pio, cs_waker).await;
    }

    /// Wait for CS to be deasserted (level-triggered)
    pub(crate) async fn bus_idle<PIO, W>(&self, pio: &mut PIO, cs_waker: &mut W)
    where
        PIO: rtic::Mutex<T = pio::PIO<P>>,
        W: rtic::Mutex<T = Option<task::Waker>>,
    {
        future::poll_fn(|poll_ctx| {
            pio.lock(|pio| {
                if pio.get_irq_raw() & (1 << CS_IRQ) != 0 {
                    pio.clear_irq(1 << CS_IRQ);
                    return core::task::Poll::Ready(());
                }

                cs_waker.lock(|w| {
                    w.replace(poll_ctx.waker().clone());
                });
                pio.irq::<PIO_IRQ>().enable_sm_interrupt(CS_IRQ);
                core::task::Poll::Pending
            })
        })
        .await;
    }

    /// Respond to a command
    pub(crate) async fn respond<PIO, W>(
        &mut self,
        pio: &mut PIO,
        cs_waker: &mut W,
        body: &'static [u32],
    ) where
        PIO: rtic::Mutex<T = pio::PIO<P>>,
        W: rtic::Mutex<T = Option<task::Waker>>,
    {
        let chan = self.dma.take().unwrap();
        let mut tx = self.sm_tx.tx.take().unwrap();

        // Enable DI output
        {
            let mut tx = self.sm_cs.tx.take().unwrap();
            tx.write(
                // This disregards delay and side-set, but we don't really care
                ::pio::InstructionOperands::SET {
                    destination: ::pio::SetDestination::PINDIRS,
                    data: 1,
                }
                .encode()
                .into(),
            );
            self.sm_cs.tx.replace(tx);
        }

        tx.write(0); // Dummy word for the command/address

        let mut config = dma::single_buffer::Config::new(chan, body, tx);
        config.bswap(true);
        let transfer = config.start();

        self.transaction_end(pio, cs_waker).await;
        assert!(transfer.is_done());
        let (chan, _, tx) = transfer.wait();
        assert!(tx.is_empty());

        self.sm_tx.tx.replace(tx);
        self.dma.replace(chan);
    }
}

impl<P, const PIO_IRQ: usize, const CS_IRQ: u8> ExiIntHandler<P, PIO_IRQ, CS_IRQ>
where
    P: pio::PIOExt,
{
    /// The real ISR implementation
    pub(crate) fn on_pio_interrupt(&self, pio: &pio::PIO<P>, cs_waker: &mut Option<task::Waker>) {
        let irq = pio.irq::<PIO_IRQ>();
        if pio.get_irq_raw() & (1 << CS_IRQ) != 0 {
            irq.disable_sm_interrupt(CS_IRQ);
            if let Some(w) = cs_waker.take() {
                w.wake();
            }
        }
    }
}

/// The CS monitor state machine
///
/// Responsible for toggling the DI output and providing an interrupt when CS is deasserted.
fn init_sm_cs<P, SMI, PinCs, PinDi>(
    pio: &mut pio::PIO<P>,
    sm: pio::UninitStateMachine<(P, SMI)>,
    irq_cs: u8,
    pin_cs: &PinCs,
    pin_di: &PinDi,
) -> PioStateMachine<P, SMI>
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

        // `pull noblock` is equivalent to `mov osr, x` if the FIFO is empty.
        // Initialize x with a nop.
        a.pull(false, true);
        a.out(OutDestination::X, 32);

        a.jmp(JmpCondition::PinHigh, &mut cs_high);

        a.bind(&mut wrap_target);
        a.wait(0, WaitSource::GPIO, pin_cs, false);

        // Execute one instruction from the FIFO when CS is asserted.
        // This allows enabling the output asynchronously.
        a.pull(false, false);
        a.out(OutDestination::EXEC, 32);

        a.wait(1, WaitSource::GPIO, pin_cs, false);
        a.bind(&mut cs_high);
        a.irq(false, false, irq_cs, false);
        a.set(SetDestination::PINDIRS, 0);
        a.bind(&mut wrap_source);
    }
    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    let installed = pio.install(&program).unwrap();
    let (sm, rx, mut tx) = pio::PIOBuilder::from_program(installed)
        .jmp_pin(pin_cs)
        .set_pins(pin_di, 1)
        .build(sm);
    let sm = sm.start();

    tx.write(
        // `mov y, y` is the canonical nop
        ::pio::Instruction {
            operands: ::pio::InstructionOperands::MOV {
                destination: ::pio::MovDestination::Y,
                op: ::pio::MovOperation::None,
                source: ::pio::MovSource::Y,
            },
            delay: 0,
            side_set: None,
        }
        .encode(program.side_set)
        .into(),
    );

    PioStateMachine {
        sm,
        rx: Some(rx),
        tx: Some(tx),
    }
}

/// The RX state machine
fn init_sm_rx<P, SMI, PinCs, PinClk, PinDo>(
    pio: &mut pio::PIO<P>,
    sm: pio::UninitStateMachine<(P, SMI)>,
    pin_cs: &PinCs,
    pin_clk: &PinClk,
    pin_do: &PinDo,
) -> PioStateMachine<P, SMI>
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

    PioStateMachine {
        sm,
        rx: Some(rx),
        tx: Some(tx),
    }
}

/// The TX state machine
fn init_sm_tx<P, SMI, PinClk, PinDi>(
    pio: &mut pio::PIO<P>,
    sm: pio::UninitStateMachine<(P, SMI)>,
    pin_clk: &PinClk,
    pin_di: &PinDi,
) -> PioStateMachine<P, SMI>
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

    PioStateMachine {
        sm,
        rx: Some(rx),
        tx: Some(tx),
    }
}
