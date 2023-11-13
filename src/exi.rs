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
//! We have a few reasons for using PIO:
//! - we need parallel output to multiple pins, in order to outdrive RTC-DOL;
//! - the hardware SPI block in the RP2040 is too slow:
//!   section *4.4.3.4. Clock ratios* of the datasheet
//!   says *SYSCLK* needs to be at least 12 times faster than the bus clock, which would be 324MHz;
//!   RP2040 overclocks very well, but that is too high for comfort.
//!
//! Lastly, this module requires overclocking the microcontroller to meet DI (MISO) timings.  
//! The simplest PIO program to detect an edge on an input and toggle a GPIO takes 4-5 cycles to
//! react.
//! At the default setting of 125MHz, the *SYSCLK* to bus clock ratio is ~4.6.
//! This means that it's a coin flip whether we'll make it before the next rising edge.  
//! Instead, we target a *SYSCLK* rate of 200MHz, resulting in a clock ratio of ~7.4.
//! In the worst case, this leaves about 2 cycles (10ns) of headroom,
//! which exceeds the 8.93ns setup time specified by the Flipper datasheet.

use crate::bsp::hal::pio;

pub(crate) fn install_di_driver<P, SMI>(
    pio: &mut pio::PIO<P>,
    sm: pio::UninitStateMachine<(P, SMI)>,
    cs_pin_id: u8,
    clk_pin_id: u8,
    di_pin_id: u8,
) where
    P: pio::PIOExt,
    SMI: pio::StateMachineIndex,
{
    let program = pio_proc::pio_asm!(
        "wait 1 gpio 4",
        "wait 0 gpio 4",
        ".wrap_target",
        "set pins, 0",
        "wait 0 gpio 5",
        "wait 1 gpio 5",
        "set pins, 1",
        "wait 0 gpio 5",
        "wait 1 gpio 5",
        ".wrap",
    );

    let installed = pio.install(&program.program).unwrap();
    let (mut sm, _, _) = pio::PIOBuilder::from_program(installed)
        .set_pins(di_pin_id, 1)
        .in_pin_base(cs_pin_id)
        .build(sm);
    sm.set_pindirs([
        (cs_pin_id, pio::PinDir::Input),
        (clk_pin_id, pio::PinDir::Input),
        (di_pin_id, pio::PinDir::Output),
    ]);
    sm.start();
}
