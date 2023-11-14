#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use panic_probe as _;
pub(crate) use rp_pico as bsp;

mod clocks;
mod exi;
mod logic_analyzer;

#[rtic::app(
    device = crate::bsp::pac,
    dispatchers = [TIMER_IRQ_1],
)]
mod app {
    use defmt::*;

    use crate::bsp::{
        self,
        hal::{
            dma::DMAExt,
            gpio::{FunctionPio0, Pin},
            pio::PIOExt,
            sio::Sio,
            watchdog::Watchdog,
        },
    };
    use crate::{clocks, exi, logic_analyzer};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        info!("Program start");
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        let sio = Sio::new(ctx.device.SIO);

        let _clocks = clocks::init_clocks_and_plls(
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let pins = bsp::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        let cs_pin: Pin<_, FunctionPio0, _> = pins.gpio4.into_function();
        let cs_pin_id = cs_pin.id().num;
        let clk_pin: Pin<_, FunctionPio0, _> = pins.gpio5.into_function();
        let clk_pin_id = clk_pin.id().num;
        let di_pin: Pin<_, FunctionPio0, _> = pins.gpio6.into_function();
        let di_pin_id = di_pin.id().num;
        let (mut pio0, pio0_sm0, _, _, _) = ctx.device.PIO0.split(&mut ctx.device.RESETS);
        exi::install_di_driver(&mut pio0, pio0_sm0, cs_pin_id, clk_pin_id, di_pin_id);

        let (mut pio1, pio1_sm0, _, _, _) = ctx.device.PIO1.split(&mut ctx.device.RESETS);
        let dma = ctx.device.DMA.split(&mut ctx.device.RESETS);
        logic_analyzer::capture_trace(&mut pio1, pio1_sm0, dma.ch0);

        (Shared {}, Local {})
    }
}
