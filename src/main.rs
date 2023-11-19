#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use panic_probe as _;
pub(crate) use rp_pico as bsp;

mod clocks;
mod exi;
mod logic_analyzer;

extern "C" {
    /// The BS2 payload section in flash
    ///
    /// The bottom 512KiB of the 2MiB flash are reserved for firmware.
    static _payload: [u32; (2048 - 512) * 1024 / core::mem::size_of::<u32>()];
}

#[rtic::app(
    device = crate::bsp::pac,
    dispatchers = [TIMER_IRQ_1],
)]
mod app {
    use crate::bsp::{
        self,
        hal::{
            dma::{self, DMAExt},
            pio::{self, PIOExt},
            sio::Sio,
            watchdog::Watchdog,
        },
        pac,
    };
    use crate::{clocks, exi};

    #[shared]
    struct Shared {
        pio0: pio::PIO<pac::PIO0>,
    }

    #[local]
    struct Local {
        exi: exi::EXI<pac::PIO0, pio::SM0, pio::SM1, pio::SM2, dma::CH0>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        defmt::info!("Program start");
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

        let dma = ctx.device.DMA.split(&mut ctx.device.RESETS);

        let (mut pio0, pio0_sm0, pio0_sm1, pio0_sm2, _) =
            ctx.device.PIO0.split(&mut ctx.device.RESETS);

        let irq_cs = 0;
        let exi = exi::EXI::new(
            &mut pio0, irq_cs, pio0_sm0, pio0_sm1, pio0_sm2, pins.gpio4, pins.gpio5, pins.gpio3,
            pins.gpio6, dma.ch0,
        );
        pio0.irq0().enable_sm_interrupt(irq_cs);

        root::spawn().unwrap();

        (Shared { pio0 }, Local { exi })
    }

    #[task(local = [exi])]
    async fn root(_ctx: root::Context) {
    }

    #[task(binds = PIO0_IRQ_0, shared = [&pio0])]
    fn pio0_irq0(ctx: pio0_irq0::Context) {
        ctx.shared.pio0.clear_irq(1 << 0);
    }
}
