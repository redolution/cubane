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
        cs_waker: Option<core::task::Waker>,
    }

    #[local]
    struct Local {
        exi: exi::Exi<pac::PIO0, pio::SM0, pio::SM1, pio::SM2, dma::CH0, 0, 0>,
        exi_int: exi::ExiIntHandler<pac::PIO0, 0, 0>,
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

        let (mut pio0, pio0_sm0, pio0_sm1, pio0_sm2, _pio0_sm3) =
            ctx.device.PIO0.split(&mut ctx.device.RESETS);

        let (exi, exi_int) = exi::Exi::new(
            &mut pio0,
            (pio0_sm0, pio0_sm1, pio0_sm2),
            exi::ExiPins {
                cs: pins.gpio4,
                clk: pins.gpio5,
                r#do: pins.gpio3,
                di: pins.gpio6,
            },
            dma.ch0,
        );

        root::spawn().unwrap();

        (
            Shared {
                pio0,
                cs_waker: None,
            },
            Local { exi, exi_int },
        )
    }

    #[task(local = [exi], shared = [pio0, cs_waker])]
    async fn root(ctx: root::Context) {
        let root::SharedResources {
            mut pio0,
            mut cs_waker,
            ..
        } = ctx.shared;
        let root::LocalResources { exi, .. } = ctx.local;

        loop {
            exi.transaction_end(&mut pio0, &mut cs_waker).await;
            defmt::debug!("hello world");
        }
    }

    #[task(binds = PIO0_IRQ_0, local = [exi_int], shared = [pio0, cs_waker])]
    fn pio0_irq0(ctx: pio0_irq0::Context) {
        let pio0_irq0::SharedResources { pio0, cs_waker, .. } = ctx.shared;
        let pio0_irq0::LocalResources { exi_int, .. } = ctx.local;

        (pio0, cs_waker).lock(|pio, cs_waker| {
            exi_int.on_pio_interrupt(pio, cs_waker);
        });
    }
}
