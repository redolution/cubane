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

fn check_payload() -> &'static [u32] {
    let payload = unsafe { &crate::_payload };
    assert_eq!(u32::from_be(payload[0]), 0x49504C42); // "IPLB"
    assert_eq!(u32::from_be(payload[1]), 0x4F4F5420); // "OOT "
    let size = u32::from_be(payload[2]) as usize / core::mem::size_of_val(&payload[0]);
    assert!(size <= payload.len());
    let payload = &payload[..size];
    assert_eq!(u32::from_be(payload[size - 1]), 0x5049434F); // "PICO"

    payload
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
        cmd_waker: Option<core::task::Waker>,
    }

    #[local]
    struct Local {
        exi: exi::Exi<pac::PIO0, pio::SM0, pio::SM1, pio::SM2, pio::SM3, dma::CH0, 0, 0, 1>,
        cs_int: exi::PioIntHandler<pac::PIO0, 0, 0>,
        cmd_int: exi::PioIntHandler<pac::PIO0, 0, 1>,
        payload: &'static [u32],
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

        let (mut pio0, pio0_sm0, pio0_sm1, pio0_sm2, pio0_sm3) =
            ctx.device.PIO0.split(&mut ctx.device.RESETS);

        #[cfg(not(feature = "pbcompat"))]
        let exi_pins = exi::ExiPins {
            cs: pins.gpio28,
            clk: pins.gpio22,
            r#do: pins.gpio26,
            di: pins.gpio27,
        };

        #[cfg(feature = "pbcompat")]
        let exi_pins = exi::ExiPins {
            cs: pins.gpio4,
            clk: pins.gpio5,
            r#do: pins.gpio3,
            di: pins.gpio6,
        };

        let (exi, cs_int, cmd_int) = exi::Exi::new(
            &mut pio0,
            (pio0_sm0, pio0_sm1, pio0_sm2, pio0_sm3),
            exi_pins,
            dma.ch0,
        );

        let payload = crate::check_payload();

        root::spawn().unwrap();

        (
            Shared {
                pio0,
                cs_waker: None,
                cmd_waker: None,
            },
            Local {
                exi,
                cs_int,
                cmd_int,
                payload,
            },
        )
    }

    #[task(local = [exi, payload], shared = [pio0, cs_waker, cmd_waker])]
    async fn root(ctx: root::Context) {
        let root::SharedResources {
            mut pio0,
            mut cs_waker,
            mut cmd_waker,
            ..
        } = ctx.shared;
        let root::LocalResources { exi, payload, .. } = ctx.local;

        loop {
            let cmd = exi.receive_command(&mut pio0, &mut cmd_waker);

            if cmd == Some((0x800 - 8) << 6) {
                break;
            }
        }

        //defmt::debug!("hello");

        for block in payload.chunks(1024 / 4) {
            exi.respond(&mut pio0, &mut cs_waker, block);
        }
        defmt::debug!("done");
    }

    #[task(
        binds = PIO0_IRQ_0,
        local = [cs_int, cmd_int],
        shared = [pio0, cs_waker, cmd_waker],
    )]
    fn pio0_irq0(ctx: pio0_irq0::Context) {
        let pio0_irq0::SharedResources {
            pio0,
            cs_waker,
            cmd_waker,
            ..
        } = ctx.shared;
        let pio0_irq0::LocalResources {
            cs_int, cmd_int, ..
        } = ctx.local;

        (pio0, cs_waker, cmd_waker).lock(|pio, cs_waker, cmd_waker| {
            cs_int.on_irq(pio, cs_waker);
            cmd_int.on_irq(pio, cmd_waker);
        });
    }
}
