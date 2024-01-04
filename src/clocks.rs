//! Clock tree configuration

use crate::bsp::{
    self,
    hal::{
        clocks,
        fugit::{HertzU32, RateExtU32},
        pac, pll,
        watchdog::Watchdog,
        xosc,
    },
};

// Portability: the PLL configurations assume that the board has a 12MHz crystal
#[allow(clippy::assertions_on_constants)]
const _: () = assert!(bsp::XOSC_CRYSTAL_FREQ == 12_000_000);

/// Initialize the clock tree according to project requirements
///
/// This is a customized copy of [`clocks::init_clocks_and_plls`].\
/// Currently, the only change is that we target a *SYSCLK* rate of 200MHz.
/// See [`crate::exi`] for an explanation.
pub(crate) fn init_clocks_and_plls(
    xosc_dev: pac::XOSC,
    clocks_dev: pac::CLOCKS,
    pll_sys_dev: pac::PLL_SYS,
    pll_usb_dev: pac::PLL_USB,
    resets: &mut pac::RESETS,
    watchdog: &mut Watchdog,
) -> Result<clocks::ClocksManager, clocks::InitError> {
    let xosc = xosc::setup_xosc_blocking(xosc_dev, bsp::XOSC_CRYSTAL_FREQ.Hz())
        .map_err(clocks::InitError::XoscErr)?;

    // Configure watchdog tick generation to tick over every microsecond
    watchdog.enable_tick_generation((bsp::XOSC_CRYSTAL_FREQ / 1_000_000) as u8);

    let mut clocks = clocks::ClocksManager::new(clocks_dev);

    let pll_sys = pll::setup_pll_blocking(
        pll_sys_dev,
        xosc.operating_frequency(),
        // 200 megahurts
        pll::PLLConfig {
            vco_freq: HertzU32::MHz(1200),
            refdiv: 1,
            post_div1: 6,
            post_div2: 1,
        },
        &mut clocks,
        resets,
    )
    .map_err(clocks::InitError::PllError)?;
    let pll_usb = pll::setup_pll_blocking(
        pll_usb_dev,
        xosc.operating_frequency(),
        pll::common_configs::PLL_USB_48MHZ,
        &mut clocks,
        resets,
    )
    .map_err(clocks::InitError::PllError)?;

    clocks
        .init_default(&xosc, &pll_sys, &pll_usb)
        .map_err(clocks::InitError::ClockError)?;
    Ok(clocks)
}
