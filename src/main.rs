#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // GPIO0 and GPIO1 are used for communicating with the debug probe, so this starts from GPIO2.
    //
    // The pin layout of A-551SR:
    //
    //   10 9 8 7 6
    //   ┌───────┐
    //   │       │
    //   │       │
    //   │       │
    //   │       │
    //   │       │
    //   └───────┘
    //   1 2 3 4 5
    //
    // Each pin corresponds to the following segment of the "8".
    // (3 & 8 are Vin, and 5 is for the right-bottom period)
    //
    //     ┌─ 7 ─┐
    //     9     6
    //     ├─10 ─┤
    //     1     4
    //     └─ 2 ─┘
    //
    let mut pin1 = pins.gpio2.into_push_pull_output();
    let mut pin2 = pins.gpio3.into_push_pull_output();
    let mut pin3 = pins.gpio4.into_push_pull_output();
    let mut pin4 = pins.gpio5.into_push_pull_output();
    let mut pin5 = pins.gpio6.into_push_pull_output();
    let mut pin6 = pins.gpio7.into_push_pull_output();
    let mut pin7 = pins.gpio8.into_push_pull_output();

    loop {
        info!("pin1");
        pin1.set_high().unwrap();
        delay.delay_ms(500);

        info!("pin2");
        pin2.set_high().unwrap();
        delay.delay_ms(500);

        info!("pin3");
        pin3.set_high().unwrap();
        delay.delay_ms(500);

        info!("pin4");
        pin4.set_high().unwrap();
        delay.delay_ms(500);

        info!("pin5");
        pin5.set_high().unwrap();
        pin1.set_low().unwrap();
        delay.delay_ms(500);

        info!("pin6");
        pin6.set_high().unwrap();
        pin2.set_low().unwrap();
        delay.delay_ms(500);

        info!("pin7");
        pin7.set_high().unwrap();
        pin3.set_low().unwrap();
        delay.delay_ms(500);

        pin4.set_low().unwrap();
        delay.delay_ms(500);
        pin5.set_low().unwrap();
        delay.delay_ms(500);
        pin6.set_low().unwrap();
        delay.delay_ms(500);
        pin7.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
