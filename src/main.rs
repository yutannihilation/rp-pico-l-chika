#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::prelude::*,
    hal::{gpio::DynPin, pwm::Slices},
};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use libm::sinf;

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

    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);

    let mut pwm1 = pwm_slices.pwm1;
    pwm1.set_ph_correct();
    pwm1.enable();

    let mut pwm2 = pwm_slices.pwm2;
    pwm2.set_ph_correct();
    pwm2.enable();

    let mut pwm3 = pwm_slices.pwm3;
    pwm3.set_ph_correct();
    pwm3.enable();

    let mut pwm4 = pwm_slices.pwm4;
    pwm4.set_ph_correct();
    pwm4.enable();

    info!("TOP: {}", pwm1.get_top());
    pwm1.set_top(u16::MAX);
    pwm2.set_top(u16::MAX);
    pwm3.set_top(u16::MAX);
    pwm4.set_top(u16::MAX);

    let pwm1 = pwm1.into_mode::<bsp::hal::pwm::FreeRunning>();
    let mut ch_a1 = pwm1.channel_a;
    let mut ch_b1 = pwm1.channel_b;
    ch_a1.output_to(pins.gpio2);
    ch_b1.output_to(pins.gpio3);

    let pwm2 = pwm2.into_mode::<bsp::hal::pwm::FreeRunning>();
    let mut ch_a2 = pwm2.channel_a;
    let mut ch_b2 = pwm2.channel_b;
    ch_a2.output_to(pins.gpio4);
    ch_b2.output_to(pins.gpio5);

    let pwm3 = pwm3.into_mode::<bsp::hal::pwm::FreeRunning>();
    let mut ch_a3 = pwm3.channel_a;
    let mut ch_b3 = pwm3.channel_b;
    ch_a3.output_to(pins.gpio6);
    ch_b3.output_to(pins.gpio7);

    let pwm4 = pwm4.into_mode::<bsp::hal::pwm::FreeRunning>();
    let mut ch_a4 = pwm4.channel_a;
    ch_a4.output_to(pins.gpio8);

    // // Every GPIO has a different type, so we cannot bundle them as an array.
    // // But, we can convert them to DynPin.
    // let mut led_pins: [DynPin; 7] = [
    //     pins.gpio2.into_mode::<bsp::hal::gpio::FunctionPwm>().into(),
    //     pins.gpio3.into_mode::<bsp::hal::gpio::FunctionPwm>().into(),
    //     pins.gpio4.into_mode::<bsp::hal::gpio::FunctionPwm>().into(),
    //     pins.gpio5.into_mode::<bsp::hal::gpio::FunctionPwm>().into(),
    //     pins.gpio6.into_mode::<bsp::hal::gpio::FunctionPwm>().into(),
    //     pins.gpio7.into_mode::<bsp::hal::gpio::FunctionPwm>().into(),
    //     pins.gpio8.into_mode::<bsp::hal::gpio::FunctionPwm>().into(),
    // ];

    let d = u16::MAX as f32 * 0.5;
    let mut phase = 0.0;
    loop {
        ch_a1.set_duty((d + d * sinf((phase + 0.00) * 2.0 * 3.1415)) as _);
        ch_b1.set_duty((d + d * sinf((phase + 0.07) * 2.0 * 3.1415)) as _);
        ch_a2.set_duty((d + d * sinf((phase + 0.14) * 2.0 * 3.1415)) as _);
        ch_b2.set_duty((d + d * sinf((phase + 0.21) * 2.0 * 3.1415)) as _);
        ch_a3.set_duty((d + d * sinf((phase + 0.28) * 2.0 * 3.1415)) as _);
        ch_b3.set_duty((d + d * sinf((phase + 0.35) * 2.0 * 3.1415)) as _);
        ch_a4.set_duty((d + d * sinf((phase + 0.42) * 2.0 * 3.1415)) as _);

        delay.delay_ms(17);
        phase += 0.01;
        // info!("pin1");
        // led_pins[0].set_high().unwrap();
        // delay.delay_ms(50);

        // info!("pin2");
        // led_pins[1].set_high().unwrap();
        // delay.delay_ms(500);

        // info!("pin3");
        // led_pins[2].set_high().unwrap();
        // delay.delay_ms(500);

        // info!("pin4");
        // led_pins[3].set_high().unwrap();
        // delay.delay_ms(500);

        // info!("pin5");
        // led_pins[4].set_high().unwrap();
        // led_pins[0].set_low().unwrap();
        // delay.delay_ms(500);

        // info!("pin6");
        // led_pins[5].set_high().unwrap();
        // led_pins[1].set_low().unwrap();
        // delay.delay_ms(500);

        // info!("pin7");
        // led_pins[6].set_high().unwrap();
        // led_pins[2].set_low().unwrap();
        // delay.delay_ms(500);

        // led_pins[3].set_low().unwrap();
        // delay.delay_ms(500);
        // led_pins[4].set_low().unwrap();
        // delay.delay_ms(500);
        // led_pins[5].set_low().unwrap();
        // delay.delay_ms(500);
        // led_pins[6].set_low().unwrap();
        // delay.delay_ms(500);
    }
}

// End of file
