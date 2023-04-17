// ## The pin layout of A-551SR:
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
// Each pin corresponds to the following positions of the 7 segments.
// (3 & 8 are Vin, and 5 is for the right-bottom period)
//
//     ┌─ 7 ─┐
//     9     6
//     ├─10 ─┤
//     1     4
//     └─ 2 ─┘
//
// ## 74HC595
//
//    ┌─────v─────┐
//  1 │           │ 16
//  2 │           │ 15
//  3 │           │ 14 Input  <------------ GPIO2
//  4 │           │ 13
//  5 │           │ 12 Clock for input  <-- GPIO3
//  6 │           │ 11 Clock for output  <- GPIO4
//  7 │           │ 10
//  8 │           │  9
//    └───────────┘

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

// TIMER_IRQ_1 is chosen probably because TIMER_IRQ_0 is used by the Timer by default?
// cf., https://github.com/rtic-rs/rtic/blob/ef8046b060a375fd5e6b23d62c3a9a303bbd6e11/rtic-monotonics/src/rp2040.rs#L170
#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {

    // use panic_probe as _;
    use panic_halt as _;

    use embedded_hal::digital::v2::OutputPin;
    use fugit::{MicrosDurationU32, MicrosDurationU64};
    use rp_pico::{
        hal::{self, clocks::init_clocks_and_plls, timer::Alarm, watchdog::Watchdog, Sio},
        XOSC_CRYSTAL_FREQ,
    };

    // Import pio crates
    use hal::pio::{PIOBuilder, Tx};
    use pio_proc::pio_file;

    // Pull in any important traits
    use rp_pico::hal::prelude::*;

    use rtic_monotonics::rp2040::*;

    const SCAN_TIME_US: MicrosDurationU32 = MicrosDurationU32::millis(100);

    #[derive(Debug, Clone, Copy)]
    struct PwmStep {
        length: u32,
        data: u32,
    }

    pub struct PwmData {
        pwm_levels: [u32; 8],
        pwm_steps: [PwmStep; 9],
    }

    impl PwmData {
        fn new() -> Self {
            let null_step = PwmStep {
                length: 255,
                data: 0,
            };

            Self {
                pwm_levels: [0; 8],
                pwm_steps: [null_step; 9],
            }
        }

        fn reflect(&mut self) {
            let mut indices: [usize; 8] = [0, 1, 2, 3, 4, 5, 6, 7];
            indices.sort_unstable_by_key(|&i| self.pwm_levels[i]);

            let mut data = 255;
            let mut prev_level = 0;
            let mut cur_level = 0;

            for (i, &cur_index) in indices.iter().enumerate() {
                cur_level = self.pwm_levels[cur_index];

                self.pwm_steps[i] = PwmStep {
                    length: cur_level - prev_level,
                    data,
                };

                data &= !(1 << cur_index);
                // info!("{:b}", data);

                prev_level = cur_level;
            }

            // period after all pins are set low
            self.pwm_steps[8] = PwmStep {
                length: 255 - cur_level,
                data: 0,
            };
        }
    }

    #[shared]
    struct Shared {
        data: PwmData,
    }

    #[local]
    struct Local {
        // tx ix is used in only one task, so this can be Local
        tx: Tx<rp_pico::hal::pio::PIO0SM0>,

        // TODO: This LED is for debugging. Remove this later.
        led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let mut resets = c.device.RESETS;
        let token = rtic_monotonics::create_rp2040_monotonic_token!();
        Timer::start(c.device.TIMER, &mut resets, token);

        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }

        let sio = Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // Note: while the compiler never complains, we cannot use pac::Peripherals::take().unwrap() directly
        let (mut pio0, sm0, _, _, _) = c.device.PIO0.split(&mut resets);

        // Create a pio program
        let program = pio_file!("./src/shift_register.pio", select_program("shift_register"),);
        let installed = pio0.install(&program.program).unwrap();

        let out_pin: hal::gpio::Pin<_, hal::gpio::FunctionPio0> = pins.gpio2.into_mode();
        let _clock_pin: hal::gpio::Pin<_, hal::gpio::FunctionPio0> = pins.gpio3.into_mode();
        let _ratch_pin: hal::gpio::Pin<_, hal::gpio::FunctionPio0> = pins.gpio4.into_mode();

        let mut led = pins.led.into_push_pull_output();
        led.set_low().unwrap();

        // Build the pio program and set pin both for set and side set!
        // We are running with the default divider which is 1 (max speed)
        let out_pin_id = out_pin.id().num;
        let (mut sm, _, tx) = PIOBuilder::from_program(installed)
            .out_pins(out_pin_id, 1)
            .side_set_pin_base(out_pin_id + 1)
            .build(sm0);

        sm.set_pindirs([
            (out_pin_id, hal::pio::PinDir::Output),
            (out_pin_id + 1, hal::pio::PinDir::Output),
            (out_pin_id + 2, hal::pio::PinDir::Output),
        ]);

        // Start state machine
        let _sm = sm.start();

        let mut data = PwmData::new();

        data.pwm_levels = [0, 10, 40, 0, 180, 90, 0, 130];
        data.reflect();

        timer_irq::spawn().ok();

        (Shared { data }, Local { tx, led })
    }

    // #[idle(shared = [led])]
    // fn background(mut c: background::Context) -> ! {
    //     loop {
    //         c.shared.led.lock(|l| l.set_low().unwrap());
    //         Timer::delay(5000.millis());
    //         c.shared.led.lock(|l| l.set_high().unwrap());
    //         Timer::delay(5000.millis());
    //     }
    // }

    // #[task(local = [led, tog: bool = true])]
    // async fn blink(cx: blink::Context) {
    //     loop {
    //         if *cx.local.tog {
    //             cx.local.led.set_high().unwrap();
    //             *cx.local.tog = false;
    //         } else {
    //             cx.local.led.set_low().unwrap();
    //             *cx.local.tog = true;
    //         }
    //         Timer::delay(10.millis()).await;
    //     }
    // }

    #[task(
        shared = [&data],
        local = [led, tx, tog: bool = true, step: u8 = 0],
    )]
    async fn timer_irq(c: timer_irq::Context) {
        let data = c.shared.data;
        let tx = c.local.tx;

        loop {
            for step in data.pwm_steps {
                tx.write(step.data << 24);

                let delay_ms = (step.length as u64).micros();
                Timer::delay(delay_ms).await;
            }
            *c.local.step = (*c.local.step + 1) % 7;

            // debug
            if *c.local.tog {
                c.local.led.set_high().unwrap();
            } else {
                c.local.led.set_low().unwrap();
            }
            *c.local.tog = !*c.local.tog;
        }
    }
}
