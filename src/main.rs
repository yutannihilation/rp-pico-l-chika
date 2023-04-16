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

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {

    // Ensure we halt the program on panic (if we don't mention this crate it won't
    // be linked)
    use panic_halt as _;

    use defmt::info;
    use embedded_hal::digital::v2::OutputPin;
    use fugit::MicrosDurationU32;
    use rp_pico::{
        hal::{self, clocks::init_clocks_and_plls, timer::Alarm, watchdog::Watchdog, Sio},
        XOSC_CRYSTAL_FREQ,
    };

    // Import pio crates
    use hal::pio::{PIOBuilder, Tx, ValidStateMachine};
    use pio_proc::pio_file;

    // Pull in any important traits
    use rp_pico::hal::prelude::*;

    const SCAN_TIME_US: MicrosDurationU32 = MicrosDurationU32::secs(3);

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
        timer: hal::Timer,
        alarm: hal::timer::Alarm0,

        // Note: It seems #[shared] macro doesn't support type parameter syntax,
        //       so we cannot use AnyPin here. DynPin might work, but let's
        //       decide which pins to use here.
        out_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio2, hal::gpio::FunctionPio0>,
        clock_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio3, hal::gpio::FunctionPio0>,
        ratch_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio4, hal::gpio::FunctionPio0>,

        led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>,

        tx: Tx<rp_pico::hal::pio::PIO0SM0>,

        data: PwmData,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let _clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let mut pac = rp_pico::hal::pac::Peripherals::take().unwrap();
        let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

        // Create a pio program
        let program = pio_file!("./src/shift_register.pio", select_program("shift_register"),);
        let installed = pio0.install(&program.program).unwrap();

        let out_pin: hal::gpio::Pin<_, hal::gpio::FunctionPio0> = pins.gpio2.into_mode();
        let clock_pin: hal::gpio::Pin<_, hal::gpio::FunctionPio0> = pins.gpio3.into_mode();
        let ratch_pin: hal::gpio::Pin<_, hal::gpio::FunctionPio0> = pins.gpio4.into_mode();

        let mut led = pins.led.into_push_pull_output();
        led.set_low().unwrap();

        // Build the pio program and set pin both for set and side set!
        // We are running with the default divider which is 1 (max speed)
        let out_pin_id = out_pin.id().num;
        let (mut sm, _, mut tx) = PIOBuilder::from_program(installed)
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

        let mut level: u8 = 0;
        let mut data = PwmData::new();
        let mut base_pin = 0;

        data.pwm_levels = [0, 20, 40, 0, 80, 0, 160, 240];
        data.reflect();

        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME_US);
        alarm.enable_interrupt();

        (
            Shared {
                timer,
                alarm,
                out_pin,
                clock_pin,
                ratch_pin,
                led,
                tx,
                data,
            },
            Local {},
            init::Monotonics(),
        )
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [timer, alarm, tx, data, led],
        local = [tog: bool = true],
    )]
    fn timer_irq(mut c: timer_irq::Context) {
        (c.shared.data, c.shared.tx).lock(|data, tx| {
            for step in data.pwm_steps {
                tx.write(step.data << 24);
            }
        });

        // debug
        if *c.local.tog {
            c.shared.led.lock(|l| l.set_high().unwrap());
        } else {
            c.shared.led.lock(|l| l.set_low().unwrap());
        }
        *c.local.tog = !*c.local.tog;

        let mut alarm = c.shared.alarm;
        (alarm).lock(|a| {
            a.clear_interrupt();
            let _ = a.schedule(SCAN_TIME_US);
        });
    }
}
