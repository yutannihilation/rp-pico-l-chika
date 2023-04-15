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

use defmt::info;
use defmt_rtt as _;
// The macro for our start-up function
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// Import pio crates
use hal::pio::{PIOBuilder, Tx, ValidStateMachine};
use pio_proc::pio_file;

#[derive(Debug, Clone, Copy)]
struct PwmStep {
    length: u32,
    data: u32,
}

struct PwmData {
    pwm_levels: [u32; 8],
    pub pwm_steps: [PwmStep; 9],
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

fn pio_shift_out<T: ValidStateMachine>(tx: &mut Tx<T>, data: u32) {
    tx.write(data);
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then fades the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // Create a pio program
    let program = pio_file!("./src/shift_register.pio", select_program("shift_register"),);
    let installed = pio0.install(&program.program).unwrap();

    let pin1: hal::gpio::Pin<_, hal::gpio::FunctionPio0> = pins.gpio2.into_mode();
    let _pin2: hal::gpio::Pin<_, hal::gpio::FunctionPio0> = pins.gpio3.into_mode();
    let _pin3: hal::gpio::Pin<_, hal::gpio::FunctionPio0> = pins.gpio4.into_mode();
    let pin1_id = pin1.id().num;

    // Build the pio program and set pin both for set and side set!
    // We are running with the default divider which is 1 (max speed)
    let (mut sm, _, mut tx) = PIOBuilder::from_program(installed)
        .out_pins(pin1_id, 1)
        .side_set_pin_base(pin1_id + 1)
        .build(sm0);

    sm.set_pindirs([
        (pin1_id, hal::pio::PinDir::Output),
        (pin1_id + 1, hal::pio::PinDir::Output),
        (pin1_id + 2, hal::pio::PinDir::Output),
    ]);

    // Start state machine
    let _sm = sm.start();

    let mut level: u8 = 0;
    let mut data = PwmData::new();
    let mut base_pin = 0;

    loop {
        let mut data_tmp: [u32; 8] = [0; 8];
        let next_pin = (base_pin + 1) % 7;

        data_tmp[base_pin] = 255;
        data_tmp[next_pin] = 50;

        info!(
            "cur: {}, next: {}, data: {:?}",
            base_pin, next_pin, data_tmp
        );

        data.pwm_levels = data_tmp;
        data.reflect();

        for _ in 0..30 {
            for step in data.pwm_steps {
                pio_shift_out(&mut tx, step.data << 24);
                // info!("{}", level);
                delay.delay_us(step.length * 100);
            }
        }

        base_pin = next_pin;
    }
}
