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
// Each pin corresponds to the following segment of the "8".
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
use hal::pio::{PIOBuilder, Running, StateMachine, Tx, ValidStateMachine, SM0};
use pio::{Instruction, InstructionOperands, OutDestination};
use pio_proc::pio_file;

fn pio_shift_register_set_output<T: ValidStateMachine>(tx: &mut Tx<T>, output: u8) {
    tx.write(output as u32);
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

    // Set gpio25 to pio
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

    // Set pio pindir for gpio25
    sm.set_pindirs([
        (pin1_id, hal::pio::PinDir::Output),
        (pin1_id + 1, hal::pio::PinDir::Output),
        (pin1_id + 2, hal::pio::PinDir::Output),
    ]);

    // Start state machine
    let sm = sm.start();

    // Loop forever and adjust duty cycle to make te led brighter
    let mut level = 0;
    loop {
        info!("Level = {}", level);
        pio_shift_register_set_output(&mut tx, level);
        level = level.wrapping_add(1);
        delay.delay_ms(250);
    }
}

// End of file
