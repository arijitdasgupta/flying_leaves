#![no_std]
#![no_main]

use core::module_path;
use cortex_m_rt::entry;
use defmt::{info, println};
use defmt_rtt as _;
use embedded_hal::{delay::DelayNs, digital::OutputPin};
use panic_probe as _;
use rp_pico::{
    hal,
    hal::{clocks::init_clocks_and_plls, Clock, Watchdog},
    pac,
};

#[entry]
fn main() -> ! {
    println!("Program start");
    let mut peripherals = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(peripherals.WATCHDOG);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        peripherals.XOSC,
        peripherals.CLOCKS,
        peripherals.PLL_SYS,
        peripherals.PLL_USB,
        &mut peripherals.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(peripherals.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

    let mut timer = hal::Timer::new(peripherals.TIMER, &mut peripherals.RESETS, &clocks);

    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();

    loop {
        timer.delay_ms(1000);
        led_pin.set_high();
        timer.delay_ms(100);
        led_pin.set_low();
    }
}
