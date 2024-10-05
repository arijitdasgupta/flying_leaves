#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::{info, println};
use defmt_rtt as _;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, iso_8859_7::FONT_8X13, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::{delay::DelayNs, digital::OutputPin};
use hal::fugit::RateExtU32;
use panic_probe as _;
use rp_pico::{
    hal::{
        self,
        clocks::init_clocks_and_plls,
        gpio::{FunctionI2C, Pin},
        Clock, Watchdog,
    },
    pac,
};
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

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
    // Configure two pins as being I²C, not GPIO
    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio4.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio5.reconfigure();

    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();

    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};
    let mut i2c = hal::I2C::i2c0(
        peripherals.I2C0,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut peripherals.RESETS,
        &clocks.system_clock,
    );
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate90)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_8X13)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hi\nWorld", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    // Text::with_baseline("", Point::new(0, 16), text_style, Baseline::Top)
    //     .draw(&mut display)
    //     .unwrap();

    display.flush().unwrap();

    loop {
        timer.delay_ms(1000);
        let _ = led_pin.set_high();
        timer.delay_ms(100);
        let _ = led_pin.set_low();
    }
}
