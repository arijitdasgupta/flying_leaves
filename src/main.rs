#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::asm::delay;
use cortex_m_rt::entry;
use defmt::{info, println};
use defmt_rtt as _;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, iso_8859_7::FONT_8X13, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
};
use hal::fugit::RateExtU32;
use heapless::String;
use panic_probe as _;
use rp_pico::{
    hal::{
        self,
        clocks::init_clocks_and_plls,
        gpio::{FunctionI2C, Interrupt, Pin},
        Clock, Watchdog,
    },
    pac::{self, io_bank0::gpio::GPIO_STATUS},
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
    let mut rot_cw = pins.gpio7.into_pull_up_input();
    let mut rot_ccw = pins.gpio8.into_pull_up_input();
    let mut row_bt = pins.gpio9.into_pull_up_input();

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

    let mut number: u8 = 15;

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_8X13)
        .text_color(BinaryColor::On)
        .build();

    // Text::with_baseline("", Point::new(0, 16), text_style, Baseline::Top)
    //     .draw(&mut display)
    //     .unwrap();

    loop {
        if rot_cw.is_low().unwrap() {
            if let Some(n) = number.checked_add(1) {
                number = n;
            }
        } else if rot_ccw.is_low().unwrap() {
            if let Some(n) = number.checked_sub(1) {
                number = n;
            }
        } else if row_bt.is_low().unwrap() {
            number = 128;
        }

        let mut screen_data = String::<3>::new();
        let _ = write!(screen_data, "{number}");
        let _ = display.clear(BinaryColor::Off);
        Text::with_baseline(&screen_data, Point::zero(), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        timer.delay_us(100);
        display.flush().unwrap();
    }
}
