#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Write};
use cortex_m::asm::delay;
use critical_section::Mutex;
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
use heapless::spsc::Queue;
use heapless::String;
use panic_probe as _;
use rp_pico::hal::{
    self,
    clocks::init_clocks_and_plls,
    entry,
    gpio::{self, FunctionI2C, Interrupt::EdgeLow, Pin},
    pac,
    pac::interrupt,
    Clock, Watchdog,
};

use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

type ClockwisePin = gpio::Pin<gpio::bank0::Gpio7, gpio::FunctionSioInput, gpio::PullUp>;
type AntiClockwisePin = gpio::Pin<gpio::bank0::Gpio8, gpio::FunctionSioInput, gpio::PullUp>;

type InputAndState = (ClockwisePin, AntiClockwisePin);

static GLOBAL_DATA: Mutex<RefCell<Option<InputAndState>>> = Mutex::new(RefCell::new(None));
static mut NUMBERS_Q: Queue<u8, 100> = Queue::new();

#[entry]
fn main() -> ! {
    let mut peripherals = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
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

    let mut led_pin = pins.gpio25.into_push_pull_output();

    // Rotation controls & associated interrupt
    let mut rot_cw = pins.gpio7.into_pull_up_input();
    let mut rot_ccw = pins.gpio8.into_pull_up_input();
    rot_cw.set_interrupt_enabled(EdgeLow, true);
    rot_ccw.set_interrupt_enabled(EdgeLow, true);

    let mut row_bt = pins.gpio9.into_pull_up_input();

    // Initializing display & text style etc.
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

    // Initializing global state
    critical_section::with(|cs| {
        GLOBAL_DATA.borrow(cs).replace(Some((rot_cw, rot_ccw)));
    });

    let (_, mut rx) = unsafe { NUMBERS_Q.split() };

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    // Show the running status
    let _ = led_pin.set_high();
    info!("Everything initialised");

    let mut number = None;

    loop {
        let mut screen_data = String::<3>::new();

        if let Some(n) = rx.dequeue() {
            number = Some(n)
        }
        let _ = match number {
            Some(n) => write!(screen_data, "{n}"),
            None => write!(screen_data, "NaN"),
        };
        let _ = display.clear(BinaryColor::Off);
        Text::with_baseline(&screen_data, Point::zero(), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        display.flush().unwrap();
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    static mut INPUTS_AND_STATE: Option<InputAndState> = None;
    static mut NUMBER: u8 = 128;

    let (mut tx, _) = unsafe { NUMBERS_Q.split() };

    if INPUTS_AND_STATE.is_none() {
        critical_section::with(|cs| {
            *INPUTS_AND_STATE = GLOBAL_DATA.borrow(cs).take();
        })
    }

    if let Some(state_stuff) = INPUTS_AND_STATE {
        let (rot_cw, rot_ccw) = state_stuff;

        if rot_cw.interrupt_status(EdgeLow) {
            if let Some(nn) = NUMBER.checked_add(1) {
                *NUMBER = nn;
            }
            info!("CW {:?}", NUMBER);
            rot_cw.clear_interrupt(EdgeLow);
            let _ = tx.enqueue(*NUMBER);
        } else if rot_ccw.interrupt_status(EdgeLow) {
            if let Some(nn) = NUMBER.checked_sub(1) {
                *NUMBER = nn;
            }

            info!("CCW {:?}", NUMBER);
            rot_ccw.clear_interrupt(EdgeLow);
            let _ = tx.enqueue(*NUMBER);
        }
    }
}
