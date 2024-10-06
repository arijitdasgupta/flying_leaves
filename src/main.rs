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
use rp_pico::{
    hal::{
        self,
        clocks::{init_clocks_and_plls, ClocksManager},
        entry,
        gpio::{self, FunctionI2C, Interrupt::EdgeLow, Pin},
        multicore::{Multicore, Stack},
        pac::{self, interrupt},
        sio::{SioFifo, Spinlock0},
        Clock, Sio, Timer, Watchdog,
    },
    Pins,
};
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

type ClockwisePin = gpio::Pin<gpio::bank0::Gpio7, gpio::FunctionSioInput, gpio::PullUp>;
type AntiClockwisePin = gpio::Pin<gpio::bank0::Gpio8, gpio::FunctionSioInput, gpio::PullUp>;
type ButtonPin = gpio::Pin<gpio::bank0::Gpio9, gpio::FunctionSioInput, gpio::PullUp>;

type SharedPins<'a> = (ClockwisePin, AntiClockwisePin, ButtonPin);
// Shared pins and SIO between CORE0 and IRQ. Sync between them is controlled by
// cortex_m::critical_section & Mutex.
static SHARED_PINS: Mutex<RefCell<Option<SharedPins>>> = Mutex::new(RefCell::new(None));

static mut SHARED_STATE: u32 = 0;

// Stack for core1
static mut CORE1_STACK: Stack<8192> = Stack::new();

// Queue to communicate between IRQ and core1
static mut IRQ_Q: Queue<u32, 100> = Queue::new();

fn core1_task(clocks: ClocksManager) -> ! {
    let mut peripherals = unsafe { pac::Peripherals::steal() };
    let mut sio = hal::Sio::new(peripherals.SIO);

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

    // Initializing display interface, display & text style etc.
    let i2c = hal::I2C::i2c0(
        peripherals.I2C0,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        1.MHz(),
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

    // Status pin
    let mut led_pin = pins.gpio25.into_push_pull_output();

    // Rotation controls & associated interrupt
    let rot_cw = pins.gpio7.into_pull_up_input();
    let rot_ccw = pins.gpio8.into_pull_up_input();
    let rot_bt = pins.gpio9.into_pull_up_input();
    rot_cw.set_interrupt_enabled(EdgeLow, true);
    rot_ccw.set_interrupt_enabled(EdgeLow, true);
    rot_bt.set_interrupt_enabled(EdgeLow, true);

    // Initializing shared pins between main and interrupts
    critical_section::with(|cs| {
        SHARED_PINS
            .borrow(cs)
            .replace(Some((rot_cw, rot_ccw, rot_bt)));
    });

    let (_, mut rx) = unsafe { IRQ_Q.split() };

    // Enabling interrupts
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    info!("inited core1 & IO interrupts");
    let _ = led_pin.set_high();

    // Local state
    let mut number = None;
    let mut number2 = None;

    loop {
        let mut screen_data = String::<4>::new();
        let mut screen_data_2 = String::<4>::new();
        let mut screen_data_3 = String::<4>::new();

        if let Some(n) = rx.dequeue() {
            number = Some(n)
        }
        let _ = match number {
            Some(n) => write!(screen_data, "{n}"),
            None => write!(screen_data, "NaN"),
        };

        if let Some(n) = sio.fifo.read() {
            number2 = Some(n)
        }
        let _ = match number2 {
            Some(n) => write!(screen_data_2, "{n}"),
            None => write!(screen_data_2, "NaN"),
        };

        Spinlock0::claim();
        let state = unsafe { SHARED_STATE };
        let _ = write!(screen_data_3, "{state}");

        // unsafe { Spinlock0::release() };

        let _ = display.clear(BinaryColor::Off);
        Text::with_baseline(&screen_data, Point::zero(), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline(&screen_data_2, Point::new(0, 15), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline(&screen_data_3, Point::new(0, 30), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        display.flush().unwrap();
    }
}

#[entry]
fn main() -> ! {
    // Core stuff & timers
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
    let mut timer = Timer::new(peripherals.TIMER, &mut peripherals.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let mut sio = hal::Sio::new(peripherals.SIO);

    // Initiazling second-core
    let mut mc = Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || core1_task(clocks));

    info!("inited core0");

    let mut counter = 0;
    loop {
        sio.fifo.write(counter);
        timer.delay_ms(1000);
        counter += 1;
    }
}

// This interrupt is called from core1
#[interrupt]
fn IO_IRQ_BANK0() {
    static mut INPUTS_AND_STATE: Option<SharedPins> = None;
    static mut NUMBER: u32 = 128;

    let (mut tx, _) = unsafe { IRQ_Q.split() };

    if INPUTS_AND_STATE.is_none() {
        critical_section::with(|cs| {
            *INPUTS_AND_STATE = SHARED_PINS.borrow(cs).take();
        })
    }

    if let Some(state_stuff) = INPUTS_AND_STATE {
        let (rot_cw, rot_ccw, rot_bt) = state_stuff;

        if rot_cw.interrupt_status(EdgeLow) {
            *NUMBER = (*NUMBER + 1).min(9999);
            let _ = tx.enqueue(*NUMBER);
            info!("CW");
            rot_cw.clear_interrupt(EdgeLow);
        } else if rot_ccw.interrupt_status(EdgeLow) {
            if let Some(nn) = NUMBER.checked_sub(1) {
                *NUMBER = nn;
            }
            let _ = tx.enqueue(*NUMBER);
            info!("CCW");
            rot_ccw.clear_interrupt(EdgeLow);
        } else if rot_bt.interrupt_status(EdgeLow) {
            *NUMBER = 9900;
            let _ = tx.enqueue(*NUMBER);
            info!("BUTTON");
            rot_bt.clear_interrupt(EdgeLow);
        }

        let _lock = Spinlock0::claim();
        unsafe {
            SHARED_STATE = (SHARED_STATE + 1).min(9999);
        }
    }
}
