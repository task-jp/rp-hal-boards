//! Example of graphics on the LCD of the Waveshare RP2040-LCD-1.28
//!
//! Draws a red and green line with a blue rectangle.
//! After that it fills the screen line for line, at the end it starts over with
//! another colour, RED, GREEN and BLUE.
#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use embedded_graphics::primitives::Line;
use fugit::RateExtU32;
use panic_halt as _;

use waveshare_rp2040_lcd_1_28::entry;
use waveshare_rp2040_lcd_1_28::{
    hal::{
        self,
        clocks::{init_clocks_and_plls, Clock},
        pac,
        pio::PIOExt,
        watchdog::Watchdog,
        Sio,
    },
    Pins, XOSC_CRYSTAL_FREQ,
};

use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle},
};
use display_interface_spi::SPIInterface;
use gc9a01a::GC9A01A;

const LCD_WIDTH: u32 = 240;
const LCD_HEIGHT: u32 = 240;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up the delay for the first core.
    let sys_freq = clocks.system_clock.freq().to_Hz();
    let mut delay = Delay::new(core.SYST, sys_freq);

    let (mut _pio, _sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // https://www.waveshare.com/wiki/RP2040-LCD-1.28
    // GC9A01A LCD
    let lcd_dc = pins.gp8.into_push_pull_output();
    let lcd_cs = pins.gp9.into_push_pull_output();
    let lcd_clk = pins.gp10.into_function::<hal::gpio::FunctionSpi>();
    let lcd_mosi = pins.gp11.into_function::<hal::gpio::FunctionSpi>();
    let lcd_rst = pins
        .gp12
        .into_push_pull_output_in_state(hal::gpio::PinState::High);
    let mut _lcd_bl = pins
        .gp25
        .into_push_pull_output_in_state(hal::gpio::PinState::High);

        // spi::Spi
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI1, (lcd_mosi, lcd_clk));

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        8_000_000u32.MHz(),
        embedded_hal::spi::MODE_0,
    );

    let spi_interface = SPIInterface::new(spi, lcd_dc, lcd_cs);

     // initialize PWM for backlight
     let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

     // Configure PWM6
     let mut pwm = pwm_slices.pwm6;
     pwm.set_ph_correct();
     pwm.enable();
 
     // Output channel B on PWM6 to GPIO 13
     let mut channel = pwm.channel_b;
     channel.output_to(pins.gp13);
 
    // LCD is a 65K IPS LCD 240x240, color order is RGB.
    // LCD controller can correct this by settings the order bit (bit 3) in MADCTL register.
    // Also the colours are inverted, LCD controller can also correct this by writing to INVON register with no paramters.
    // All this is handled by the GC9A01A crate.
    let mut display = GC9A01A::new(spi_interface, lcd_rst, channel);

    display.reset(&mut delay).unwrap();
    // Initialize registers
    display.initialize(&mut delay).unwrap();
    // Fill screen with single color
    display.clear(Rgb565::CSS_FOREST_GREEN).unwrap();
    // Turn on backlight
    display.set_backlight(55000);


    let lcd_zero = Point::zero();
    let lcd_max_corner = Point::new((LCD_WIDTH - 1) as i32, (LCD_HEIGHT - 1) as i32);

    let style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLACK)
        .build();

    Rectangle::with_corners(
        Point::new((LCD_WIDTH / 3) as i32, (LCD_HEIGHT / 3) as i32),
        Point::new((LCD_WIDTH / 3 * 2) as i32, (LCD_HEIGHT / 3 * 2) as i32),
    )
    .into_styled(style)
    .draw(&mut display)
    .unwrap();

    Line::new(lcd_zero, lcd_max_corner)
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 1))
        .draw(&mut display)
        .unwrap();

    Line::new(
        Point::new(0, (LCD_HEIGHT - 1) as i32),
        Point::new((LCD_WIDTH - 1) as i32, 0),
    )
    .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1))
    .draw(&mut display)
    .unwrap();

    // Infinite colour wheel loop
    let mut l: i32 = 0;
    let mut c = Rgb565::RED;
    loop {
        Line::new(Point::new(0, l), Point::new((LCD_WIDTH - 1) as i32, l))
            .into_styled(PrimitiveStyle::with_stroke(c, 1))
            .draw(&mut display)
            .unwrap();
        delay.delay_ms(10);
        l += 1;
        if l == LCD_HEIGHT as i32 {
            l = 0;
            c = match c {
                Rgb565::RED => Rgb565::GREEN,
                Rgb565::GREEN => Rgb565::BLUE,
                _ => Rgb565::RED,
            }
        }
    }
}
