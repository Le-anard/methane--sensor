// src/main.rs
// =============================================================
//  Methane Gas Detector — ESP32-C6 + SSD1306 OLED (I2C)
// =============================================================
//  Pin map:
//   SSD1306 OLED
//     GND  → GND
//     VCC  → 3.3 V
//     SDA  → GPIO6   (I2C0 data)
//     SCL  → GPIO7   (I2C0 clock)
//
//   MQ-2 gas sensor
//     VCC  → 5 V
//     GND  → GND
//     AOUT → GPIO2   (ADC1 channel 2)
//
//   ⚠️  GPIO4 & GPIO5 are JTAG pins on ESP32-C6 — cannot be plain outputs.
//   Buzzer → GPIO10  (active-high; add NPN transistor for load)
//   LED    → GPIO11  (active-high; 330 Ω series resistor)
// =============================================================

use anyhow::Result;
use core::fmt::Write as FmtWrite;

use esp_idf_svc::{
    hal::{
        adc::{
            attenuation::DB_12,
            oneshot::{config::AdcChannelConfig, AdcChannelDriver, AdcDriver},
        },
        delay::FreeRtos,
        gpio::PinDriver,
        i2c::{I2cConfig, I2cDriver},
        peripherals::Peripherals,
        units::FromValueType,
    },
    log::EspLogger,
};

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, ascii::FONT_9X18_BOLD, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::{Baseline, Text},
};
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

const SAFE_THRESHOLD: u16    = 400;
const WARNING_THRESHOLD: u16 = 550;
const PPM_SCALE: f32 = 0.244;
const FILTER_WINDOW: usize = 8;
const LOOP_DELAY_MS: u32 = 500;

#[derive(Debug, Clone, Copy, PartialEq)]
enum GasStatus { Safe, Warning, Danger }

impl GasStatus {
    fn from_raw(raw: u16) -> Self {
        match raw {
            r if r < SAFE_THRESHOLD    => GasStatus::Safe,
            r if r < WARNING_THRESHOLD => GasStatus::Warning,
            _                          => GasStatus::Danger,
        }
    }
    fn label(&self) -> &'static str {
        match self {
            GasStatus::Safe    => "SAFE",
            GasStatus::Warning => "WARNING",
            GasStatus::Danger  => "DANGER",
        }
    }
}

struct MovingAverage {
    buf: [u16; FILTER_WINDOW],
    idx: usize,
    filled: bool,
}

impl MovingAverage {
    const fn new() -> Self {
        Self { buf: [0u16; FILTER_WINDOW], idx: 0, filled: false }
    }
    fn update(&mut self, sample: u16) -> u16 {
        self.buf[self.idx] = sample;
        self.idx = (self.idx + 1) % FILTER_WINDOW;
        if self.idx == 0 { self.filled = true; }
        let len = if self.filled { FILTER_WINDOW } else { self.idx.max(1) };
        let sum: u32 = self.buf[..len].iter().map(|&v| v as u32).sum();
        (sum / len as u32) as u16
    }
}

type OledDisplay<'a> = Ssd1306<
    I2CInterface<I2cDriver<'a>>,
    DisplaySize128x64,
    BufferedGraphicsMode<DisplaySize128x64>,
>;

fn draw_screen(display: &mut OledDisplay<'_>, ppm: f32, raw: u16, status: GasStatus) {
    display.clear(BinaryColor::Off).unwrap();

    let bold = MonoTextStyleBuilder::new()
        .font(&FONT_9X18_BOLD).text_color(BinaryColor::On).build();
    Text::with_baseline("Methane Detect", Point::new(0, 0), bold, Baseline::Top)
        .draw(display).unwrap();

    Rectangle::new(Point::new(0, 18), Size::new(128, 1))
        .into_styled(PrimitiveStyleBuilder::new().fill_color(BinaryColor::On).build())
        .draw(display).unwrap();

    let small = MonoTextStyleBuilder::new()
        .font(&FONT_6X10).text_color(BinaryColor::On).build();

    let mut s: heapless::String<32> = heapless::String::new();
    write!(&mut s, "Gas: {:.0} ppm", ppm).unwrap();
    Text::with_baseline(s.as_str(), Point::new(0, 22), small, Baseline::Top)
        .draw(display).unwrap();

    let mut r: heapless::String<32> = heapless::String::new();
    write!(&mut r, "Raw ADC: {}", raw).unwrap();
    Text::with_baseline(r.as_str(), Point::new(0, 34), small, Baseline::Top)
        .draw(display).unwrap();

    let (fg, bg) = match status {
        GasStatus::Safe    => (BinaryColor::On,  BinaryColor::Off),
        GasStatus::Warning => (BinaryColor::Off, BinaryColor::On),
        GasStatus::Danger  => (BinaryColor::Off, BinaryColor::On),
    };
    if bg == BinaryColor::On {
        Rectangle::new(Point::new(0, 46), Size::new(128, 12))
            .into_styled(PrimitiveStyleBuilder::new().fill_color(BinaryColor::On).build())
            .draw(display).unwrap();
    }

    let status_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10).text_color(fg).build();
    let mut st: heapless::String<20> = heapless::String::new();
    write!(&mut st, "Status: {}", status.label()).unwrap();
    Text::with_baseline(st.as_str(), Point::new(2, 46), status_style, Baseline::Top)
        .draw(display).unwrap();

    let bar_w = ((raw as u32 * 128) / 4095).min(128) as u32;
    if bar_w > 0 {
        Rectangle::new(Point::new(0, 60), Size::new(bar_w, 4))
            .into_styled(PrimitiveStyleBuilder::new().fill_color(BinaryColor::On).build())
            .draw(display).unwrap();
    }
    Rectangle::new(Point::new(0, 60), Size::new(128, 4))
        .into_styled(PrimitiveStyleBuilder::new()
            .stroke_color(BinaryColor::On).stroke_width(1).build())
        .draw(display).unwrap();
}

fn main() -> Result<()> {
    EspLogger::initialize_default();
    log::info!("=== Methane Detector (ESP32-C6) starting ===");

    let peripherals = Peripherals::take()?;

    // I2C → SSD1306 OLED (GPIO6=SDA, GPIO7=SCL)
    let i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio6,
        peripherals.pins.gpio7,
        &I2cConfig::new().baudrate(400_u32.kHz().into()),
    )?;
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().expect("SSD1306 init failed");
    display.flush().unwrap();
    log::info!("OLED display ready");

    // ADC → MQ-2 sensor (GPIO2)
    let adc_driver = AdcDriver::new(peripherals.adc1)?;
    let adc_channel_cfg = AdcChannelConfig {
        attenuation: DB_12,
        ..Default::default()
    };
    let mut adc_pin = AdcChannelDriver::new(
        &adc_driver,
        peripherals.pins.gpio2,
        &adc_channel_cfg,
    )?;
    log::info!("ADC initialised on GPIO2");

    // GPIO outputs — GPIO10=buzzer, GPIO11=LED
    // (GPIO4/5 are JTAG MTMS/MTDI on ESP32-C6, cannot be plain outputs)
    let mut buzzer = PinDriver::output(peripherals.pins.gpio10)?;
    let mut led    = PinDriver::output(peripherals.pins.gpio11)?;
    buzzer.set_low()?;
    led.set_low()?;
    log::info!("GPIO outputs ready (buzzer=GPIO10, led=GPIO11)");

    let mut filter    = MovingAverage::new();
    let mut peak_raw  = 0u16;
    let mut loop_tick = 0u32;
    log::info!("Entering main loop…");

    loop {
        loop_tick = loop_tick.wrapping_add(1);

        let raw_sample: u16 = adc_pin.read().unwrap_or(0);
        let raw = filter.update(raw_sample);
        let ppm = raw as f32 * PPM_SCALE;
        let status = GasStatus::from_raw(raw);
        if raw > peak_raw { peak_raw = raw; }

        match status {
            GasStatus::Safe => { buzzer.set_low()?; led.set_low()?; }
            GasStatus::Warning => {
                if loop_tick % 2 == 0 { buzzer.set_high()?; led.set_high()?; }
                else                  { buzzer.set_low()?;  led.set_low()?;  }
            }
            GasStatus::Danger => { buzzer.set_high()?; led.set_high()?; }
        }

        draw_screen(&mut display, ppm, raw, status);
        display.flush().unwrap();

        log::info!(
            "[{}] raw={} filtered={} ppm={:.0} status={:?} peak={}",
            loop_tick, raw_sample, raw, ppm, status, peak_raw
        );

        FreeRtos::delay_ms(LOOP_DELAY_MS);
    }
}