# Methane Gas Detector — ESP32-C6 + SSD1306 OLED

A real-time methane gas detection system built with Rust on the ESP32-C6 microcontroller. The system reads an MQ-2 gas sensor, displays live gas levels on an SSD1306 OLED screen, and activates a buzzer and LED alarm when unsafe concentrations are detected.

---

## Hardware

| Component | Purpose |
|---|---|
| ESP32-C6 DevKitC-1 | Main microcontroller (RISC-V, ESP-IDF) |
| SSD1306 OLED 128x64 | Displays gas level, raw ADC, status bar |
| MQ-2 Gas Sensor | Detects methane, LPG, propane, smoke |
| Buzzer (active) | Audio alarm |
| Red LED + 330Ω resistor | Visual alarm |

---

## Wiring

### SSD1306 OLED (I2C — 4 pins only)

| OLED | ESP32-C6 |
|---|---|
| GND | GND |
| VCC | 3.3V |
| SCL | GPIO7 |
| SDA | GPIO6 |

### MQ-2 Gas Sensor

| MQ-2 | ESP32-C6 | Note |
|---|---|---|
| VCC | 5V | Heater requires 5V |
| GND | GND | |
| AOUT | GPIO2 | Analog output |
| DOUT | — | Not used |

### Buzzer

| Pin | ESP32-C6 |
|---|---|
| + | GPIO10 |
| − | GND |

### LED

| Pin | ESP32-C6 |
|---|---|
| Anode | GPIO10 → 330Ω → A |
| Cathode | GND |

> **Why GPIO10/11 and not GPIO4/5?**
> GPIO4 and GPIO5 are JTAG pins (MTMS/MTDI) on the ESP32-C6. Using them as plain
> output PinDrivers fails silently because the debug subsystem holds those pads.
> GPIO10 and GPIO11 are safe general-purpose outputs.

---

## Alarm Logic

| ADC Raw Value | Status | Buzzer | LED |
|---|---|---|---|
| < 1500 | SAFE | Off | Off |
| 1500 – 2499 | WARNING | Intermittent | Blink |
| ≥ 2500 | DANGER | Continuous | On |

---

## Calibration

The MQ-2 requires a 60-second warm-up after power-on. In clean air it typically reads 300–600 raw ADC counts. Adjust `SAFE_THRESHOLD` and `WARNING_THRESHOLD` in `main.rs` to suit your environment.

```rust
const SAFE_THRESHOLD: u16    = 1500;
const WARNING_THRESHOLD: u16 = 2500;
```

---

## Build & Flash

### Prerequisites

```bash
# Install Espressif toolchain (needed even for RISC-V C6)
cargo install espup
espup install
source ~/export-esp.sh

# Install linker helper and flash tool
cargo install ldproxy
cargo install espflash
```

### Build

```bash
cargo build --release
```

### Flash

```bash
espflash flash --monitor target/riscv32imac-esp-espidf/release/methane_c6
```

---

## Dependencies

| Crate | Version | Purpose |
|---|---|---|
| esp-idf-svc | 0.52 | ESP-IDF services + HAL re-export |
| esp-idf-sys | 0.37 | ESP-IDF raw bindings + entry point |
| ssd1306 | 0.8 | OLED display driver |
| embedded-graphics | 0.8 | Text and shape rendering |
| heapless | 0.8 | Stack-allocated strings |
| anyhow | 1 | Error handling |
| esp-println | 0.13 | Serial logging |

---
```
