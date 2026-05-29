// GPIO pin assignments for the Honda Megapro Programmable CDI hardware.
// One source of truth — every module reads pin numbers from here.
// ESP32 DOIT DevKit V1 (ESP32-WROOM-32).
#pragma once

#include <cstdint>
#include <driver/adc.h>

namespace cdi::pins {

// ---------- Pulser inputs (digital interrupt + ADC for scope mode) ----------
// Optocoupler collector tied to these via 10k pull-up to 3.3V.
// GPIO34/35 are input-only and have no internal pull-up.
constexpr uint8_t PULSER_CH1 = 34;  // leading edge of magnet
constexpr uint8_t PULSER_CH2 = 35;  // trailing edge of magnet

// ADC channel mapping (must use ADC1; ADC2 conflicts with WiFi)
constexpr adc1_channel_t PULSER_CH1_ADC = ADC1_CHANNEL_6;  // GPIO34
constexpr adc1_channel_t PULSER_CH2_ADC = ADC1_CHANNEL_7;  // GPIO35

// ---------- Spark output ----------
// Wiring used in this build is INDUCTIVE (TCI), NOT CDI:
//   GPIO25 → series resistor (220-1k Ω) → MOSFET/NPN gate → coil
//   primary → +12V battery.
// Behaviour: GPIO25 HIGH for dwell_us charges primary; LOW → flyback
// → spark fires on the FALLING edge. The scheduler holds HIGH for
// dwell_us then drops LOW (NOT a momentary HIGH pulse like SCR CDI).
//
// External 10 kΩ pull-down GPIO25→GND is STRONGLY recommended — the
// pin floats during ~50 ms of bootloader execution before main.cpp
// can drive it LOW, and a floating MOSFET gate can start charging
// the primary, then a sudden LOW transition fires an unwanted spark
// at every power-on. Software pinMode in setup() helps but doesn't
// cover the pre-setup window.
constexpr uint8_t SPARK_OUT = 25;

// ---------- Status LEDs ----------
// STATUS_LED: HIGH saat ada WS client connected, LOW saat tidak ada.
// (Bukan blinking heartbeat — sebelumnya direncanakan tapi tidak
// diimplementasi. Cukup useful as connection indicator.)
constexpr uint8_t STATUS_LED  = 2;
// MODE_LED: HIGH selama dwell (visible spark indicator).
constexpr uint8_t MODE_LED    = 26;
// SHIFT_LIGHT: HIGH/flashing saat RPM >= warn/shift threshold.
constexpr uint8_t SHIFT_LIGHT = 27;

// ---------- User input ----------
constexpr uint8_t BOOT_BTN     = 0;  // long-press 2s → SAFE_HOLD (panic kill)
constexpr uint8_t LAUNCH_INPUT = 13; // clutch pressed → 2-step active
constexpr uint8_t QUICKSHIFTER = 14; // shift sensor → cut ignition

// ---------- Analog sense ----------
// VBAT divider 1:4 dari 12V supply ke ADC pin (0-3.3V range).
// Dipakai oleh ALVP untuk derate/disarm pada under-voltage.
constexpr uint8_t VBAT_SENSE = 32;  // ADC1_CH4
constexpr adc1_channel_t VBAT_SENSE_ADC = ADC1_CHANNEL_4;

// ---------- Voltage divider ratios (for ALVP / gear sense conversion) ----------
constexpr float VBAT_DIVIDER_RATIO = 4.0f;   // 12V → 3V at ADC pin
// DB_11 attenuation full-scale on ESP32 is ~3.55 V (NOT 3.3 V). Using 3.3
// here under-read vbat by ~8% and saturated at 13.2 V (audit M4). 3.55 is
// the nominal DB_11 FS; for per-chip accuracy a future change should use
// the eFuse-calibrated analogReadMilliVolts()/esp_adc_cal path.
constexpr float ADC_VREF_V         = 3.55f;
constexpr uint16_t ADC_FULL_SCALE  = 4095;

} // namespace cdi::pins
