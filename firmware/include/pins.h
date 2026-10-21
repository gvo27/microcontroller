#pragma once
// Pin map for ESP32-S3-N16R8 -- LEFT header (P1) only, see README.md.
// P1 order (USB ports at the bottom, chip facing you):
//   3V3 3V3 RST 4 5 6 7 15 16 17 18 8 3 46 9 10 11 12 13 14 5V G

// ---- Active (driven by this firmware) ----
constexpr int PIN_STEER_PWM    = 4;   // P1-4  -> DS3225 servo signal
constexpr int PIN_THROTTLE_PWM = 5;   // P1-5  -> THW-1060 ESC throttle signal

// ---- Reserved for future devices (NOT configured by this firmware) ----
constexpr int PIN_BATT_LOW     = 6;   // P1-6  battery hi/low digital in (ADC1_CH5 capable)
constexpr int PIN_IMU_INT      = 7;   // P1-7  6-axis IMU interrupt / data-ready
constexpr int PIN_ENC_A        = 15;  // P1-8  shaft encoder channel A (PCNT)
constexpr int PIN_ENC_B        = 16;  // P1-9  shaft encoder channel B (PCNT)
constexpr int PIN_SPARE_UART_TX = 17; // P1-10 spare (UART1 TX, alt. Pi link)
constexpr int PIN_SPARE_UART_RX = 18; // P1-11 spare (UART1 RX, alt. Pi link)
constexpr int PIN_I2C_SDA      = 8;   // P1-12 I2C SDA (IMU + LIDAR-Lite v4)
constexpr int PIN_I2C_SCL      = 9;   // P1-15 I2C SCL (IMU + LIDAR-Lite v4)
constexpr int PIN_DIST_TRIG    = 10;  // P1-16 ultrasonic TRIG / LIDAR GPIO
constexpr int PIN_DIST_ECHO    = 11;  // P1-17 ultrasonic ECHO (via divider) / LIDAR ready
// Free: 12 (P1-18), 13 (P1-19), 14 (P1-20)

// ---- USB-C ports ----
// Left  "USB": native USB (GPIO19/20) -> planned regulated power input only
// Right "COM": CH343 -> UART0 (GPIO43/44) -> flashing + host link (laptop now, Pi later)
// ESP32 powers no peripherals; sensors use a separate regulated rail, common ground.

// ---- Do not use ----
// GPIO3  (P1-13): strapping pin (JTAG signal source)
// GPIO46 (P1-14): strapping pin (boot mode / ROM log)
// GPIO35-37: wired to octal PSRAM inside the module (right header)
// GPIO19/20: native USB D-/D+ (right header); GPIO43/44: UART0 -> CH343 "COM" port
