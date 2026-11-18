#ifndef SENTINEL_CONFIG_H
#define SENTINEL_CONFIG_H

/**
 * ==============================================================================
 * SENTINEL ESP32-12E HARDWARE CONFIGURATION
 * ==============================================================================
 * IMPORTANT:
 * 1. Do NOT connect or drive motors until your HW-094 motor driver board model
 *    and exact pinout are physically verified with a multimeter / datasheet.
 * 2. ESP32 GPIOs operate at 3.3V logic. Ultrasonic Echo pins may output 5V
 *    unless using a 3.3V-compatible sensor (like HC-SR04P or voltage divider).
 * ==============================================================================
 */

// --- Wi-Fi Configuration ---
#define WIFI_MODE_AP true                // true: ESP32 creates own AP, false: connect to local Wi-Fi
#define AP_SSID       "SENTINEL_ROVER"
#define AP_PASSWORD   "sentinel123"

#define STA_SSID      "YOUR_WIFI_SSID"
#define STA_PASSWORD  "YOUR_WIFI_PASSWORD"

#define HTTP_SERVER_PORT 8080

// --- Safety & Watchdog ---
#define WATCHDOG_TIMEOUT_MS 500          // Auto-stop motors if no command received within 500ms

// --- Motor Driver Pins (Placeholders - Verify HW-094 board before connecting) ---
// HW-094 is typically a dual H-bridge (L298N mini or similar).
// These GPIOs are safe defaults on ESP32-12E (avoiding bootstrap pins GPIO0, 2, 12, 15).
#define MOTOR_LEFT_IN1    16
#define MOTOR_LEFT_IN2    17
#define MOTOR_RIGHT_IN3   18
#define MOTOR_RIGHT_IN4   19

// PWM Channels (ESP32 LEDC)
#define PWM_FREQ          1000
#define PWM_RESOLUTION    8
#define PWM_CHAN_LEFT     0
#define PWM_CHAN_RIGHT    1

// --- Ultrasonic Sensor Pins (Verify sensor model before connecting) ---
#define ULTRASONIC_TRIG_PIN 22
#define ULTRASONIC_ECHO_PIN 23

#endif // SENTINEL_CONFIG_H
