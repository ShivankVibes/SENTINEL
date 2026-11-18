#ifndef SENTINEL_CONFIG_H
#define SENTINEL_CONFIG_H

/**
 * ==============================================================================
 * SENTINEL ROVER HARDWARE CONFIGURATION — OPTION 2 (HARDWARE PWM VIA ENA/ENB)
 * Supports both ESP8266 (NodeMCU 12E) and ESP32
 * ==============================================================================
 */

// --- Wi-Fi Configuration ---
#define WIFI_MODE_AP true                // true: creates Access Point, false: connects to local Wi-Fi
#define AP_SSID       "SENTINEL_ROVER"
#define AP_PASSWORD   "sentinel123"

#define STA_SSID      "YOUR_WIFI_SSID"
#define STA_PASSWORD  "YOUR_WIFI_PASSWORD"

#define HTTP_SERVER_PORT 8080

// --- Safety & Watchdog ---
#define WATCHDOG_TIMEOUT_MS 500          // Auto-stop motors if no command received within 500ms

#if defined(ESP8266)
  // ============================================================================
  // NodeMCU ESP8266-12E (OPTION 2: Hardware PWM Enabled)
  // ============================================================================
  // Left Motor (Motor A)
  #define MOTOR_LEFT_ENA      0   // NodeMCU Pin D3 (PWM Speed Control 0-255)
  #define MOTOR_LEFT_IN1      5   // NodeMCU Pin D1 (Direction Fwd)
  #define MOTOR_LEFT_IN2      4   // NodeMCU Pin D2 (Direction Rev)

  // Right Motor (Motor B)
  #define MOTOR_RIGHT_ENB     2   // NodeMCU Pin D4 (PWM Speed Control 0-255)
  #define MOTOR_RIGHT_IN3    14   // NodeMCU Pin D5 (Direction Fwd)
  #define MOTOR_RIGHT_IN4    12   // NodeMCU Pin D6 (Direction Rev)

  // Ultrasonic Sensor
  #define ULTRASONIC_TRIG_PIN 13  // NodeMCU Pin D7
  #define ULTRASONIC_ECHO_PIN 15  // NodeMCU Pin D8

#else
  // ============================================================================
  // ESP32-12E / Dev Module (OPTION 2: Hardware PWM Enabled)
  // ============================================================================
  // Left Motor (Motor A)
  #define MOTOR_LEFT_ENA     25   // ESP32 Pin D25 (PWM Speed Control 0-255)
  #define MOTOR_LEFT_IN1     16   // ESP32 Pin RX2 / D16 (Direction Fwd)
  #define MOTOR_LEFT_IN2     17   // ESP32 Pin TX2 / D17 (Direction Rev)

  // Right Motor (Motor B)
  #define MOTOR_RIGHT_ENB    26   // ESP32 Pin D26 (PWM Speed Control 0-255)
  #define MOTOR_RIGHT_IN3    18   // ESP32 Pin D18 (Direction Fwd)
  #define MOTOR_RIGHT_IN4    19   // ESP32 Pin D19 (Direction Rev)

  // Ultrasonic Sensor
  #define ULTRASONIC_TRIG_PIN 22  // ESP32 Pin D22
  #define ULTRASONIC_ECHO_PIN 23  // ESP32 Pin D23
#endif

#endif // SENTINEL_CONFIG_H
