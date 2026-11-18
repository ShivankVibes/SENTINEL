#ifndef SENTINEL_CONFIG_H
#define SENTINEL_CONFIG_H

/**
 * ==============================================================================
 * SENTINEL ROVER HARDWARE CONFIGURATION
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
  // ESP8266 NodeMCU 12E Pin Mapping
  // ============================================================================
  // Safe GPIOs that do not block ESP8266 boot:
  // D1 = GPIO 5, D2 = GPIO 4, D5 = GPIO 14, D6 = GPIO 12, D7 = GPIO 13, D8 = GPIO 15
  #define MOTOR_LEFT_IN1      5   // NodeMCU Pin D1 (Left Forward)
  #define MOTOR_LEFT_IN2      4   // NodeMCU Pin D2 (Left Reverse)
  #define MOTOR_RIGHT_IN3    14   // NodeMCU Pin D5 (Right Forward)
  #define MOTOR_RIGHT_IN4    12   // NodeMCU Pin D6 (Right Reverse)

  // Ultrasonic Sensor
  #define ULTRASONIC_TRIG_PIN 13  // NodeMCU Pin D7
  #define ULTRASONIC_ECHO_PIN 15  // NodeMCU Pin D8 (Note: idle LOW is safe for boot)

#else
  // ============================================================================
  // ESP32-12E / ESP32 Dev Module Pin Mapping
  // ============================================================================
  #define MOTOR_LEFT_IN1     16   // ESP32 Pin RX2 / D16 (Left Forward)
  #define MOTOR_LEFT_IN2     17   // ESP32 Pin TX2 / D17 (Left Reverse)
  #define MOTOR_RIGHT_IN3    18   // ESP32 Pin D18 (Right Forward)
  #define MOTOR_RIGHT_IN4    19   // ESP32 Pin D19 (Right Reverse)

  // Ultrasonic Sensor
  #define ULTRASONIC_TRIG_PIN 22  // ESP32 Pin D22
  #define ULTRASONIC_ECHO_PIN 23  // ESP32 Pin D23
#endif

#endif // SENTINEL_CONFIG_H
