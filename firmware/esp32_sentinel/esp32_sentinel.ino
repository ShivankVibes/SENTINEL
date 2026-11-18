/**
 * ==============================================================================
 * SENTINEL — Autonomous Rover Firmware (OPTION 2: PWM Speed via ENA & ENB)
 * Dual-Compatible: NodeMCU ESP8266-12E and ESP32
 * ==============================================================================
 */

#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
  typedef ESP8266WebServer WebServerType;
#else
  #include <WiFi.h>
  #include <WebServer.h>
  typedef WebServer WebServerType;
#endif

#include "config.h"

WebServerType server(HTTP_SERVER_PORT);

// State tracking
unsigned long lastCommandTime = 0;
bool isEmergencyStopped = false;
String currentAction = "IDLE";
int currentSpeed = 0;
float currentDistanceCm = -1.0;
unsigned long pulseEndTime = 0;

void setMotorSpeed(int speedLeft, int speedRight) {
  speedLeft = constrain(speedLeft, 0, 255);
  speedRight = constrain(speedRight, 0, 255);

  #if defined(ESP8266)
    analogWrite(MOTOR_LEFT_ENA, speedLeft);
    analogWrite(MOTOR_RIGHT_ENB, speedRight);
  #else
    analogWrite(MOTOR_LEFT_ENA, speedLeft);
    analogWrite(MOTOR_RIGHT_ENB, speedRight);
  #endif
}

void setupMotors() {
  pinMode(MOTOR_LEFT_ENA, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);

  pinMode(MOTOR_RIGHT_ENB, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);

  #if defined(ESP8266)
    analogWriteRange(255);  // Set standard 8-bit PWM (0-255)
  #endif

  stopMotors();
}

void setupSensors() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
}

void stopMotors() {
  setMotorSpeed(0, 0);
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  currentAction = "STOP";
}

void moveForward(int speed) {
  if (isEmergencyStopped) return;
  // Left Motor Forward
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  // Right Motor Forward
  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);

  setMotorSpeed(speed, speed);
  currentAction = "MOVE_FORWARD";
}

void turnLeft(int speed) {
  if (isEmergencyStopped) return;
  // Left Motor Reverse, Right Motor Forward
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);

  setMotorSpeed(speed, speed);
  currentAction = "TURN_LEFT";
}

void turnRight(int speed) {
  if (isEmergencyStopped) return;
  // Left Motor Forward, Right Motor Reverse
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, HIGH);

  setMotorSpeed(speed, speed);
  currentAction = "TURN_RIGHT";
}

float readUltrasonic() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 25000); // 25ms timeout (~4m max)
  if (duration == 0) return -1.0;
  return (duration * 0.0343) / 2.0;
}

// HTTP API Handlers
void handleCommand() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"status\":\"ERROR\",\"message\":\"Missing body\"}");
    return;
  }

  String body = server.arg("plain");
  int actionIdx = body.indexOf("\"action\":\"");
  if (actionIdx == -1) {
    server.send(400, "application/json", "{\"status\":\"ERROR\",\"message\":\"Invalid action\"}");
    return;
  }
  int actionEnd = body.indexOf("\"", actionIdx + 10);
  String action = body.substring(actionIdx + 10, actionEnd);

  int durIdx = body.indexOf("\"duration_ms\":");
  int duration = 250;
  if (durIdx != -1) {
    duration = body.substring(durIdx + 14).toInt();
  }

  int spdIdx = body.indexOf("\"speed\":");
  int speed = 180;
  if (spdIdx != -1) {
    speed = body.substring(spdIdx + 8).toInt();
  }

  lastCommandTime = millis();
  pulseEndTime = millis() + duration;

  if (action == "EMERGENCY_STOP") {
    isEmergencyStopped = true;
    stopMotors();
    server.send(200, "application/json", "{\"status\":\"OK\",\"message\":\"EMERGENCY_STOPPED\"}");
    return;
  }

  if (isEmergencyStopped && action != "RESET_EMERGENCY") {
    server.send(403, "application/json", "{\"status\":\"ERROR\",\"message\":\"Rover is in EMERGENCY_STOP mode\"}");
    return;
  }

  if (action == "RESET_EMERGENCY") {
    isEmergencyStopped = false;
    stopMotors();
    server.send(200, "application/json", "{\"status\":\"OK\",\"message\":\"Emergency cleared\"}");
    return;
  }

  if (action == "MOVE_FORWARD") {
    moveForward(speed);
  } else if (action == "TURN_LEFT") {
    turnLeft(speed);
  } else if (action == "TURN_RIGHT") {
    turnRight(speed);
  } else if (action == "STOP") {
    stopMotors();
  }

  server.send(200, "application/json", "{\"status\":\"OK\",\"action\":\"" + action + "\",\"speed\":" + String(speed) + "}");
}

void handleTelemetry() {
  currentDistanceCm = readUltrasonic();
  String json = "{";
  json += "\"rover_id\":\"SENTINEL-MK1\",";
  json += "\"is_connected\":true,";
  json += "\"current_state\":\"" + currentAction + "\",";
  json += "\"is_emergency_stop\":" + String(isEmergencyStopped ? "true" : "false") + ",";
  json += "\"ultrasonic_distance_cm\":" + String(currentDistanceCm, 1) + ",";
  json += "\"rssi_dbm\":" + String(WiFi.RSSI());
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[SENTINEL] Booting Rover Firmware (PWM Enabled)...");

  setupMotors();
  setupSensors();

  #if WIFI_MODE_AP
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print("[WiFi] Access Point started: ");
    Serial.println(AP_SSID);
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.softAPIP());
  #else
    WiFi.begin(STA_SSID, STA_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(300);
      Serial.print(".");
    }
    Serial.print("\n[WiFi] Connected. IP: ");
    Serial.println(WiFi.localIP());
  #endif

  server.on("/cmd", HTTP_POST, handleCommand);
  server.on("/telemetry", HTTP_GET, handleTelemetry);
  server.begin();
  Serial.println("[HTTP] Server listening.");
}

void loop() {
  server.handleClient();

  // Pulse duration timeout
  if (pulseEndTime > 0 && millis() >= pulseEndTime && currentAction != "STOP") {
    stopMotors();
    pulseEndTime = 0;
  }

  // Safety Watchdog
  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT_MS && currentAction != "STOP") {
    stopMotors();
  }
}
