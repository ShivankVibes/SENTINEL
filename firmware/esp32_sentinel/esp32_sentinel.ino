/**
 * ==============================================================================
 * SENTINEL — Autonomous Rover Firmware for HW-094
 * Direct 4-Wire Control (IN1, IN2, IN3, IN4) with PWM Speed Support
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

void setupMotors() {
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
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
  analogWrite(MOTOR_LEFT_IN1, 0);
  analogWrite(MOTOR_LEFT_IN2, 0);
  analogWrite(MOTOR_RIGHT_IN3, 0);
  analogWrite(MOTOR_RIGHT_IN4, 0);
  currentAction = "STOP";
}

void moveForward(int speed) {
  if (isEmergencyStopped) return;
  speed = constrain(speed, 0, 255);
  // Left Motor Forward: IN1 = PWM, IN2 = LOW
  analogWrite(MOTOR_LEFT_IN1, speed);
  analogWrite(MOTOR_LEFT_IN2, 0);
  // Right Motor Forward: IN3 = PWM, IN4 = LOW
  analogWrite(MOTOR_RIGHT_IN3, speed);
  analogWrite(MOTOR_RIGHT_IN4, 0);
  currentAction = "MOVE_FORWARD";
}

void turnLeft(int speed) {
  if (isEmergencyStopped) return;
  speed = constrain(speed, 0, 255);
  // Left Motor Reverse: IN1 = LOW, IN2 = PWM
  analogWrite(MOTOR_LEFT_IN1, 0);
  analogWrite(MOTOR_LEFT_IN2, speed);
  // Right Motor Forward: IN3 = PWM, IN4 = LOW
  analogWrite(MOTOR_RIGHT_IN3, speed);
  analogWrite(MOTOR_RIGHT_IN4, 0);
  currentAction = "TURN_LEFT";
}

void turnRight(int speed) {
  if (isEmergencyStopped) return;
  speed = constrain(speed, 0, 255);
  // Left Motor Forward: IN1 = PWM, IN2 = LOW
  analogWrite(MOTOR_LEFT_IN1, speed);
  analogWrite(MOTOR_LEFT_IN2, 0);
  // Right Motor Reverse: IN3 = LOW, IN4 = PWM
  analogWrite(MOTOR_RIGHT_IN3, 0);
  analogWrite(MOTOR_RIGHT_IN4, speed);
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
  String action = "";
  int duration = 250;
  int speed = 180;

  // 1. Check Query Parameters (e.g., /cmd?action=MOVE_FORWARD&speed=180)
  if (server.hasArg("action")) {
    action = server.arg("action");
    if (server.hasArg("speed")) speed = server.arg("speed").toInt();
    if (server.hasArg("duration_ms")) duration = server.arg("duration_ms").toInt();
  }
  // 2. Check JSON Body (supports arbitrary whitespace)
  else if (server.hasArg("plain")) {
    String body = server.arg("plain");

    int actKey = body.indexOf("\"action\"");
    if (actKey != -1) {
      int colon = body.indexOf(":", actKey);
      int q1 = body.indexOf("\"", colon);
      int q2 = body.indexOf("\"", q1 + 1);
      if (q1 != -1 && q2 != -1) {
        action = body.substring(q1 + 1, q2);
      }
    }

    int spdKey = body.indexOf("\"speed\"");
    if (spdKey != -1) {
      int colon = body.indexOf(":", spdKey);
      int end = body.indexOf(",", colon);
      if (end == -1) end = body.indexOf("}", colon);
      if (end != -1) speed = body.substring(colon + 1, end).toInt();
    }

    int durKey = body.indexOf("\"duration_ms\"");
    if (durKey != -1) {
      int colon = body.indexOf(":", durKey);
      int end = body.indexOf(",", colon);
      if (end == -1) end = body.indexOf("}", colon);
      if (end != -1) duration = body.substring(colon + 1, end).toInt();
    }
  }

  action.trim();
  if (action.length() == 0) {
    server.send(400, "application/json", "{\"status\":\"ERROR\",\"message\":\"Invalid action\"}");
    return;
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
  Serial.println("\n[SENTINEL] Booting Rover Firmware for HW-094...");

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
