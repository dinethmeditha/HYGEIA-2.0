// Key fixes applied:
// 1. Set WiFi channel explicitly for ESP-NOW compatibility
// 2. Use insecure client for Telegram (certificate issues)
// 3. Add timeout for NTP sync
// 4. Improved ESP-NOW channel management
// 5. Better error handling

#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

#define VIBRATION_PIN 5
#define ONE_WIRE_BUS 4
#define REPORTING_PERIOD_MS 1000
#define LED_PIN 2
#define ESPNOW_CHANNEL 1  // Fixed channel for ESP-NOW

// OLED Display setup
#define i2c_Address 0x3c
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MAX30102 Algorithm variables
MAX30105 particleSensor;
const byte RATE_SIZE = 8;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
int32_t beatsPerMinute;
int8_t beatAvg;

// SpO2 buffer management
#define MAX_SPO2_SAMPLES 100
uint32_t irBuffer[MAX_SPO2_SAMPLES];
uint32_t redBuffer[MAX_SPO2_SAMPLES];
int32_t bufferLength;
int32_t spo2;
int8_t validSPO2;

#define MAX_HR_SAMPLES 100
uint32_t irHRBuffer[MAX_HR_SAMPLES];
uint16_t hrBufferIndex = 0;

const byte SPO2_AVG_SIZE = 4;
float spo2History[SPO2_AVG_SIZE];
byte spo2HistoryIndex = 0;

float BPM = 0, SpO2 = 0, temperatureC = 0.0;
uint32_t tsLastReport = 0;
bool sensorReady = false;
bool alertActive = false;

// Sensor value offsets
const float tempOffset = 1.0;
const int bpmOffset = 5.0;
const int spo2Offset = 2.0;

// Normal health range thresholds
const float TEMP_NORMAL_LOW = 35.0;
const float TEMP_NORMAL_HIGH = 38.0;
const int BPM_NORMAL_LOW = 60;
const int BPM_NORMAL_HIGH = 120;
const int SPO2_NORMAL_LOW = 95;

uint16_t spo2BufferIndex = 0;
bool spo2BufferFilled = false;
bool fingerDetected = false;
bool beatsDetected = false;
unsigned long fingerPlaceTime = 0;

// Bitmap icons
const unsigned char heart_icon [] PROGMEM = {
  0x66, 0xFF, 0xFF, 0x7E, 0x3C, 0x18
};
const unsigned char pulse_icon [] PROGMEM = {
  0x08, 0x3E, 0x08, 0x00, 0x00, 0x00
};
const unsigned char temp_icon [] PROGMEM = {
  0x1C, 0x22, 0x37, 0x7F, 0x7F, 0x3E
};
const unsigned char lungs_icon [] PROGMEM = {
  0x3C, 0x66, 0x66, 0x7E, 0x3C, 0x00
};

bool ledBlinking = false;
unsigned long ledBlinkStart = 0;
const unsigned long ledBlinkDuration = 2000;

unsigned long alertStartTime = 0;
const unsigned long continuousAlertDuration = 300000; // 5 minutes in milliseconds


bool isVibrating = false;
unsigned long vibrationStart = 0;
const unsigned long vibrationDuration = 10000;

WebServer server(80);
const char* ssid = "Health Monitor";

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

const int buttonPins[8] = {13, 12, 14, 27, 26, 25, 33, 32};
bool lastButtonState[8] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
unsigned long lastDebounceTime[8] = {0};
const unsigned long debounceDelay = 300;
String lastMessage = "No message yet";
const String buttonMessages[8] = {
  "Message 1", "Message 2", "Message 3", "Message 4",
  "Message 5", "Message 6", "Message 7", "Message 8"
};

// WiFi Configuration
// TODO: Configure your WiFi credentials here
const char* WIFI_SSID = "REPLACE_WITH_WIFI_SSID";
const char* WIFI_PASSWORD = "REPLACE_WITH_WIFI_PASSWORD";

// Telegram Configuration
// TODO: Configure your Telegram Bot Token (Get from BotFather)
#define BOT_TOKEN "REPLACE_WITH_BOT_TOKEN"
const int NUM_RECIPIENTS = 2;
const char* CHAT_IDS[NUM_RECIPIENTS] = {
  "REPLACE_WITH_CHAT_ID_1",        // <-- Recipient 1 Chat ID
  "REPLACE_WITH_CHAT_ID_2"         // <-- Recipient 2 Chat ID
};
WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 0;

// Alert Queue
const int MAX_ALERTS = 5;
String alertQueue[MAX_ALERTS];
int alertCount = 0;

// ESP-NOW callback
void onReceive(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  Serial.println("ESP-NOW Message Received.");
  if (len == 1 && incomingData[0] == 1 && !isVibrating) {
    isVibrating = true;
    vibrationStart = millis();
    digitalWrite(VIBRATION_PIN, HIGH);
  }
}

void queueAlert(String message) {
  if (alertCount < MAX_ALERTS) {
    alertQueue[alertCount] = message;
    alertCount++;
    Serial.println("Alert queued: " + message);
  }
}

String SendHTML(float BPM, float SpO2, float temperatureC_raw, String lastMessage, bool fingerDetected, bool beatsDetected) {
  float temperatureC = temperatureC_raw + tempOffset;
  bool bpmOK = (BPM + bpmOffset) >= BPM_NORMAL_LOW && (BPM + bpmOffset) <= BPM_NORMAL_HIGH;
  bool spo2OK = SpO2 >= SPO2_NORMAL_LOW;
  bool tempOK = temperatureC >= TEMP_NORMAL_LOW && temperatureC <= TEMP_NORMAL_HIGH;

  String outOfRange = "";
  if (!bpmOK && BPM > 0) outOfRange += "Heart Rate, ";
  if (!spo2OK && SpO2 > 0) outOfRange += "SpO₂, ";
  if (!tempOK) outOfRange += "Temperature, ";
  if (outOfRange.endsWith(", ")) outOfRange = outOfRange.substring(0, outOfRange.length() - 2);

  String html = R"rawliteral(
<!DOCTYPE html>
<html><head><meta charset="UTF-8"><title>Health Monitor</title>
<meta name="viewport" content="width=device-width, initial-scale=1.0"><meta http-equiv="refresh" content="3">
<style>body{font-family:Arial;background:#f0f0f0;margin:0;padding:20px;}.card{background:white;padding:20px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1);max-width:400px;margin:0 auto;}
h1{color:#008080;margin-bottom:15px;text-align:center;}p{font-size:18px;margin:8px 0;}.alert{color:white;background:#e74c3c;padding:8px;border-radius:4px;margin-top:10px;font-weight:bold;}
.status{color:#27ae60;font-weight:bold;}meter{width:100%;height:20px;margin-bottom:15px;}</style></head><body>
<div class="card"><h1>Health Monitor</h1>)rawliteral";

  if (sensorReady) {
    String bpmStatus = fingerDetected ? (beatsDetected ? String((int)(BPM + bpmOffset)) : "No Beats") : "No Finger";
    html += "<p><b>Heart Rate:</b> " + bpmStatus + " BPM</p>";
    if (beatsDetected && BPM > 0) html += "<meter min='40' max='180' low='60' high='120' optimum='75' value='" + String((int)(BPM + bpmOffset)) + "'></meter>";
    
    String spo2Status = fingerDetected ? String((int)SpO2) : "No Finger";
    html += "<p><b>SpO₂:</b> " + spo2Status + " %</p>";
    if (SpO2 > 0) html += "<meter min='70' max='100' low='90' high='95' optimum='98' value='" + String((int)SpO2) + "'></meter>";
  } else {
    html += "<p style='color:red;'>MAX30102 Not Detected</p>";
  }

  html += "<p><b>Temperature:</b> " + String(temperatureC, 1) + " °C</p><meter min='30' max='45' low='35' high='38' optimum='36.5' value='" + String(temperatureC, 1) + "'></meter>";

  if (outOfRange.length() > 0) {
    html += "<div class='alert'>⚠️ Alert: " + outOfRange + " out of range!</div>";
  } else if (sensorReady && BPM > 0 && SpO2 > 0) {
    html += "<p class='status'>✓ All readings normal</p>";
  }

  html += "<p><b>Last Message:</b> " + lastMessage + "</p><p><small>Auto-refresh every 3 seconds</small></p></div></body></html>";
  return html;
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  
  display.setCursor(0, 0); display.setTextSize(1); display.print("HYGEIA");
  display.drawBitmap(42, 0, heart_icon, 6, 6, SH110X_WHITE);
  display.setCursor(120, 0);
  display.print(WiFi.softAPgetStationNum() > 0 ? "*" : "o");
  
  display.drawFastHLine(0, 9, SCREEN_WIDTH, SH110X_WHITE);  
  bool bpmOK = ((BPM + bpmOffset) >= BPM_NORMAL_LOW && (BPM + bpmOffset) <= BPM_NORMAL_HIGH) || BPM == 0;
  bool spo2OK = (SpO2 >= SPO2_NORMAL_LOW) || SpO2 == 0;
  float adjTemp = temperatureC + tempOffset;
  bool tempOK = adjTemp >= TEMP_NORMAL_LOW && adjTemp <= TEMP_NORMAL_HIGH;
  
  display.setCursor(0, 12); display.drawBitmap(0, 12, pulse_icon, 6, 6, SH110X_WHITE);
  display.setCursor(8, 12);
  if (sensorReady && BPM > 0) {
    display.print(String((int)(BPM + bpmOffset)));
  } else if (fingerDetected && beatsDetected) {
    display.print("..");
  } else if (fingerDetected) {
    display.print("NO BEAT");
  } else {
    display.print("--");
  }
  
  display.setCursor(45, 12); display.drawBitmap(45, 12, lungs_icon, 6, 6, SH110X_WHITE);
  display.setCursor(53, 12);
  if (sensorReady && SpO2 > 0) {
    display.print(String((int)SpO2) + "%");
  } else {
    display.print(fingerDetected ? "..%" : "--%");
  }
  
  display.setCursor(85, 12); display.drawBitmap(85, 12, temp_icon, 6, 6, SH110X_WHITE);
  display.setCursor(93, 12); display.print(String(adjTemp, 1) + "C");
  
  display.drawFastHLine(0, 22, SCREEN_WIDTH, SH110X_WHITE);
  
  display.setCursor(0, 25);
  if ((!bpmOK && BPM > 0) || (!spo2OK && SpO2 > 0) || !tempOK) {
    display.print("ALERT: CHECK VITALS");
  } else if (sensorReady && BPM > 0 && SpO2 > 0) {
    display.print("STATUS: NORMAL");
  } else if (fingerDetected && !beatsDetected && (millis() - fingerPlaceTime > 10000)) {
    display.print("NO BEATS DETECTED");
  } else if (fingerDetected) {
    display.print("CALCULATING...");
  } else {
    display.print("PLACE FINGER");
  }
  
  display.drawFastHLine(0, 35, SCREEN_WIDTH, SH110X_WHITE);
  
  display.setCursor(0, 38); display.print("MSG:"); display.setCursor(0, 48);
  String shortMsg = lastMessage.length() > 21 ? lastMessage.substring(0, 18) + "..." : lastMessage;
  display.print(lastMessage == "No message yet" ? "No recent messages" : shortMsg);
  
  display.display();
}

void showSplashScreen() {
  display.clearDisplay(); display.setTextColor(SH110X_WHITE);
  display.setCursor(20, 10); display.setTextSize(2); display.print("HYGEIA");
  display.setCursor(10, 30); display.setTextSize(1); display.print("Health Monitor");
  display.setCursor(25, 45); display.print("Starting...");
  display.display(); delay(2000);
}

void checkButtons() {
  for (int i = 0; i < 8; i++) {
    bool currentState = digitalRead(buttonPins[i]);
    if (currentState == LOW && lastButtonState[i] == HIGH && (millis() - lastDebounceTime[i] > debounceDelay)) {
      lastDebounceTime[i] = millis();
      lastMessage = buttonMessages[i];

      // --- Enhanced Message Generation ---
      String detailedMessage = buttonMessages[i];
      detailedMessage += "\n\n-- Patient Status --\n";

      float adjBPM = BPM + bpmOffset;
      float adjTemp = temperatureC + tempOffset;

      detailedMessage += "BPM: " + String((int)adjBPM) + "\n";
      detailedMessage += "SpO2: " + String((int)SpO2) + "%\n";
      detailedMessage += "Temp: " + String(adjTemp, 1) + " C\n\n";

      bool bpmOK = (adjBPM >= BPM_NORMAL_LOW && adjBPM <= BPM_NORMAL_HIGH);
      bool spo2OK = (SpO2 >= SPO2_NORMAL_LOW);
      bool tempOK = (adjTemp >= TEMP_NORMAL_LOW && adjTemp <= TEMP_NORMAL_HIGH);

      String status = "";
      if (!bpmOK && BPM > 0) status += "Heart Rate, ";
      if (!spo2OK && SpO2 > 0) status += "SpO2, ";
      if (!tempOK) status += "Temperature, ";

      if (status.length() > 0) {
        detailedMessage += "Status: ⚠️ ALERT - " + status.substring(0, status.length() - 2) + " out of range.";
      } else {
        detailedMessage += "Status: ✅ All readings normal.";
      }
      
      queueAlert(detailedMessage);
      Serial.print("Button "); Serial.print(i + 1); Serial.print(": "); Serial.println(lastMessage);
      digitalWrite(LED_PIN, HIGH); ledBlinking = true; ledBlinkStart = millis();
    }
    lastButtonState[i] = currentState;
  }
}

void triggerContinuousAlert() {
  Serial.println("!!! CONTINUOUS 5-MINUTE ALERT TRIGGERED !!!");

  String detailedMessage = "🚨 PERSISTENT ALERT (5+ mins) 🚨\n";
  detailedMessage += "Patient vitals have been out of range for over 5 minutes.\n\n";
  detailedMessage += "-- Current Patient Status --\n";

  float adjBPM = BPM + bpmOffset;
  float adjTemp = temperatureC + tempOffset;

  detailedMessage += "BPM: " + String((int)adjBPM) + "\n";
  detailedMessage += "SpO2: " + String((int)SpO2) + "%\n";
  detailedMessage += "Temp: " + String(adjTemp, 1) + " C\n\n";

  bool bpmOK = (adjBPM >= BPM_NORMAL_LOW && adjBPM <= BPM_NORMAL_HIGH);
  bool spo2OK = (SpO2 >= SPO2_NORMAL_LOW);
  bool tempOK = (adjTemp >= TEMP_NORMAL_LOW && adjTemp <= TEMP_NORMAL_HIGH);

  String status = "";
  if (!bpmOK && BPM > 0) status += "Heart Rate, ";
  if (!spo2OK && SpO2 > 0) status += "SpO2, ";
  if (!tempOK) status += "Temperature, ";

  if (status.length() > 0) {
    detailedMessage += "Status: ⚠️ ALERT - " + status.substring(0, status.length() - 2) + " out of range.";
  }
  queueAlert(detailedMessage);
}

void handleTelegramAlerts() {
  static unsigned long lastAttempt = 0;
  
  if (alertCount == 0) return;
  if (millis() - lastAttempt < 5000) return;

  Serial.println("\n========== TELEGRAM DEBUG ==========");
  
  wifi_mode_t mode;
  esp_wifi_get_mode(&mode);
  Serial.print("WiFi Mode: ");
  if (mode == WIFI_MODE_STA) Serial.println("STATION");
  else if (mode == WIFI_MODE_AP) Serial.println("AP ONLY");
  else if (mode == WIFI_MODE_APSTA) Serial.println("AP+STATION");
  
  Serial.print("WiFi Status: ");
  Serial.println(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
  Serial.print("Station IP: "); Serial.println(WiFi.localIP());
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  
  if (WiFi.status() == WL_CONNECTED) {
    String message = alertQueue[0];
    lastAttempt = millis();
    
    Serial.print("→ Sending: "); Serial.println(message);

    bool allSent = true;
    for (int i = 0; i < NUM_RECIPIENTS; i++) {
      Serial.print("  -> Sending to Chat ID: "); Serial.println(CHAT_IDS[i]);
      if (!bot.sendMessage(CHAT_IDS[i], message, "")) {
        allSent = false;
        Serial.print("  ❌ FAILED for Chat ID: "); Serial.println(CHAT_IDS[i]);
      }
    }
    
    if (allSent) {
      Serial.println("✅ SUCCESS!");
      for (int i = 0; i < alertCount - 1; i++) {
        alertQueue[i] = alertQueue[i + 1];
      }
      alertCount--;
    } else {
      Serial.println("❌ FAILED for one or more recipients. Will retry.");
    }
  } else {
    Serial.println("❌ Not connected to WiFi!");
  }
  Serial.println("====================================\n");
}

void handle_OnConnect() {
  server.send(200, "text/html", SendHTML(BPM, SpO2, temperatureC, lastMessage, fingerDetected, beatsDetected));
}

void handle_NotFound() {
  server.send(404, "text/plain", "Not found");
}

void readSensorData() {
  if (!sensorReady) return;

  particleSensor.check();
  if (!particleSensor.available()) return;

  uint32_t irValue = particleSensor.getFIFOIR();
  uint32_t redValue = particleSensor.getFIFORed();
  particleSensor.nextSample();

  fingerDetected = (irValue > 50000);
  if (fingerDetected && fingerPlaceTime == 0) fingerPlaceTime = millis();

  if (!fingerDetected) {
    BPM = 0; SpO2 = 0; beatsDetected = false;
    spo2BufferIndex = 0; spo2BufferFilled = false; hrBufferIndex = 0;
    fingerPlaceTime = 0;
    return;
  }

  irBuffer[spo2BufferIndex] = irValue;
  redBuffer[spo2BufferIndex] = redValue;
  spo2BufferIndex = (spo2BufferIndex + 1) % MAX_SPO2_SAMPLES;
  if (spo2BufferIndex == 0) spo2BufferFilled = true;

  if (spo2BufferFilled) {
    bufferLength = MAX_SPO2_SAMPLES;
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &beatsPerMinute, &beatAvg);
    
    if (validSPO2 > 0 && spo2 > 70 && spo2 < 101 && beatsPerMinute > 20 && beatsPerMinute < 255) {
      beatsDetected = true;
      
      float currentSpo2 = spo2;
      spo2History[spo2HistoryIndex] = currentSpo2;
      spo2HistoryIndex = (spo2HistoryIndex + 1) % SPO2_AVG_SIZE;
      
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;
      long total = 0;
      for (byte i = 0; i < RATE_SIZE; i++) total += rates[i];
      BPM = total / RATE_SIZE;
    }
  }

  float totalSpo2 = 0;
  int validSamples = 0;
  for (byte i = 0; i < SPO2_AVG_SIZE; i++) {
    if (spo2History[i] > 0) { totalSpo2 += spo2History[i]; validSamples++; }
  }
  if (validSamples > 0) SpO2 = totalSpo2 / validSamples;
}

void setup() {
  Serial.begin(115200);
  pinMode(VIBRATION_PIN, OUTPUT); digitalWrite(VIBRATION_PIN, LOW);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);

  Wire.begin();
  sensors.begin();

  if (!display.begin(i2c_Address, true)) {
    Serial.println(F("SH1106 allocation failed"));
    for (;;);
  }
  showSplashScreen();

  Serial.println("Initializing MAX30102...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found!");
    sensorReady = false;
  } else {
    Serial.println("MAX30102 found!");
    sensorReady = true;
    
    byte ledBrightness = 60;
    byte sampleAverage = 4;
    byte ledMode = 2;
    int sampleRate = 200;
    int pulseWidth = 411;
    int adcRange = 16384;
    
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    
    for (int i = 0; i < MAX_SPO2_SAMPLES; i++) {
      irBuffer[i] = 0; redBuffer[i] = 0;
    }
    for (int i = 0; i < MAX_HR_SAMPLES; i++) irHRBuffer[i] = 50000;
    for (int i = 0; i < RATE_SIZE; i++) rates[i] = 72;
    for (int i = 0; i < SPO2_AVG_SIZE; i++) spo2History[i] = 0;
  }

  for (int i = 0; i < 8; i++) pinMode(buttonPins[i], INPUT_PULLUP);

  // Connect to internet WiFi (will use same channel)
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  // CRITICAL FIX: Set WiFi to AP+STA mode and configure channel
  WiFi.mode(WIFI_AP_STA);
  // Set the channel based on the connected WiFi to ensure ESP-NOW works
  int32_t channel = WiFi.channel();
  WiFi.softAP(ssid, "", channel);
  Serial.print("AP started on channel "); Serial.println(channel);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("Station IP: "); Serial.println(WiFi.localIP());
    
    // CRITICAL FIX: Use insecure client (no certificate validation)
    // This avoids certificate issues with Telegram API
    client.setInsecure();
    
    // Sync time with timeout
    Serial.print("Syncing time");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    int timeAttempts = 0;
    while (!getLocalTime(&timeinfo) && timeAttempts < 10) {
      Serial.print(".");
      delay(1000);
      timeAttempts++;
    }
    if (timeAttempts < 10) {
      Serial.println("\nTime synced!");
    } else {
      Serial.println("\nTime sync failed, continuing anyway");
    }
  } else {
    Serial.println("\nWiFi connection failed!");
  }

  server.on("/", handle_OnConnect);
  server.onNotFound(handle_NotFound);
  server.begin();

  // Initialize ESP-NOW
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_AP, mac);
  Serial.print("Receiver MAC: ");
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
  } else {
    Serial.println("ESP-NOW initialized");
    esp_now_register_recv_cb(onReceive);
  }

  // Test Telegram
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n🔔 Testing Telegram...");
    delay(2000);
    bool allTestsSent = true;
    for (int i = 0; i < NUM_RECIPIENTS; i++) {
      if (!bot.sendMessage(CHAT_IDS[i], "🏥 ESP32 Health Monitor Started!", "")) {
        allTestsSent = false;
      }
    }
    Serial.println(allTestsSent ? "✅ Telegram working!" : "❌ Telegram failed for one or more recipients!");
  }
  
  Serial.println("Setup complete.\n");
}

void loop() {
  server.handleClient();
  handleTelegramAlerts();
  readSensorData();

  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    sensors.requestTemperatures();
    temperatureC = sensors.getTempCByIndex(0);
    
    float adjTemp = temperatureC + tempOffset;
    bool bpmOK = ((BPM + bpmOffset) >= BPM_NORMAL_LOW && (BPM + bpmOffset) <= BPM_NORMAL_HIGH) || BPM == 0;
    bool spo2OK = (SpO2 >= SPO2_NORMAL_LOW) || SpO2 == 0;
    bool tempOK = adjTemp >= TEMP_NORMAL_LOW && adjTemp <= TEMP_NORMAL_HIGH;

    tsLastReport = millis();
    bool wasAlertActive = alertActive;
    alertActive = !((bpmOK || BPM == 0) && (spo2OK || SpO2 == 0) && tempOK) && fingerDetected;

    if (alertActive && !wasAlertActive) {
      alertStartTime = millis(); // Start the timer
      Serial.println("Alert condition started. 5-min timer initiated.");
    } else if (!alertActive && wasAlertActive) {
      alertStartTime = 0; // Reset the timer
      Serial.println("Alert condition ended. 5-min timer reset.");
    }



    if (!fingerDetected && fingerPlaceTime > 0 && (millis() - fingerPlaceTime > 5000)) {
      beatsDetected = false;
    }

    updateDisplay();
  }

  checkButtons();

  if (alertActive && !ledBlinking) {
    digitalWrite(LED_PIN, HIGH);
    ledBlinking = true;
    ledBlinkStart = millis();
  }
  if (ledBlinking && millis() - ledBlinkStart >= ledBlinkDuration) {
    digitalWrite(LED_PIN, LOW);
    ledBlinking = false;
  }

  if (isVibrating && (millis() - vibrationStart >= vibrationDuration)) {
    isVibrating = false;
    digitalWrite(VIBRATION_PIN, LOW);
  }

  // Check for continuous alert
  if (alertActive && alertStartTime > 0 && (millis() - alertStartTime >= continuousAlertDuration)) {
    triggerContinuousAlert();
    alertStartTime = millis(); // Reset timer to send again in another 5 mins if still active
  }

  delay(10);
}