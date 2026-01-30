#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <time.h>
#include <esp_now.h>

#define STEP_PIN 19
#define DIR_PIN 18
const int ledPin = 21;
const int msgLedPin = 13;
const int stepsPerRevolution = 20;
const int microsteps = 10; // Assuming 1/10 microstepping
const long totalSteps = (stepsPerRevolution * microsteps) / 12; // Each slot is 30 degrees (360/12). (200 steps / 360 deg) * 30 deg = ~16.67 steps.
const long delayPerStep = 2500;

WebServer server(80);

const char* ssid = "Time Scheduler";
// TODO: Set a secure password for the Access Point
const char* password = "REPLACE_WITH_AP_PASSWORD";

// --- ESP-NOW & WiFi STA CONFIG ---
const char* GLOVE_AP_SSID = "Health Monitor"; // The SSID of the smart glove's AP

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800; // GMT+5:30
const int daylightOffset_sec = 0;

// IMPORTANT: Replace this with the actual MAC address from the smart glove's serial monitor
// TODO: Set the receiver MAC address
uint8_t receiverMAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t senderMAC[6];

bool scheduled[12] = {false};
bool triggered[12] = {false};
bool expired[12] = {false};
time_t triggerTimes[12] = {0};
int originalDelays[12] = {0}; // Store the original delay in minutes for each slot
bool runLoopNow = false;
bool isRunning[12] = {false};
bool manualRunning = false;

// Non-blocking stepper/LED control
bool stepperSequenceActive = false;
unsigned long stepperSequenceStart = 0;
const unsigned long ledOnDuration = 3000;

bool msgLedBlink = false;
unsigned long msgLedStart = 0;
const unsigned long msgLedDuration = 2000;

unsigned long stopwatchStartTime = 0;
bool stopwatchRunning = false;
unsigned long elapsedTime = 0;

String lastReceivedMessage = ""; //  Store last received ESP-NOW message

// Function Prototypes (Forward Declarations)
void startStepperSequence(bool isManual, int scheduleIndex = -1);
void handleStepperSequence();

void setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(msgLedPin, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(msgLedPin, LOW);

  // Set device as a Wi-Fi Station and Access Point
  WiFi.mode(WIFI_AP_STA);

  // Connect to the smart glove's AP to ensure they are on the same channel
  WiFi.begin(GLOVE_AP_SSID);
  Serial.print("Connecting to Glove AP");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Glove AP!");

  // Start this device's own AP for its web interface on the SAME channel
  WiFi.softAP(ssid, password, WiFi.channel());
  Serial.println("Web Interface AP Started on same channel as Glove.");

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  WiFi.macAddress(senderMAC);
  Serial.print("Sender MAC Address: ");
  for (int i = 0; i < 6; i++) {
    if (senderMAC[i] < 16) Serial.print("0");
    Serial.print(senderMAC[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
  }

  esp_now_register_recv_cb(onDataRecv);
  Serial.println("ESP-NOW receive callback registered.");

  server.on("/", HTTP_GET, handleRoot);
  server.on("/reset", HTTP_GET, handleReset);
  server.on("/triggerLoop", HTTP_GET, handleManualTrigger);
  server.on("/setSchedule", HTTP_GET, handleSetSchedule);
  server.on("/getStopwatch", HTTP_GET, handleGetStopwatch);
  server.on("/getLastMsg", HTTP_GET, handleGetLastMessage); //  API to serve last received message

  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();
  checkAndRunSchedule();

  if (runLoopNow) {
    runLoopNow = false;
    startStepperSequence(true);
  }

  if (stopwatchRunning) {
    bool allCompleted = true;
    for (int i = 0; i < 12; i++) {
      if (scheduled[i] && !triggered[i]) {
        allCompleted = false;
        break;
      }
    }

    if (allCompleted) {
      stopwatchRunning = false;
      elapsedTime = millis() - stopwatchStartTime;
    }
  }

  if (msgLedBlink && (millis() - msgLedStart >= msgLedDuration)) {
    digitalWrite(msgLedPin, LOW);
    msgLedBlink = false;
  }

  handleStepperSequence();
}

void checkAndRunSchedule() {
  time_t now;
  time(&now);

  for (int i = 0; i < 12; i++) {
    if (scheduled[i] && !triggered[i]) {
      if (now >= triggerTimes[i]) {
        startStepperSequence(false, i);
        triggered[i] = true;
      } else if (now > triggerTimes[i] + 60) {
        expired[i] = true;
        scheduled[i] = false;
      }
    }
  }
}

void handleSetSchedule() {
  if (server.hasArg("b")) {
    String param = server.arg("b");
    param.trim();

    for (int i = 0; i < 12; i++) {
      scheduled[i] = false;
      triggered[i] = false;
      expired[i] = false;
      triggerTimes[i] = 0;
      originalDelays[i] = 0; // Reset original delays
    }

    time_t now;
    time(&now);

    bool anyScheduled = false;
    while (param.length() > 0) {
      int sep = param.indexOf(',');
      String pair = (sep == -1) ? param : param.substring(0, sep);
      int dash = pair.indexOf('-');
      if (dash > 0) {
        int index = pair.substring(0, dash).toInt();
        int delayMin = pair.substring(dash + 1).toInt();
        if (index >= 0 && index < 12 && delayMin > 0) {
          scheduled[index] = true;
          triggerTimes[index] = now + (delayMin * 60);
          originalDelays[index] = delayMin; // Store the original delay
          anyScheduled = true;
        }
      }
      if (sep == -1) break;
      param = param.substring(sep + 1);
    }

    if (anyScheduled) {
      stopwatchStartTime = millis();
      elapsedTime = 0;
      stopwatchRunning = true;
      Serial.println("Stopwatch started");
    }
  }

  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  Serial.print("Received message from: ");
  char macStr[18];
  snprintf(macStr, sizeof(macStr),
           "%02X:%02X:%02X:%02X:%02X:%02X",
           info->src_addr[0], info->src_addr[1], info->src_addr[2],
           info->src_addr[3], info->src_addr[4], info->src_addr[5]);
  Serial.println(macStr);

  Serial.print("Data: ");
  lastReceivedMessage = ""; // Reset
  for (int i = 0; i < len; i++) {
    lastReceivedMessage += (char)data[i]; // Store message
    Serial.print((char)data[i]);
  }
  Serial.println();

  digitalWrite(msgLedPin, HIGH);
  msgLedStart = millis();
  msgLedBlink = true;
}

void handleGetStopwatch() {
  unsigned long currentTime = stopwatchRunning ? (millis() - stopwatchStartTime) : elapsedTime;
  server.send(200, "text/plain", String(currentTime));
}

void handleGetLastMessage() {
  server.send(200, "text/plain", lastReceivedMessage); // Serve last message
}

void handleRoot() {
  String html = R"rawliteral(
    <html><head>
    <title>Medicine Supply Scheduler</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
      body {
        font-family: 'Segoe UI', Tahoma, sans-serif;
        background: #f9f9f9;
        margin: 0;
        padding: 20px;
        color: #333;
      }
      .header {
        background-color: #4CAF50;
        color: white;
        padding: 20px;
        border-radius: 12px;
        text-align: center;
        margin-bottom: 20px;
      }
      .header h1 {
        margin: 0;
        font-size: 26px;
      }
      .card {
        background: white;
        padding: 20px;
        border-radius: 12px;
        box-shadow: 0 2px 6px rgba(0,0,0,0.1);
        margin-bottom: 20px;
      }
      .stopwatch {
        font-size: 1.4em;
        text-align: center;
        background: #e0f7fa;
        padding: 10px;
        border-radius: 8px;
        margin-bottom: 20px;
      }
      .manual, .set-btn, .done-btn {
        width: 100%;
        height: 45px;
        margin-top: 10px;
        font-weight: bold;
        font-size: 16px;
        border: none;
        border-radius: 6px;
        cursor: pointer;
      }
      .manual { background-color: #ffc107; }
      .set-btn { background-color: #8bc34a; color: white; }
      .reset-btn { background-color: #e57373; color: white; }
      label {
        display: block;
        margin: 10px 0 5px;
        font-weight: bold;
      }
      input[type=number] {
        width: 60px;
        padding: 5px;
        font-size: 16px;
        margin-right: 5px;
        border-radius: 4px;
        border: 1px solid #ccc;
      }
      .time-row {
        display: flex;
        align-items: center;
        justify-content: space-between;
        margin-bottom: 8px;
      }
      .status {
        font-size: 14px;
        font-weight: bold;
        padding-left: 10px;
      }
      .status.running { color: #2196F3; }
      .status.done { color: green; }
      .status.scheduled { color: orange; }
      .status.expired { color: red; }

      .input-columns {
        display: flex;
        flex-wrap: wrap;
        gap: 20px;
      }
      .left-column, .right-column {
        flex: 1;
        min-width: 300px;
      }
    </style>
    </head><body>

    <div class="header">
      <h1>Medicine Supply Scheduler</h1>
      <p>Enter Patient's Medicine Supply Timetable</p>
    </div>

    <div class="stopwatch" id="stopwatchDisplay">Stopwatch: 00:00:00.000</div>

    <div class="card">
      <h2>Manual Trigger</h2>
      <button class="manual" id="manualTriggerBtn" onclick="triggerLoop()">Run Stepper Motor and Vibrating Loop</button>
    </div>

    <div class="card">
      <h2>Set Time (hours and minutes)</h2>
      <form id="scheduleForm" onsubmit="return false;">
      <div id="time-slots-container">
        <!-- Time slots will be dynamically inserted here -->
      </div>
      <button class="set-btn" onclick="addSlot()">+ Add Time Slot</button>
  )rawliteral";

  time_t now;
  time(&now);

  html += R"rawliteral(
      </form>
      <button class='set-btn' id="setBtn" onclick='setSchedule()'>Set Schedule</button>
      <form action="/reset" method="get">
        <button class='reset-btn' type="submit">Reset All</button>
      </form>
    </div>
    <script>
      let slotCounter = 0;
      const MAX_SLOTS = 12;

      function setSchedule() {
        let params = [];
        const slots = document.querySelectorAll('.time-row');
        
        // Create a map to hold index and total minutes
        const scheduleMap = new Map();

        slots.forEach(slot => {
          const indexInput = slot.querySelector('input[type=hidden]');
          const hInput = slot.querySelector('.hour-input');
          const mInput = slot.querySelector('.min-input');
          
          if (indexInput && hInput && mInput) {
            const index = parseInt(indexInput.value);
            const h = hInput.value;
            const m = mInput.value;
            
            if (h !== '' || m !== '') {
              const totalMin = (parseInt(h || '0') * 60) + parseInt(m || '0');
              if (totalMin > 0) {
                scheduleMap.set(index, totalMin);
              }
            }
          }
        });

        // Convert map to params array, ensuring unique indices
        for(let [index, totalMin] of scheduleMap.entries()){
            params.push(index + '-' + totalMin);
        }

        if (params.length > 0) {
          window.location.href = '/setSchedule?b=' + params.join(',');
        }
      }

      function triggerLoop() {
        const btn = document.getElementById('manualTriggerBtn');
        btn.disabled = true;
        btn.className = 'manual';
        btn.innerText = "Running...";
        fetch('/triggerLoop').then(() => {
          setTimeout(() => {
            btn.disabled = false;
            btn.innerText = "Run Stepper Motor and Vibrating Loop";
          }, 4000);
        });
      }

      function formatTime(ms) {
        let hours = Math.floor(ms / 3600000);
        ms %= 3600000;
        let minutes = Math.floor(ms / 60000);
        ms %= 60000;
        let seconds = Math.floor(ms / 1000);
        ms %= 1000;

        return (hours < 10 ? '0' : '') + hours + ':' +
               (minutes < 10 ? '0' : '') + minutes + ':' +
               (seconds < 10 ? '0' : '') + seconds + '.' +
               (ms < 100 ? (ms < 10 ? '00' : '0') : '') + ms;
      }

      function updateStopwatch() {
        fetch('/getStopwatch')
          .then(response => response.text())
          .then(time => {
            document.getElementById('stopwatchDisplay').innerText = 'Stopwatch: ' + formatTime(parseInt(time));
          });
      }

      function updateLastMessage() {
        fetch('/getLastMsg')
          .then(response => response.text())
          .then(msg => {
            document.getElementById('msgDisplay').innerText = msg || "None";
          });
      }



      const scheduled = [)rawliteral";

  for (int i = 0; i < 12; i++) {
    if (i > 0) html += ",";
    html += scheduled[i] ? "true" : "false";
  }
  html += "]\nconst triggered = [";
  for (int i = 0; i < 12; i++) {
    if (i > 0) html += ",";
    html += triggered[i] ? "true" : "false";
  }
  html += "]\nconst expired = [";
  for (int i = 0; i < 12; i++) {
    if (i > 0) html += ",";
    html += expired[i] ? "true" : "false";
  }
  html += "]\nconst running = [";
  for (int i = 0; i < 12; i++) {
    if (i > 0) html += ",";
    html += isRunning[i] ? "true" : "false";
  }
  html += "]\nconst triggerTimes = [";
  for (int i = 0; i < 12; i++) {
    if (i > 0) html += ",";
    html += String(triggerTimes[i]);
  }
  html += "]\nconst originalDelays = [";
  for (int i = 0; i < 12; i++) {
    if (i > 0) html += ",";
    html += String(originalDelays[i]);
  }
  html += "]\n";

  html += R"rawliteral(
      const container = document.getElementById('time-slots-container');

      function createSlot(index, hours = '', minutes = '') {
        if (slotCounter >= MAX_SLOTS) return;
        
        const div = document.createElement('div');
        div.className = 'time-row';
        div.innerHTML = `
          <div>
            <label>Input ${slotCounter + 1}:</label>
            <input type="hidden" value="${index}">
            <input type="number" class="hour-input" placeholder="hr" min="0" value="${hours}" />
            <input type="number" class="min-input" placeholder="min" min="0" value="${minutes}" />
          </div>
          <span class="status" id="status${index}"></span>
        `;
        container.appendChild(div);
        slotCounter++;
        updateStatus(index);
      }

      function addSlot() {
        if (slotCounter >= MAX_SLOTS) {
          alert("Maximum of 12 time slots reached.");
          return;
        }
        // Find the next available index that is not already scheduled
        let nextIndex = 0;
        while(nextIndex < MAX_SLOTS) {
            const isUsed = Array.from(container.querySelectorAll('input[type=hidden]')).some(input => parseInt(input.value) === nextIndex);
            if (!isUsed) break;
            nextIndex++;
        }
        if(nextIndex < MAX_SLOTS) {
            createSlot(nextIndex);
        }
      }

      function updateStatus(index) {
        const el = document.getElementById('status' + index);
        if (!el) return;

        if (running[index]) {
          el.innerText = 'Running';
          el.className = 'status running';
        } else if (triggered[index]) {
          el.innerText = 'Done';
          el.className = 'status done';
        } else if (scheduled[index]) {
          el.innerText = 'Scheduled';
          el.className = 'status scheduled';
        } else if (expired[index]) {
          el.innerText = 'Expired';
          el.className = 'status expired';
        } else {
          el.innerText = '';
          el.className = 'status';
        }
      }

      function updateRemainingTimes() {
        const now = Math.floor(Date.now() / 1000);
        for (let i = 0; i < triggerTimes.length; i++) {
          if (scheduled[i] && triggerTimes[i] > now) {
            const remaining = triggerTimes[i] - now;
            const hours = Math.floor(remaining / 3600);
            const minutes = Math.floor((remaining % 3600) / 60);
            const seconds = remaining % 60;

            const statusEl = document.getElementById(`status${i}`);
            if (statusEl) {
              statusEl.setAttribute("title", `${hours}h ${minutes}m ${seconds}s remaining`);
            }
          }
        }
      }

      document.addEventListener('DOMContentLoaded', () => {
        const now = Math.floor(Date.now() / 1000);
        for (let i = 0; i < triggerTimes.length; i++) {
          if (scheduled[i] && originalDelays[i] > 0) {
            const hours = Math.floor(originalDelays[i] / 60);
            const minutes = originalDelays[i] % 60;
            createSlot(i, hours, minutes);
          }
        }
      });

      setInterval(updateLastMessage, 1000);
      setInterval(updateRemainingTimes, 1000);
      setInterval(updateStopwatch, 100);
      setInterval(() => location.reload(), 30000);
    </script>
    </body></html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleManualTrigger() {
  runLoopNow = true;
  server.send(200, "text/plain", "OK");
}

void handleReset() {
  for (int i = 0; i < 12; i++) {
    scheduled[i] = false;
    triggered[i] = false;
    expired[i] = false;
    isRunning[i] = false; 
    triggerTimes[i] = 0;
  }
  stopwatchRunning = false;
  elapsedTime = 0;
  lastReceivedMessage = ""; //  Clear message too if needed
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void startStepperSequence(bool isManual, int scheduleIndex) {
  if (stepperSequenceActive) return; // Prevent re-triggering if already running

  stepperSequenceActive = true;
  stepperSequenceStart = millis();
  manualRunning = isManual;
  if (scheduleIndex != -1) {
    isRunning[scheduleIndex] = true;
  }

  uint8_t msg[] = {1};
  esp_err_t result = esp_now_send(receiverMAC, msg, sizeof(msg));
  if (result == ESP_OK) {
    Serial.println("Vibration signal sent via ESP-NOW.");
  } else {
    Serial.println("Failed to send vibration signal.");
  }  
  
  for (long i = 0; i < totalSteps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(delayPerStep);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(delayPerStep);
  }
  
  digitalWrite(ledPin, HIGH);
}

void handleStepperSequence() {
  if (!stepperSequenceActive) return;

  if (millis() - stepperSequenceStart >= ledOnDuration) {
    digitalWrite(ledPin, LOW);
    stepperSequenceActive = false;
    manualRunning = false;

    // Reset isRunning flags
    for(int i=0; i<12; i++){
      if(isRunning[i]){
        isRunning[i] = false;
      }
    }
    Serial.println("Stepper sequence finished.");
  }
}
