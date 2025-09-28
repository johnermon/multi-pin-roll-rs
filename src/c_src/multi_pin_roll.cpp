#include <Wire.h>
#include <Adafruit_INA219.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "indexhtml.h"

// WiFi Credentials
const char* ssid = "";
const char* password = "";

#define SWITCH_PIN 0

#define SDA_PIN 20
#define SCL_PIN 21

int P[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21};
const int numPinsMax = sizeof(P) / sizeof(P[0]);
int numPinsUsed = 8;// use first 8 pins, increase to 12 if INA device is not detected

int testTimer = 0;
int lTestPeriod = 150;

struct Solenoid {
  uint8_t pin;
  unsigned long pulseStopTime = 0;
  bool isActive = false;
};
Solenoid Sol[numPinsMax];

struct PulseEvent {
  uint8_t pin;
  uint32_t startTime;
  uint32_t endTime;
  bool active;
};

std::vector<PulseEvent> schedule;
bool isPlaying = false;
uint32_t playStartTime = 0;

typedef struct {
  char command[16];  // A text command like "play"
} PlayCommand;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

volatile int modeFlag = 0;// 0 = stop flag; 1 = hold state; 2 = pulse;
unsigned long lastCommandTime = 0;

Adafruit_INA219 ina219;
// Sensor data
float voltage = 0;
float current = 0;
float power = 0;
bool sensorAvailable = false;

// Web Server
WebServer server(80);

void setAllPins(int states) {
    for(int i=0;i<(numPinsUsed);i++){
      if((states >> i) & 1){
        digitalWrite(P[i], HIGH);
      }else{
        digitalWrite(P[i], LOW);
      }
    }
}

void onReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  PlayCommand cmd;
  memcpy(&cmd, data, sizeof(cmd));

  if (strcmp(cmd.command, "play") == 0) {
    Serial.println("Received PLAY command via ESP-NOW");
    startPlayback();  // your function to begin playback
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);  // Set Wi-Fi mode to "station" (required for ESP-NOW)

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onReceive);  // Register your message handler
}

void startPlayback(){
  playStartTime = millis();
  isPlaying = true;
  
  for (auto& evt : schedule) {
    evt.active = false;
    digitalWrite(evt.pin, LOW);
  }
}

void sendPlayCommand(const uint8_t *slaveAddr) {
  PlayCommand cmd;
  strcpy(cmd.command, "play");  // Fill the struct with the command
  esp_now_send(slaveAddr, (uint8_t *)&cmd, sizeof(cmd));  // Send it over ESP-NOW
}

// Control Command Handler
void handleControl(){
  lastCommandTime = millis();
  String respText = "";
  
  Serial.print("command received");
  if(server.hasArg("set")){
    modeFlag = 1;
    int val = server.arg("set").toInt();
    setAllPins(val);

  }else if(server.hasArg("pulse")){
    if(playStartTime == 0){ // if playback has not started yet, set this to current time
      playStartTime = millis();
    }
    modeFlag = 2;
    int pin = server.arg("pulse").toInt();
    int duration = server.arg("duration").toInt();
    
    PulseEvent evt;
    evt.pin = pin;
    evt.startTime = millis() - playStartTime + 0; // start and end times measured from playStartTime
    evt.endTime = evt.startTime + duration;
    evt.active = false;  // start at next loop
    schedule.push_back(evt);
      
    isPlaying = true;
    

    respText = "Pulsing pin for " + String(duration) + " ms";
    
  }else if(server.hasArg("ltest")){
    modeFlag = 2;
    testTimer = 50;
    lTestPeriod = server.arg("ltest").toInt();
    
    respText = "Latency Test";
    
    
  }else if(server.hasArg("stop")){
      modeFlag = 1;
      respText = "stopped.";
  }else{
    respText = "Invalid command.";
  }
  
  server.send(200, "text/plain", respValues() + respText);
  
}

void handleStatus() {
  server.send(200, "text/plain", respValues());
}

String respValues(){ //generate a string to send variables and their values
 String resp = "";
 if(sensorAvailable){
  resp += "INA Sensor Connected<br>ty";
 }
  resp += ", modeFlag = " + String(modeFlag);
  resp += ", voltage = " + String(voltage,2);
  resp += ", current = " + String(current,2);
  resp += ", power = " + String(power,2);
  return resp;
}

void handleLoad() {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "Missing body");
    return;
  }

  DynamicJsonDocument doc(16384);  // Large buffer for big sequences
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }

  schedule.clear();

  for (JsonObject obj : doc.as<JsonArray>()) {
    PulseEvent evt;
    evt.pin = obj["pn"];
    uint32_t delay_ms = obj["de"];
    uint32_t duration_ms = obj["du"];

    evt.startTime = delay_ms;
    evt.endTime = delay_ms + duration_ms;
    evt.active = false;

    schedule.push_back(evt);
    pinMode(evt.pin, OUTPUT);
    digitalWrite(evt.pin, LOW);
  }

  server.send(200, "text/plain", "Schedule loaded");
}


void handlePlay() {
  sendPlayCommand(broadcastAddress);
  startPlayback();
  String resp = "";
  resp += "Current Free heap: " + String(ESP.getFreeHeap());
  
  server.send(200, "text/plain", resp);
}

// Root Handler
void handleRoot() {
  server.send(200, "text/html", index_html);
}

void setup() {
  // Initialize Serial
  Serial.begin(9600);
    
  pinMode(SWITCH_PIN, INPUT_PULLDOWN);

  // Attempt to connect INA219
  Wire.begin(SDA_PIN,SCL_PIN);
  if (ina219.begin()) {
    sensorAvailable = true;
  }else{
    Serial.println("Failed to find INA219 chip");
    numPinsUsed = numPinsMax;
  }
    
  for(int i=0;i<(numPinsUsed);i++){
    // Initialize GPIO Pins
      pinMode(P[i], OUTPUT);
      digitalWrite(P[i], LOW);
      Sol[i] = {P[i]};
    }
    
  // Connect to WiFi
  WiFi.begin(ssid, password);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Set up ESP NOW
  setupEspNow();

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;      // 0 = current Wi-Fi channel
  peerInfo.encrypt = false;  // No encryption for broadcast
  esp_now_add_peer(&peerInfo);
  
  // Configure Web Server
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/control", handleControl);
  
  server.on("/load", HTTP_POST, handleLoad);
  server.on("/play", HTTP_GET, handlePlay);
  server.begin();
}

void loop() {
  server.handleClient();

  unsigned long now = millis();

  // Read INA219 values periodically
  static unsigned long lastRead = 0;
  bool sensorEnabled = true;
  if (millis() - lastRead >= 250 && sensorAvailable && sensorEnabled) {
    lastRead = millis();
    voltage = ina219.getBusVoltage_V();
    current = ina219.getCurrent_mA();
    power = ina219.getPower_mW();
    Serial.printf("V: %.2f V, I: %.2f mA, P: %.2f mW\n", voltage, current, power);
  }
  if (isPlaying) {
    unsigned long playNow = now - playStartTime;
    bool anyActive = false;

    for (auto& evt : schedule) {
      // Start pulse
      if (!evt.active && playNow >= evt.startTime && playNow < evt.endTime) {
        digitalWrite(evt.pin, HIGH);
        evt.active = true;
      }

      // End pulse
      if (evt.active && playNow >= evt.endTime) {
        digitalWrite(evt.pin, LOW);
        evt.active = false;
      }

      if (playNow < evt.endTime) {
        anyActive = true;
      }
    }

    if (!anyActive) {
      isPlaying = false;
    }
  }
}
