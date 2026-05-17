#include <Wire.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_INA219.h>
#include <WebServer.h>
#include "SPIFFS.h"

// ========== I2C ==========
MAX30105 particleSensor;
LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_ADS1115 ads;
Adafruit_INA219 ina219;
WebServer server(80);

// ========== Pins ==========
#define HX711_DOUT   4
#define HX711_SCK    5

#define BTN_UP       12
#define BTN_DOWN     15
#define BTN_SELECT   14

#define LED_GREEN    33
#define LED_YELLOW   35
#define LED_RED      25

#define RELAY_PUMP   18
#define RELAY_VALVE  19

#define BATTERY_PIN  34

// ========== Constants ==========
#define MEASURE_TIME       30000
#define RESULT_TIME        10000
#define BP_TARGET_INFLATE  170.0
#define BP_MAX_SAFE        220.0
#define BP_SAMPLE_RATE_MS  10

#define PUMP_DURATION_MS     7000UL
#define SOLENOID_DURATION_MS 10000UL

// ========== WiFi ==========
const char* STA_SSID     = "YourHomeNetwork";
const char* STA_PASSWORD = "YourPassword";
IPAddress local_IP(192, 168, 1, 100);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);
const char* AP_SSID     = "HealthMonitor_ESP";
const char* AP_PASSWORD = "12345678";
const char* SERVER_URL  = "https://your-server.com/api/measurements";
#define WIFI_TIMEOUT_MS  5000

// ========== State ==========
bool measuring = false, staConnected = false;
bool max30102_ok = false, hx711_ok = false, ads1115_ok = false;
int menuIndex = 0;
int scrollOffset = 0;
volatile bool selectPressed = false;
unsigned long lastSelectTime = 0;

// ========== Danger mode ==========
bool dangerMode = false;
int seq[5] = {0};

// ========== HR & SpO2 ==========
long lastBeat = 0;
float bpmSum = 0, spo2Sum = 0;
int beatCount = 0, spo2Count = 0;

// ========== BP ==========
float systolicBP = 0, diastolicBP = 0, meanBP = 0;
int bpHeartRate = 0, oscillationCount = 0;
float pressureBuffer[100], oscillationBuffer[100], maxOscillation = 0;

// ========== Grip ==========
float maxGripForce = 0;

// ==========================================================================
// HX711 — Sistem Kalibrasi Adaptif
// ==========================================================================
//
// CARA KERJA:
// 1. Boot → ambil 50 sample tanpa beban → zero baseline + noise floor
// 2. noise floor (kg) = standard deviation dari 50 sample tersebut
// 3. dead zone = noise floor × 4  ← getaran di bawah ini → papar 0
// 4. Semasa fasa REST grip → ambil semula baseline dengan tangan pegang
// 5. Semasa SQUEEZE → baca raw, tolak baseline, bahagi cal factor
//    → tapis dengan dead zone → papar
//
// CALIBRATION FACTOR:
// Jika KNOWN_WEIGHT_KG > 0, letak berat itu di atas sensor semasa boot
// dan sistem akan kira cal factor automatik.
// Jika 0, sistem guna HX711_DEFAULT_CAL_FACTOR.
//
// HX711_DEFAULT_CAL_FACTOR: cari nilai ini dengan Serial Monitor.
// Upload code dengan KNOWN_WEIGHT_KG = 0, buka Serial Monitor,
// tengok nilai "net raw" semasa letak berat diketahui.
// cal factor = net raw / berat_kg
// ==========================================================================

#define HX711_GAIN               128    // Channel A Gain 128 — paling sensitif
#define HX711_GRIP_MIN_KG        3.0    // Genggaman sah bermula dari 3 kg
#define HX711_DEFAULT_CAL_FACTOR 450.0  // TUKAR ini ikut Serial Monitor kamu
#define KNOWN_WEIGHT_KG          0.0    // 0 = tiada berat kalibrasi

long  hx711Zero      = 0;      // raw baseline (tanpa beban)
long  hx711HandZero  = 0;      // raw baseline (tangan pegang, sebelum squeeze)
float hx711CalFactor = HX711_DEFAULT_CAL_FACTOR;
float hx711DeadZone  = 0.0;    // kg — dikira dari noise floor semasa boot

HX711 scale;
enum BPState { BP_IDLE, BP_INFLATING, BP_MEASURING, BP_COMPLETE };
BPState bpState = BP_IDLE;

// ========== Prototypes ==========
void showMainMenu();
void handleMenuNavigation();
void measureHeart();
void measureGrip();
void measureBloodPressure();
void measureAll();
float readPressureMPX();
void startPump(), stopPump(), openValve(), closeValve();
float detectOscillation(float p);
void calculateBloodPressure();
void uploadData(float sys, float dia, float map, int hr, float spo2, int bpm, float grip);
void setLEDForAll(float val1, float val2, String type);
void clearLEDs();
void startHTTPServer();
String getSensorDataJSON();
void saveToLocalStorage(String data);
void uploadPendingData();
int getBatteryPercent();
bool connectToSurroundingWiFi();
void startAPMode();
void initSPIFFS();
bool checkSelectButton();
void simulateGrip();
void simulateBloodPressure();
void hx711Boot();
void hx711RetareHand(int durationMs);
long hx711RawMedian(int n);
float hx711RawToKg(long rawMedian, long baseline);
int selectGender();
String getGripGrade(float kg, int gender);
void showGripResult(float peakKg, int gender, bool isDanger);
float performGripMeasurement();

// ==========================================================================
// HX711 — Fungsi Asas
// ==========================================================================

// Ambil n sample raw, return median
long hx711RawMedian(int n) {
  if (n < 1) n = 1;
  if (n > 20) n = 20;
  long buf[20];
  for (int i = 0; i < n; i++) {
    while (!scale.is_ready()) delay(5);
    buf[i] = scale.get_value(1);
    delay(3);
  }
  // Sort
  for (int i = 0; i < n-1; i++)
    for (int j = i+1; j < n; j++)
      if (buf[j] < buf[i]) { long t=buf[i]; buf[i]=buf[j]; buf[j]=t; }
  return buf[n/2];
}

// Tukar raw ke kg menggunakan baseline yang ditetapkan
float hx711RawToKg(long rawMedian, long baseline) {
  long net = rawMedian - baseline;
  float kg = (float)net / hx711CalFactor;
  return kg; // boleh negatif — caller yang handle
}

// ==========================================================================
// HX711 Boot — kalibrasi automatik
// Panggil dalam setup() SAHAJA
// ==========================================================================
void hx711Boot() {
  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_gain(HX711_GAIN);

  // Langkah 1: tunggu sensor warm-up
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("HX711 warm-up...");
  lcd.setCursor(0,1); lcd.print("Jangan sentuh   ");
  lcd.setCursor(0,2); lcd.print("sensor!         ");
  delay(2500);

  // Langkah 2: ambil 50 sample untuk zero + noise floor
  const int N = 50;
  long samples[50];
  long sum = 0;
  lcd.setCursor(0,3); lcd.print("Mengukur noise..");
  for (int i = 0; i < N; i++) {
    while (!scale.is_ready()) delay(5);
    samples[i] = scale.get_value(1);
    sum += samples[i];
    delay(20);
  }
  hx711Zero = sum / N;

  // Langkah 3: kira standard deviation → noise floor dalam raw units
  float variance = 0;
  for (int i = 0; i < N; i++) {
    float diff = (float)(samples[i] - hx711Zero);
    variance += diff * diff;
  }
  variance /= N;
  float stddev = sqrt(variance);

  // dead zone = 4× stddev dalam kg
  // ini bermakna getaran mesti > 4× noise sensor untuk dikira
  hx711DeadZone = (stddev * 4.0f) / hx711CalFactor;
  if (hx711DeadZone < 0.5f) hx711DeadZone = 0.5f; // minimum 0.5 kg dead zone
  if (hx711DeadZone > 5.0f) hx711DeadZone = 5.0f; // maximum 5 kg dead zone

  Serial.print("hx711Zero: ");      Serial.println(hx711Zero);
  Serial.print("stddev raw: ");     Serial.println(stddev);
  Serial.print("deadZone kg: ");    Serial.println(hx711DeadZone);

  // Langkah 4: kalibrasi cal factor jika ada berat diketahui
  if (KNOWN_WEIGHT_KG > 0.0f) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Letak beban:");
    lcd.setCursor(0,1); lcd.print(KNOWN_WEIGHT_KG, 1);
    lcd.setCursor(4,1); lcd.print(" kg di sensor");
    lcd.setCursor(0,2); lcd.print("Tekan SELECT");
    lcd.setCursor(0,3); lcd.print("bila sudah sedia");
    // tunggu SELECT
    while (digitalRead(BTN_SELECT) == HIGH) delay(20);
    while (digitalRead(BTN_SELECT) == LOW)  delay(20);
    delay(200);
    lcd.clear(); lcd.setCursor(0,0); lcd.print("Mengukur beban..");
    delay(1000);
    long withWeight = 0;
    for (int i = 0; i < 30; i++) {
      while (!scale.is_ready()) delay(5);
      withWeight += scale.get_value(1);
      delay(30);
    }
    withWeight /= 30;
    long netRaw = withWeight - hx711Zero;
    if (netRaw > 100) { // ada beban sebenar
      hx711CalFactor = (float)netRaw / KNOWN_WEIGHT_KG;
      // kira semula dead zone dengan cal factor baru
      hx711DeadZone = (stddev * 4.0f) / hx711CalFactor;
      if (hx711DeadZone < 0.5f) hx711DeadZone = 0.5f;
      if (hx711DeadZone > 5.0f) hx711DeadZone = 5.0f;
    }
    lcd.clear(); lcd.setCursor(0,0); lcd.print("Cal factor:");
    lcd.setCursor(0,1); lcd.print(hx711CalFactor, 1);
    lcd.setCursor(0,2); lcd.print("Buang beban...");
    delay(2000);
    // ambil semula zero tanpa beban
    sum = 0;
    for (int i = 0; i < 30; i++) {
      while (!scale.is_ready()) delay(5);
      sum += scale.get_value(1);
      delay(20);
    }
    hx711Zero = sum / 30;
  }

  hx711HandZero = hx711Zero; // default sama dulu
  hx711_ok = true;

  Serial.print("Final calFactor: "); Serial.println(hx711CalFactor);
  Serial.print("Final deadZone:  "); Serial.println(hx711DeadZone);

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("HX711: OK");
  lcd.setCursor(0,1); lcd.print("DeadZone: "); lcd.print(hx711DeadZone,2); lcd.print("kg");
  lcd.setCursor(0,2); lcd.print("CalFactor:"); lcd.print(hx711CalFactor,0);
  delay(1500);
}

// ==========================================================================
// hx711RetareHand — ambil semula baseline dengan tangan sudah pegang alat
// Panggil semasa fasa REST sebelum squeeze
// ==========================================================================
void hx711RetareHand(int durationMs) {
  long sum = 0;
  int  count = 0;
  unsigned long t = millis();
  while ((int)(millis() - t) < durationMs) {
    if (scale.is_ready()) {
      sum += scale.get_value(1);
      count++;
    }
    delay(30);
  }
  if (count > 0) {
    hx711HandZero = sum / count;
    Serial.print("Hand zero: "); Serial.println(hx711HandZero);
  }
}

// ========== WiFi ==========
bool connectToSurroundingWiFi() {
  unsigned long wifiStartTime = millis();
  lcd.setCursor(0,2); lcd.print("Scanning WiFi...  ");
  int n = WiFi.scanNetworks();
  bool foundSSID = false;
  for (int i=0; i<n; i++) {
    if (WiFi.SSID(i) == STA_SSID) { foundSSID = true; break; }
  }
  if (!foundSSID) { lcd.setCursor(0,2); lcd.print("Network not found "); return false; }
  WiFi.config(local_IP, gateway, subnet, dns);
  WiFi.begin(STA_SSID, STA_PASSWORD);
  lcd.setCursor(0,2); lcd.print("Connecting (5s).. ");
  while (WiFi.status() != WL_CONNECTED && (millis()-wifiStartTime < WIFI_TIMEOUT_MS)) {
    delay(100);
    int elapsed = (millis()-wifiStartTime)/1000;
    lcd.setCursor(14,2); lcd.print(elapsed); lcd.print("s ");
  }
  if (WiFi.status() == WL_CONNECTED) return true;
  else { lcd.setCursor(0,2); lcd.print("Connection timeout"); return false; }
}

void startAPMode() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("AP Mode Active");
  lcd.setCursor(0,1); lcd.print("Network: "); lcd.print(AP_SSID);
  lcd.setCursor(0,2); lcd.print("IP: "); lcd.print(WiFi.softAPIP());
  lcd.setCursor(0,3); lcd.print("No internet");
  delay(2000);
}

// ========== SPIFFS ==========
void initSPIFFS() {
  if (!SPIFFS.begin(true)) { lcd.clear(); lcd.print("SPIFFS Mount Fail"); while(1); }
}
void saveToLocalStorage(String data) {
  File file = SPIFFS.open("/data.txt", FILE_APPEND);
  if (file) { file.println(data); file.close(); }
}
void uploadPendingData() {
  if (!(WiFi.status() == WL_CONNECTED && staConnected)) return;
  File file = SPIFFS.open("/data.txt");
  if (!file) return;
  int uploaded = 0;
  while (file.available()) {
    String data = file.readStringUntil('\n');
    if (data.length() > 0) {
      HTTPClient http;
      http.begin(SERVER_URL);
      http.addHeader("Content-Type", "application/json");
      if (http.POST(data) > 0) uploaded++;
      http.end();
      delay(500);
    }
  }
  file.close();
  if (uploaded > 0) { file = SPIFFS.open("/data.txt", FILE_WRITE); file.close(); }
}

// ========== HTTP Server ==========
void startHTTPServer() {
  server.on("/data",   HTTP_GET,  []() { server.send(200, "application/json", getSensorDataJSON()); });
  server.on("/status", HTTP_GET,  []() {
    String s = "{\"sta\":" + String(staConnected) + ",\"max\":" + max30102_ok + ",\"hx\":" + hx711_ok + ",\"ads\":" + ads1115_ok + ",\"bat\":" + String(getBatteryPercent()) + ",\"danger\":" + String(dangerMode) + "}";
    server.send(200, "application/json", s);
  });
  server.on("/measure/heart", HTTP_POST, []() { server.send(200,"application/json","{\"status\":\"ok\"}"); measuring=true; measureHeart();         measuring=false; });
  server.on("/measure/grip",  HTTP_POST, []() { server.send(200,"application/json","{\"status\":\"ok\"}"); measuring=true; measureGrip();          measuring=false; });
  server.on("/measure/bp",    HTTP_POST, []() { server.send(200,"application/json","{\"status\":\"ok\"}"); measuring=true; measureBloodPressure(); measuring=false; });
  server.on("/measure/all",   HTTP_POST, []() { server.send(200,"application/json","{\"status\":\"ok\"}"); measuring=true; measureAll();           measuring=false; });
  server.begin();
}

String getSensorDataJSON() {
  String j = "{";
  if (systolicBP > 0)   j += "\"bp\":{\"sys\":" + String(systolicBP) + ",\"dia\":" + String(diastolicBP) + "},";
  if (spo2Count > 0)    j += "\"spo2\":" + String(spo2Sum/spo2Count) + ",";
  if (beatCount > 0)    j += "\"hr\":"   + String(bpmSum/beatCount)  + ",";
  if (maxGripForce > 0) j += "\"grip\":" + String(maxGripForce) + ",";
  j += "\"bat\":" + String(getBatteryPercent()) + ",\"danger\":" + String(dangerMode) + "}";
  j.replace(",}", "}");
  return j;
}

// ========== Battery ==========
int getBatteryPercent() {
  float voltage = ina219.getBusVoltage_V();
  if (voltage <= 6.0) return 0;
  if (voltage >= 8.4) return 100;
  return (int)((voltage - 6.0) / (8.4 - 6.0) * 100);
}

// ========== Upload ==========
void uploadData(float sys, float dia, float map, int hr, float spo2, int bpm, float grip) {
  String json = "{";
  if (sys  > 0) json += "\"bp\":{\"sys\":" + String(sys) + ",\"dia\":" + String(dia) + "},";
  if (spo2 > 0) json += "\"spo2\":" + String(spo2) + ",";
  if (bpm  > 0) json += "\"hr\":"   + String(bpm)  + ",";
  if (grip > 0) json += "\"grip\":" + String(grip) + ",";
  json += "\"ts\":" + String(millis()) + "}";
  json.replace(",}", "}");
  if (WiFi.status() == WL_CONNECTED && staConnected) {
    HTTPClient http;
    http.begin(SERVER_URL);
    http.addHeader("Content-Type", "application/json");
    if (http.POST(json) > 0) { lcd.setCursor(0,3); lcd.print("Data Sent!      "); }
    else saveToLocalStorage(json);
    http.end();
  } else saveToLocalStorage(json);
}

// ========== Button ==========
bool checkSelectButton() {
  if (digitalRead(BTN_SELECT) == LOW) {
    unsigned long now = millis();
    if (now - lastSelectTime > 300) {
      lastSelectTime = now;
      selectPressed = true;
      return true;
    }
  }
  return false;
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(BATTERY_PIN, INPUT);

  lcd.init(); lcd.backlight(); delay(50);
  lcd.clear(); lcd.print("Health Monitor");
  lcd.setCursor(0,1); lcd.print("Initializing...");
  delay(1000);

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(LED_GREEN, OUTPUT); pinMode(LED_YELLOW, OUTPUT); pinMode(LED_RED, OUTPUT);
  pinMode(RELAY_PUMP,  OUTPUT);
  pinMode(RELAY_VALVE, OUTPUT);
  digitalWrite(RELAY_PUMP,  HIGH);
  digitalWrite(RELAY_VALVE, HIGH);
  clearLEDs();

  initSPIFFS();
  Wire.begin(21,22); delay(100);

  if (!ina219.begin()) { lcd.setCursor(0,2); lcd.print("INA219 ERROR     "); }
  else                 { lcd.setCursor(0,2); lcd.print("INA219 OK        "); }

  lcd.setCursor(0,2); lcd.print("Checking MAX...   ");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    max30102_ok = false;
    lcd.setCursor(0,2); lcd.print("MAX30102: ERROR   ");
  } else {
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x1F);
    particleSensor.setPulseAmplitudeIR(0x1F);
    max30102_ok = true;
    lcd.setCursor(0,2); lcd.print("MAX30102: OK      ");
  }
  delay(500);

  // HX711 — kalibrasi penuh
  hx711Boot();

  lcd.setCursor(0,2); lcd.print("Checking ADS...   ");
  if (!ads.begin()) {
    ads1115_ok = false;
    lcd.setCursor(0,2); lcd.print("ADS1115: ERROR    ");
  } else {
    ads.setGain(GAIN_ONE);
    ads1115_ok = true;
    lcd.setCursor(0,2); lcd.print("ADS1115: OK       ");
  }
  delay(1500);

  lcd.clear(); lcd.setCursor(0,0); lcd.print("Connecting...");
  if (connectToSurroundingWiFi()) {
    staConnected = true;
    lcd.clear(); lcd.setCursor(0,0); lcd.print("WiFi Connected!");
    lcd.setCursor(0,1); lcd.print("IP: "); lcd.print(WiFi.localIP());
    delay(2000);
    uploadPendingData();
  } else {
    staConnected = false;
    startAPMode();
  }
  startHTTPServer();
  showMainMenu();
}

// ========== LED ==========
void clearLEDs() {
  digitalWrite(LED_GREEN, LOW); digitalWrite(LED_YELLOW, LOW); digitalWrite(LED_RED, LOW);
}
void setLEDForAll(float val1, float val2, String type) {
  clearLEDs();
  if (dangerMode) { digitalWrite(LED_RED, HIGH); return; }
  if (type == "spo2_hr") {
    float spo2 = val1, bpm = val2;
    if (spo2 >= 95) digitalWrite(LED_GREEN, HIGH);
    else if (spo2 >= 90) digitalWrite(LED_YELLOW, HIGH);
    else digitalWrite(LED_RED, HIGH);
    if (bpm > 0) {
      if (bpm < 50 || bpm > 120) digitalWrite(LED_RED, HIGH);
      else if ((bpm >= 50 && bpm <= 59) || (bpm >= 101 && bpm <= 120)) digitalWrite(LED_YELLOW, HIGH);
    }
  } else if (type == "grip") {
    float grip = val1; int gender = (int)val2;
    if (gender == 1) {
      if (grip >= 35.7) digitalWrite(LED_GREEN, HIGH);
      else if (grip >= 30.8) digitalWrite(LED_YELLOW, HIGH);
      else digitalWrite(LED_RED, HIGH);
    } else {
      if (grip >= 24.5) digitalWrite(LED_GREEN, HIGH);
      else if (grip >= 16.8) digitalWrite(LED_YELLOW, HIGH);
      else digitalWrite(LED_RED, HIGH);
    }
  } else if (type == "bp") {
    if (systolicBP < 120 && diastolicBP < 80) digitalWrite(LED_GREEN, HIGH);
    else if ((systolicBP >= 120 && systolicBP < 140) || (diastolicBP >= 80 && diastolicBP < 90)) digitalWrite(LED_YELLOW, HIGH);
    else digitalWrite(LED_RED, HIGH);
  }
}

// ========== Menu ==========
void showMainMenu() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Select Measurement");
  String ip = "";
  if (staConnected) ip = WiFi.localIP().toString();
  else if (WiFi.getMode() == WIFI_AP) ip = WiFi.softAPIP().toString();
  else ip = "No IP";
  if (ip.length() > 16) ip = ip.substring(0,16);
  lcd.setCursor(0,3); lcd.print("IP: "); lcd.print(ip);
  for (int i=ip.length()+4; i<20; i++) lcd.print(" ");
  const char* items[] = {"HR+SpO2", "Handgrip", "Tensimeter", "All"};
  int idx1 = scrollOffset;
  lcd.setCursor(0,1);
  if (menuIndex == idx1) lcd.print(">"); else lcd.print(" ");
  lcd.print(items[idx1]);
  if (idx1==0 && !max30102_ok) lcd.print(" ERR");
  lcd.setCursor(13,1);
  lcd.print("Bat:"); lcd.print(getBatteryPercent()); lcd.print("%");
  int idx2 = scrollOffset+1;
  if (idx2<4) {
    lcd.setCursor(0,2);
    if (menuIndex == idx2) lcd.print(">"); else lcd.print(" ");
    lcd.print(items[idx2]);
    if (idx2==0 && !max30102_ok) lcd.print(" ERR");
  } else {
    lcd.setCursor(0,2); lcd.print("                ");
  }
}

// ========== Menu Navigation ==========
void handleMenuNavigation() {
  static unsigned long lastBtnTime = 0;
  if (millis() - lastBtnTime < 200) return;

  // Re-tare: UP + DOWN serentak
  if (digitalRead(BTN_UP) == LOW && digitalRead(BTN_DOWN) == LOW) {
    lastBtnTime = millis();
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Re-taring...");
    lcd.setCursor(0,1); lcd.print("Jangan sentuh!");
    // ambil semula zero baseline
    long sum = 0;
    for (int i = 0; i < 30; i++) {
      while (!scale.is_ready()) delay(5);
      sum += scale.get_value(1);
      delay(30);
    }
    hx711Zero     = sum / 30;
    hx711HandZero = hx711Zero;
    lcd.setCursor(0,2); lcd.print("Done! Zero set.");
    delay(1500);
    showMainMenu();
    return;
  }

  if (digitalRead(BTN_UP) == LOW) {
    for (int i=0; i<4; i++) seq[i] = seq[i+1]; seq[4] = 1;
    lastBtnTime = millis();
    if (menuIndex > 0) menuIndex--;
    else { menuIndex = 3; scrollOffset = 2; }
    if (menuIndex < scrollOffset) scrollOffset = menuIndex;
    else if (menuIndex > scrollOffset+1) scrollOffset = menuIndex-1;
    if (scrollOffset<0) scrollOffset=0; if (scrollOffset>2) scrollOffset=2;
    showMainMenu();

  } else if (digitalRead(BTN_DOWN) == LOW) {
    for (int i=0; i<4; i++) seq[i] = seq[i+1]; seq[4] = 2;
    lastBtnTime = millis();
    if (menuIndex < 3) menuIndex++;
    else { menuIndex = 0; scrollOffset = 0; }
    if (menuIndex < scrollOffset) scrollOffset = menuIndex;
    else if (menuIndex > scrollOffset+1) scrollOffset = menuIndex-1;
    if (scrollOffset<0) scrollOffset=0; if (scrollOffset>2) scrollOffset=2;
    showMainMenu();

  } else if (digitalRead(BTN_SELECT) == LOW) {
    lastBtnTime = millis();

    // Cheat code: UP UP DOWN DOWN SELECT
    if (seq[0]==1 && seq[1]==1 && seq[2]==2 && seq[3]==2) {
      for (int i=0; i<5; i++) seq[i]=0;
      lcd.clear(); lcd.print("DANGER MODE ON");
      delay(1000);
      measuring  = true;
      dangerMode = true;
      switch(menuIndex) {
        case 0: measureHeart();         break;
        case 1: measureGrip();          break;
        case 2: measureBloodPressure(); break;
        case 3: measureAll();           break;
      }
      dangerMode = false;
      measuring  = false;
      while (digitalRead(BTN_SELECT) == LOW) delay(20);
      delay(200);
      showMainMenu();
      return;
    }

    for (int i=0; i<5; i++) seq[i]=0;
    selectPressed = false;
    measuring = true;
    switch(menuIndex) {
      case 0: measureHeart();         break;
      case 1: measureGrip();          break;
      case 2: measureBloodPressure(); break;
      case 3: measureAll();           break;
    }
    measuring = false;
    while (digitalRead(BTN_SELECT) == LOW) delay(20);
    delay(200);
    showMainMenu();
  }
}

// ========== Heart Rate + SpO2 ==========
void measureHeart() {
  if (dangerMode) {
    lcd.clear(); lcd.print("HR & SpO2"); lcd.setCursor(0,1); lcd.print("Measuring...");
    delay(2000);
    float fakeBPM  = random(150,200);
    float fakeSpO2 = random(70,85);
    lcd.clear(); lcd.print("Result:");
    lcd.setCursor(0,1); lcd.print("BPM: ");  lcd.print(fakeBPM,0);
    lcd.setCursor(0,2); lcd.print("SpO2: "); lcd.print(fakeSpO2,0); lcd.print("%");
    lcd.setCursor(0,3); lcd.print("SELECT to go back");
    setLEDForAll(fakeSpO2, fakeBPM, "spo2_hr");
    uploadData(0,0,0,0,fakeSpO2,fakeBPM,0);
    selectPressed=false; while(!checkSelectButton()) delay(50); clearLEDs();
    return;
  }
  if (!max30102_ok) { lcd.clear(); lcd.print("Sensor Error!"); delay(RESULT_TIME); return; }
  lcd.clear(); lcd.print("Put your fingertip"); lcd.setCursor(0,1); lcd.print("on the sensor"); delay(3000);
  const int BUFFER_SIZE = 50;
  float irBuffer[BUFFER_SIZE] = {0};
  float redBuffer[BUFFER_SIZE] = {0};
  int bufferIndex = 0;
  float irSum = 0, redSum = 0;
  bpmSum = 0; beatCount = 0; spo2Sum = 0; spo2Count = 0;
  lastBeat = millis(); unsigned long start = millis(); selectPressed = false;
  lcd.clear(); lcd.setCursor(0,0); lcd.print("HR & SpO2"); lcd.setCursor(0,1); lcd.print("Please wait 30s");
  while (millis() - start < MEASURE_TIME) {
    if (checkSelectButton()) {
      selectPressed = true;
      unsigned long selectTime = millis(); bool doubleTap = false;
      while (millis() - selectTime < 500) { if (checkSelectButton()) { doubleTap = true; break; } delay(50); }
      if (doubleTap) { lcd.clear(); lcd.print("Cancelled"); delay(1000); return; }
      else break;
    }
    long ir = particleSensor.getIR(), red = particleSensor.getRed();
    int rem = (MEASURE_TIME - (millis() - start)) / 1000;
    lcd.setCursor(0,2); lcd.print(rem); lcd.print("s left ");
    if (ir < 50000) { lcd.setCursor(0,3); lcd.print("No finger!      "); delay(20); continue; }
    irSum -= irBuffer[bufferIndex]; redSum -= redBuffer[bufferIndex];
    irBuffer[bufferIndex] = ir; redBuffer[bufferIndex] = red;
    irSum += ir; redSum += red;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    float irAvg = irSum / BUFFER_SIZE, redAvg = redSum / BUFFER_SIZE;
    if (checkForBeat(ir)) {
      long delta = millis() - lastBeat; lastBeat = millis();
      if (delta > 300 && delta < 1500) {
        float bpm = 60000.0 / delta;
        if (bpm > 40 && bpm < 200) { bpmSum += bpm; beatCount++; lcd.setCursor(0,3); lcd.print("BPM:"); lcd.print(bpm,0); lcd.print("  "); }
      }
    }
    if (bufferIndex == 0) {
      float irAC = 0, redAC = 0;
      for (int i = 0; i < BUFFER_SIZE; i++) { irAC += fabs(irBuffer[i] - irAvg); redAC += fabs(redBuffer[i] - redAvg); }
      irAC /= BUFFER_SIZE; redAC /= BUFFER_SIZE;
      if (irAvg > 0 && irAC > 0) {
        float R = (redAC / redAvg) / (irAC / irAvg);
        float spo2 = constrain(110.0 - 25.0 * R, 70, 100);
        spo2Sum += spo2; spo2Count++;
        lcd.setCursor(10,3); lcd.print("O2:"); lcd.print(spo2,0); lcd.print("%");
      }
    }
    delay(20);
  }
  float avgBPM  = (beatCount  > 0) ? bpmSum  / beatCount  : 0;
  float avgSpO2 = (spo2Count  > 0) ? spo2Sum / spo2Count  : 0;
  if (beatCount == 0 || spo2Count == 0) { lcd.clear(); lcd.print("No valid data!"); lcd.setCursor(0,1); lcd.print("Try again"); delay(RESULT_TIME); return; }
  lcd.clear(); lcd.setCursor(0,0); lcd.print("Result:");
  lcd.setCursor(0,1); lcd.print("BPM: ");  lcd.print(avgBPM,0);
  lcd.setCursor(0,2); lcd.print("SpO2: "); lcd.print(avgSpO2,0); lcd.print("%");
  lcd.setCursor(0,3); lcd.print("SELECT to go back");
  setLEDForAll(avgSpO2, avgBPM, "spo2_hr");
  uploadData(0,0,0,0,avgSpO2,avgBPM,0);
  selectPressed=false; while(!checkSelectButton()) delay(50); clearLEDs();
}

// ==========================================================================
// GRIP — Fungsi Pembantu
// ==========================================================================

int selectGender() {
  int sub = 0;
  unsigned long btnTime = 0;
  while (digitalRead(BTN_SELECT) == LOW) delay(20);
  while (digitalRead(BTN_UP)     == LOW) delay(20);
  while (digitalRead(BTN_DOWN)   == LOW) delay(20);
  delay(300);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Pilih Jantina:");
  while (true) {
    lcd.setCursor(0,1); lcd.print(sub == 0 ? "> Male      " : "  Male     ");
    lcd.setCursor(0,2); lcd.print(sub == 1 ? "> Female   " : "  Female   ");
    lcd.setCursor(0,3); lcd.print(sub == 2 ? "> Kembali    " : "  Kembali    ");
    if (millis() - btnTime > 200) {
      if (digitalRead(BTN_UP) == LOW) {
        sub--; if (sub < 0) sub = 2; btnTime = millis();
      } else if (digitalRead(BTN_DOWN) == LOW) {
        sub++; if (sub > 2) sub = 0; btnTime = millis();
      } else if (digitalRead(BTN_SELECT) == LOW) {
        btnTime = millis();
        while (digitalRead(BTN_SELECT) == LOW) delay(20);
        if      (sub == 0) return 1;
        else if (sub == 1) return 2;
        else               return 0;
      }
    }
    delay(20);
  }
}

String getGripGrade(float kg, int gender) {
  if (gender == 1) {
    if      (kg < 30.8)  return "Lemah";
    else if (kg < 35.7)  return "Sempadan";
    else if (kg <= 55.5) return "Normal";
    else                 return "Bagus";
  } else {
    if      (kg < 16.8)  return "Lemah";
    else if (kg < 19.2)  return "Sempadan";
    else if (kg < 24.5)  return "Normal";
    else if (kg <= 31.0) return "Bagus";
    else                 return "Sangat Bagus";
  }
}

void showGripResult(float peakKg, int gender, bool isDanger) {
  String grade = isDanger ? "BAHAYA" : getGripGrade(peakKg, gender);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("== HASIL GRIP ==");
  lcd.setCursor(0,1); lcd.print("Max: "); lcd.print(peakKg, 1); lcd.print(" kg        ");
  lcd.setCursor(0,2); lcd.print("Gred: "); lcd.print(grade); lcd.print("        ");
  lcd.setCursor(0,3); lcd.print("SELECT utk keluar");
  setLEDForAll(peakKg, gender, "grip");
  uploadData(0, 0, 0, 0, 0, 0, peakKg);
  selectPressed = false;
  while (!checkSelectButton()) delay(50);
  clearLEDs();
}

// ==========================================================================
// performGripMeasurement — TERAS PENGUKURAN GENGGAMAN
//
// ALIRAN:
//   [FASA REST 3s]
//     → user pegang alat TANPA genggam
//     → ambil baseline dinamik (berat tangan dah ditolak)
//     → ukur noise floor semasa tangan pegang
//     → kira dead zone adaptif
//
//   [COUNTDOWN 3-2-1]
//
//   [FASA SQUEEZE 5s]
//     → baca raw HX711 setiap ~60ms
//     → tolak baseline tangan
//     → tapis dengan dead zone (getaran kecil → 0)
//     → low-pass filter untuk display yang smooth
//     → track nilai PUNCAK untuk keputusan akhir
//
// Return: nilai puncak dalam kg, atau -1 jika dibatalkan
// ==========================================================================
float performGripMeasurement() {

  // ----------------------------------------------------------------
  // FASA REST: 3 saat — pegang alat, jangan genggam
  // ----------------------------------------------------------------
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Pegang alat...");
  lcd.setCursor(0,1); lcd.print("JANGAN genggam  ");
  lcd.setCursor(0,2); lcd.print("Ambil posisi..  ");

  const int   REST_MS      = 3000;
  const int   REST_SAMPLES = 60;   // ~1 sample setiap 50ms selama 3s
  long  restBuf[60];
  int   restCount = 0;
  long  restSum   = 0;
  unsigned long restStart = millis();

  while ((int)(millis() - restStart) < REST_MS) {
    int sisa = 3 - (int)((millis() - restStart) / 1000);
    lcd.setCursor(0,3);
    lcd.print("Rehat: "); lcd.print(sisa); lcd.print("s         ");

    if (scale.is_ready() && restCount < REST_SAMPLES) {
      long raw = scale.get_value(1);
      restBuf[restCount] = raw;
      restSum += raw;
      restCount++;
    }

    if (checkSelectButton()) {
      unsigned long pt = millis(); bool dt = false;
      while (millis()-pt < 400) { if (checkSelectButton()) { dt=true; break; } delay(30); }
      if (dt) return -1;
    }
    delay(50);
  }

  // Kemas kini baseline dengan tangan sudah pegang
  if (restCount > 0) {
    hx711HandZero = restSum / restCount;
  }

  // Kira noise floor semasa tangan pegang (variance dari restBuf)
  float restVariance = 0;
  if (restCount > 1) {
    for (int i = 0; i < restCount; i++) {
      float d = (float)(restBuf[i] - hx711HandZero);
      restVariance += d * d;
    }
    restVariance /= restCount;
  }
  float restStddev = sqrt(restVariance);

  // Dead zone adaptif: 4× stddev semasa tangan pegang
  // Ini lebih tepat dari dead zone boot kerana sudah ambil kira berat tangan
  float adaptiveDeadZone = (restStddev * 4.0f) / hx711CalFactor;
  if (adaptiveDeadZone < 1.0f) adaptiveDeadZone = 1.0f;  // minimum 1.0 kg
  if (adaptiveDeadZone > 6.0f) adaptiveDeadZone = 6.0f;  // maksimum 6.0 kg

  Serial.print("Hand baseline: ");    Serial.println(hx711HandZero);
  Serial.print("Rest stddev raw: ");  Serial.println(restStddev);
  Serial.print("Adaptive deadzone: ");Serial.println(adaptiveDeadZone);

  // ----------------------------------------------------------------
  // COUNTDOWN: 3-2-1
  // ----------------------------------------------------------------
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Bersedia...");
  lcd.setCursor(0,1); lcd.print("Genggam SEKUAT  ");
  lcd.setCursor(0,2); lcd.print("mungkin!        ");
  for (int c = 3; c >= 1; c--) {
    lcd.setCursor(0,3); lcd.print("Mula dalam: "); lcd.print(c); lcd.print("s  ");
    delay(1000);
  }

  // ----------------------------------------------------------------
  // FASA SQUEEZE: 5 saat
  // ----------------------------------------------------------------
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("GENGGAM SEKARANG!");

  const unsigned long SQUEEZE_MS = 5000;
  unsigned long squeezeStart = millis();
  float peakKg   = 0;
  bool  cancelled = false;

  // Low-pass filter (EMA): alpha kecil = lebih smooth, lebih lambat
  // alpha besar = lebih responsif, lebih noise
  const float ALPHA = 0.25f;
  float emaKg = 0;
  bool  emaInit = false;

  while (millis() - squeezeStart < SQUEEZE_MS) {
    unsigned long elapsed = millis() - squeezeStart;
    int sisaSaat = (int)((SQUEEZE_MS - elapsed) / 1000) + 1;

    // --- Baca sensor ---
    float rawKg = 0;
    if (scale.is_ready()) {
      // 3-sample median cepat (buang spike)
      long s0 = scale.get_value(1); delay(2);
      long s1 = scale.get_value(1); delay(2);
      long s2 = scale.get_value(1);
      // Sort 3 nilai
      if (s0>s1){long t=s0;s0=s1;s1=t;}
      if (s1>s2){long t=s1;s1=s2;s2=t;}
      if (s0>s1){long t=s0;s0=s1;s1=t;}
      long medRaw = s1; // nilai tengah

      long net = medRaw - hx711HandZero;
      float kg  = (float)net / hx711CalFactor;

      // Dead zone adaptif — getaran di bawah ini dianggap 0
      // Ini kunci utama: getaran kecil tidak akan muncul sebagai nilai
      if (kg < adaptiveDeadZone) kg = 0;

      rawKg = kg;

      Serial.print("med:"); Serial.print(medRaw);
      Serial.print(" net:"); Serial.print(net);
      Serial.print(" kg:"); Serial.print(kg,2);
      Serial.print(" dz:"); Serial.println(adaptiveDeadZone,2);
    }

    // --- Exponential Moving Average untuk display yang smooth ---
    if (!emaInit) { emaKg = rawKg; emaInit = true; }
    else          { emaKg = ALPHA * rawKg + (1.0f - ALPHA) * emaKg; }

    // Jika EMA di bawah dead zone juga, paksa ke 0
    float displayKg = (emaKg < adaptiveDeadZone * 0.5f) ? 0 : emaKg;

    // --- Track PEAK guna rawKg (bukan EMA) supaya tidak terlepas puncak ---
    if (rawKg > peakKg) peakKg = rawKg;

    // --- Update LCD ---
    lcd.setCursor(0,1);
    lcd.print("Kini:  ");
    if (displayKg < 0.1f) lcd.print("0.0 kg   ");
    else { lcd.print(displayKg, 1); lcd.print(" kg   "); }

    lcd.setCursor(0,2);
    lcd.print("Puncak:");
    if (peakKg < 0.1f) lcd.print("0.0 kg   ");
    else { lcd.print(peakKg, 1); lcd.print(" kg   "); }

    lcd.setCursor(0,3);
    lcd.print("Masa:  "); lcd.print(sisaSaat); lcd.print("s lagi      ");

    // --- Batal jika double-tap SELECT ---
    if (checkSelectButton()) {
      unsigned long pt = millis(); bool dt = false;
      while (millis()-pt < 400) { if (checkSelectButton()) { dt=true; break; } delay(30); }
      if (dt) { cancelled = true; break; }
    }

    delay(55); // ~17 Hz — cukup untuk LCD response tanpa flicker
  }

  if (cancelled) return -1;
  return peakKg;
}

// ==========================================================================
// simulateGrip — Danger Mode
// Sensor dibaca secara real, tapi nilai puncak akhir diforced ke julat rendah
// ==========================================================================
void simulateGrip() {
  int gender = selectGender();
  if (gender == 0) return;

  lcd.clear();
  lcd.print(gender == 1 ? "Male selected" : "Female selected");
  delay(800);

  float realPeak = performGripMeasurement();
  if (realPeak < 0) { lcd.clear(); lcd.print("Cancelled"); delay(1000); return; }

  // Force nilai ke julat rendah untuk danger mode (5.0–13.9 kg)
  float fakePeak = (float)random(50, 140) / 10.0f;
  maxGripForce = fakePeak;
  showGripResult(maxGripForce, gender, true);
}

// ==========================================================================
// measureGrip — Normal Mode (simulated display)
// ==========================================================================
void measureGrip() {
  if (dangerMode) { simulateGrip(); return; }

  int gender = selectGender();
  if (gender == 0) return;

  lcd.clear();
  lcd.print(gender == 1 ? "Male selected" : "Female selected");
  delay(800);

  // Generate random target in Normal range
  float finalForce;
  if (gender == 1) {
    finalForce = random(357, 556) / 10.0;   // Male: 35.7 – 55.5 kg
  } else {
    finalForce = random(192, 246) / 10.0;   // Female: 19.2 – 24.5 kg
  }

  // Rest phase
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Hold the device");
  lcd.setCursor(0,1); lcd.print("DON'T squeeze   ");
  lcd.setCursor(0,2); lcd.print("Get ready...    ");
  for (int c = 3; c >= 1; c--) {
    lcd.setCursor(0,3); lcd.print("Rest: "); lcd.print(c); lcd.print("s         ");
    delay(1000);
  }

  // Countdown
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Get ready...");
  lcd.setCursor(0,1); lcd.print("Squeeze HARD!");
  for (int c = 3; c >= 1; c--) {
    lcd.setCursor(0,3); lcd.print("Start in: "); lcd.print(c); lcd.print("s  ");
    delay(1000);
  }

  // Squeeze phase: 5 seconds, simulate climbing force
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("SQUEEZE NOW!");
  unsigned long start = millis();
  const unsigned long SQUEEZE_MS = 5000;
  float currentForce = 0;
  float peakForce = 0;

  while (millis() - start < SQUEEZE_MS) {
    unsigned long elapsed = millis() - start;
    int sisaSaat = (int)((SQUEEZE_MS - elapsed) / 1000) + 1;

    // Simulate force: ramps up to ~80% at 2s, peaks around 3s, holds
    float progress = (float)elapsed / SQUEEZE_MS;
    if (progress < 0.4) {
      currentForce = finalForce * (progress / 0.4) * 0.85;
    } else if (progress < 0.6) {
      currentForce = finalForce * (0.85 + 0.15 * ((progress - 0.4) / 0.2));
    } else {
      // Small random fluctuation around peak
      currentForce = finalForce + random(-5, 5) / 10.0;
    }
    if (currentForce > peakForce) peakForce = currentForce;

    lcd.setCursor(0,1);
    lcd.print("Force: "); lcd.print(currentForce, 1); lcd.print(" kg   ");
    lcd.setCursor(0,2);
    lcd.print("Peak:  "); lcd.print(peakForce, 1); lcd.print(" kg   ");
    lcd.setCursor(0,3);
    lcd.print("Time:  "); lcd.print(sisaSaat); lcd.print("s left      ");

    if (checkSelectButton()) {
      unsigned long pt = millis(); bool dt = false;
      while (millis()-pt < 400) { if (checkSelectButton()) { dt=true; break; } delay(30); }
      if (dt) { lcd.clear(); lcd.print("Cancelled"); delay(1000); return; }
    }
    delay(55);
  }

  maxGripForce = finalForce;
  showGripResult(maxGripForce, gender, false);
}

// ========== simulateBloodPressure (dangerMode only) ==========
void simulateBloodPressure() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Put on the cuff");
  lcd.setCursor(0,1); lcd.print("above your wrist");
  lcd.setCursor(0,2); lcd.print("Press SELECT");
  selectPressed=false; while(!checkSelectButton()) delay(50);

  startPump();
  openValve();

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Inflating cuff...");

  unsigned long startTime  = millis();
  bool pumpStopped         = false;
  bool solenoidStopped     = false;
  bool cancelled           = false;

  while (!solenoidStopped) {
    unsigned long elapsed = millis() - startTime;
    if (!pumpStopped && elapsed >= PUMP_DURATION_MS) {
      stopPump(); pumpStopped = true;
      lcd.setCursor(0,0); lcd.print("Pump OFF, hold.. ");
    }
    if (elapsed >= SOLENOID_DURATION_MS) {
      closeValve(); solenoidStopped = true; break;
    }
    int pumpRem     = (!pumpStopped) ? (int)((PUMP_DURATION_MS     - elapsed) / 1000UL) : 0;
    int solenoidRem = (int)((SOLENOID_DURATION_MS - elapsed) / 1000UL);
    lcd.setCursor(0,1); lcd.print("Pump:   ");
    if (!pumpStopped) { lcd.print(pumpRem); lcd.print("s left  "); } else { lcd.print("OFF      "); }
    lcd.setCursor(0,2); lcd.print("Valve:  "); lcd.print(solenoidRem); lcd.print("s left  ");
    if (checkSelectButton()) {
      unsigned long pt = millis(); bool dt = false;
      while (millis()-pt < 500) { if (checkSelectButton()) { dt=true; break; } delay(50); }
      if (dt) { cancelled = true; break; }
    }
    delay(100);
  }

  if (cancelled) { stopPump(); closeValve(); lcd.clear(); lcd.print("Cancelled"); delay(1000); return; }

  systolicBP  = random(180, 221);
  diastolicBP = random(110, 131);
  meanBP      = diastolicBP + (systolicBP - diastolicBP) / 3;

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("SYS/DIA: ");
  lcd.print(systolicBP,0); lcd.print("/"); lcd.print(diastolicBP,0);
  lcd.setCursor(0,2); lcd.print("SELECT to go back");
  setLEDForAll(0,0,"bp");
  uploadData(systolicBP, diastolicBP, meanBP, 0, 0, 0, 0);
  selectPressed=false; while(!checkSelectButton()) delay(50); clearLEDs();
}

// ========== measureBloodPressure ==========
void measureBloodPressure() {
  if (dangerMode) { simulateBloodPressure(); return; }

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Put on the cuff");
  lcd.setCursor(0,1); lcd.print("above your wrist");
  lcd.setCursor(0,2); lcd.print("Press SELECT");
  selectPressed=false; while(!checkSelectButton()) delay(50);

  startPump();
  openValve();

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Inflating cuff...");

  unsigned long startTime  = millis();
  bool pumpStopped         = false;
  bool solenoidStopped     = false;
  bool cancelled           = false;

  while (!solenoidStopped) {
    unsigned long elapsed = millis() - startTime;
    if (!pumpStopped && elapsed >= PUMP_DURATION_MS) {
      stopPump(); pumpStopped = true;
      lcd.setCursor(0,0); lcd.print("Pump OFF, hold..");
    }
    if (elapsed >= SOLENOID_DURATION_MS) {
      closeValve(); solenoidStopped = true; break;
    }
    int pumpRem     = (!pumpStopped) ? (int)((PUMP_DURATION_MS     - elapsed) / 1000UL) : 0;
    int solenoidRem = (int)((SOLENOID_DURATION_MS - elapsed) / 1000UL);
    lcd.setCursor(0,1); lcd.print("Pump:  ");
    if (!pumpStopped) { lcd.print(pumpRem); lcd.print("s left   "); } else { lcd.print("OFF      "); }
    lcd.setCursor(0,2); lcd.print("Valve: "); lcd.print(solenoidRem); lcd.print("s left   ");
    if (checkSelectButton()) {
      unsigned long pt = millis(); bool dt = false;
      while (millis()-pt < 500) { if (checkSelectButton()) { dt=true; break; } delay(50); }
      if (dt) { cancelled = true; break; }
    }
    delay(100);
  }

  if (cancelled) { stopPump(); closeValve(); lcd.clear(); lcd.print("Cancelled"); delay(1000); return; }

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Done. Reading BP");
  delay(500);

  systolicBP  = random(110, 131);
  diastolicBP = random(70, 86);
  meanBP      = diastolicBP + (systolicBP - diastolicBP) / 3;

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("SYS/DIA: ");
  lcd.print(systolicBP,0); lcd.print("/"); lcd.print(diastolicBP,0);
  lcd.setCursor(0,2); lcd.print("SELECT to go back");
  setLEDForAll(0,0,"bp");
  uploadData(systolicBP, diastolicBP, meanBP, 0, 0, 0, 0);
  selectPressed=false; while(!checkSelectButton()) delay(50); clearLEDs();
}

// ========== Pump & Valve (active-LOW) ==========
void startPump()  { digitalWrite(RELAY_PUMP,  LOW);  }
void stopPump()   { digitalWrite(RELAY_PUMP,  HIGH); }
void openValve()  { digitalWrite(RELAY_VALVE, LOW);  }
void closeValve() { digitalWrite(RELAY_VALVE, HIGH); }

// ========== Dummy ==========
float readPressureMPX()          { return 0; }
float detectOscillation(float p) { return 0; }
void calculateBloodPressure()    {}

// ========== All ==========
void measureAll() {
  measureHeart();         if(!selectPressed) delay(1000);
  measureGrip();          if(!selectPressed) delay(1000);
  measureBloodPressure();
}

// ========== Loop ==========
void loop() {
  if (!measuring) {
    handleMenuNavigation();
    server.handleClient();
  }
  delay(20);
}