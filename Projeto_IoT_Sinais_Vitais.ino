// ------------------ Projeto Monitor Vital IoT ------------------
#define BLYNK_TEMPLATE_ID "TMPL2KnVpgXl3"
#define BLYNK_TEMPLATE_NAME "Projeto Sinais Vitais Com IoT"
#define BLYNK_AUTH ""//DKP9s8MEKGbsdo3mXpV-nc1RecDTkzXm"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MAX30105.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>

// ------------------ Configurações ------------------
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDR 0x3C
#define I2C_SDA 6
#define I2C_SCL 7

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
MAX30105 particleSensor;

#define WIFI_SSID "Manzotti"
#define WIFI_PASS "Gabriel1402"

//#define WIFI_SSID "iPhone de Felipe"
//#define WIFI_PASS "testando"

BlynkTimer timer;

// ------------------ Variáveis ------------------
const long MIN_IR_VALUE = 5000;
const float alphaIR = 0.1;
float irEMA = 0.0;

unsigned long lastPeakTime = 0;
float bpmFiltered = 0.0;
const float alphaBPM = 0.3;

#define BUFFER_SIZE 100
int redBuffer[BUFFER_SIZE];
int irBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

bool wifiConnected = false;
bool blynkConnected = false;

// ------------------ Funções ------------------
void displaySplashScreen();
void displayUpdateVitals(float bpm, float spo2, bool fingerDetected);
void sensorReadOnce(long &irValue, long &redValue);
void processPeakAndBpm(long irValue);
float calculateSpO2_simple(int redBuf[], int irBuf[], int samples);
void sendToBlynk();
void checkConnectionStatus();

// ------------------ Setup ------------------
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n[Iniciando Monitor Vital IoT]");

  // Inicializa I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("Barramento I2C iniciado.");

  // Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("Erro: display SSD1306 não encontrado!");
    while (1);
  }

  // Tela inicial
  displaySplashScreen();

  // Sensor MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("Erro: MAX30102 não encontrado!");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Sensor nao");
    display.setCursor(0,20);
    display.println("encontrado");
    display.display();
    while (1);
  }

  particleSensor.setup(0x1F, 4, 3, 100, 411, 4096);
  particleSensor.enableDIETEMPRDY();
  Serial.println("Sensor MAX30102 iniciado com sucesso!");

// Indicador de Bateria

  pinMode(A0, INPUT);

  // WiFi (não bloqueante)
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando WiFi...");

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 5000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\nWiFi conectado!");
  } else {
    wifiConnected = false;
    Serial.println("\nWiFi não conectado (modo offline).");
  }

  // Configura Blynk (sem travar)
  Blynk.config(BLYNK_AUTH);
  if (wifiConnected) {
    Blynk.connect(1000);
    blynkConnected = Blynk.connected();
  }

  timer.setInterval(1000L, sendToBlynk);
  timer.setInterval(3000L, checkConnectionStatus);

  display.clearDisplay();
  display.display();
}

// ------------------ Loop ------------------
void loop() {
  Blynk.run();
  timer.run();

  long irValue, redValue;
  sensorReadOnce(irValue, redValue);

  bool fingerDetected = (irEMA > MIN_IR_VALUE);

  if (fingerDetected) {
    redBuffer[bufferIndex] = (int)redValue;
    irBuffer[bufferIndex] = (int)irValue;
    bufferIndex++;
    if (bufferIndex >= BUFFER_SIZE) {
      bufferIndex = 0;
      bufferFilled = true;
    }
    processPeakAndBpm(irValue);
  } else {
    bpmFiltered = 0;
    lastPeakTime = 0;
    bufferIndex = 0;
    bufferFilled = false;
  }

  float spo2 = 0.0f;
  if (bufferFilled) spo2 = calculateSpO2_simple(redBuffer, irBuffer, BUFFER_SIZE);

  // Atualiza display
  static unsigned long lastDisplayMs = 0;
  if (millis() - lastDisplayMs > 500) {
    displayUpdateVitals(bpmFiltered, spo2, fingerDetected);
    lastDisplayMs = millis();
  }

  delay(10);
}

// ------------------ Indicador de Bateria ------------------
float readBatteryVoltage() {
  uint32_t Vbatt = 0;

  for (int v = 0; v < 16; v++) {
    Vbatt += analogReadMilliVolts(A0);
  }

  float Vbattf = 2 * (Vbatt / 16.0) / 1000.0;  // mV -> V
  return Vbattf; 
}
  
// ------------------ Display ------------------

// Mostra somente no início (2 segundos)
void displaySplashScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Monitor");
  display.setCursor(0, 20);
  display.println("Vital");
  display.setCursor(80, 40);
  display.println("IoT");
  display.display();
  delay(2000);
}

void displayUpdateVitals(float bpm, float spo2, bool fingerDetected) {
  display.clearDisplay();

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("BPM: ");
  if (fingerDetected && bpm > 0.0f) display.print((int)(bpm + 0.5));
  else display.print("--");

  display.setCursor(0, 28);
  display.print("SpO2: ");
if (fingerDetected && spo2 > 0.0f) {
  display.print((int)(spo2 + 0.5));
  display.print("%");
} else {
  display.print("--%");
}

  display.setTextSize(1);
  display.setCursor(0, 56);
  if (wifiConnected && blynkConnected)
    display.print("Status: ONLINE");
  else if (wifiConnected)
    display.print("Status: WIFI OK");
  else
    display.print("Status: OFFLINE");

// Logo Bateria
//id: 0 rect 26 
display.drawRect(105, 1, 22, 12, SSD1306_WHITE);
//id: 1 line 34 
display.drawLine(106, 11, 116, 1, SSD1306_WHITE);
//id: 2 fillRect 35 
display.fillRect(115, 2, 12, 10, SSD1306_WHITE);
//id: 3 fillTriangle 36 
display.fillTriangle(105, 12, 115, 2, 115, 12, SSD1306_WHITE);
//id: 4 fillRect 39 
display.fillRect(103, 4, 3, 6, SSD1306_WHITE);

float voltage = readBatteryVoltage();
display.setCursor(97, 15);
display.print(voltage, 2);
display.print("v");

  display.display();
}

// ------------------ Sensor ------------------
void sensorReadOnce(long &irValue, long &redValue) {
  irValue = particleSensor.getIR();
  redValue = particleSensor.getRed();

  if (irEMA == 0.0f)
    irEMA = (float)irValue;
  else
    irEMA = alphaIR * (float)irValue + (1.0f - alphaIR) * irEMA;
}

void processPeakAndBpm(long irValue) {
  static long prevIR = 0, prevPrevIR = 0;
  unsigned long now = millis();

  if (prevIR > prevPrevIR && prevIR > irValue && prevIR > MIN_IR_VALUE) {
    if (lastPeakTime == 0) {
      lastPeakTime = now;
      return;
    }

    unsigned long interval = now - lastPeakTime;
    lastPeakTime = now;

    if (interval < 300 || interval > 2000) return;

    float bpmInstant = 60.0f / (interval / 1000.0f);
    if (bpmFiltered == 0.0f)
      bpmFiltered = bpmInstant;
    else
      bpmFiltered = alphaBPM * bpmInstant + (1.0f - alphaBPM) * bpmFiltered;
  }

  prevPrevIR = prevIR;
  prevIR = irValue;
}

// ------------------ SpO2 ------------------
float calculateSpO2_simple(int redBuf[], int irBuf[], int samples) {
  long sumRed=0, sumIr=0;
  int rMin = INT_MAX, rMax = 0, iMin = INT_MAX, iMax = 0;

  for (int i=0;i<samples;i++){
    int r = redBuf[i], ir = irBuf[i];
    if (r < rMin) rMin = r;
    if (r > rMax) rMax = r;
    if (ir < iMin) iMin = ir;
    if (ir > iMax) iMax = ir;
    sumRed += r;
    sumIr += ir;
  }

  int redAC = rMax - rMin;
  int irAC = iMax - iMin;
  float redDC = (float)sumRed / samples;
  float irDC = (float)sumIr / samples;

  if (redDC == 0 || irDC == 0 || irAC == 0) return 0.0f;

  float R = ((float)redAC / redDC) / ((float)irAC / irDC);
  float SpO2 = 110.0f - 25.0f * R;

  if (SpO2 > 100.0f) SpO2 = 100.0f;
  if (SpO2 < 0.0f) SpO2 = 0.0f;
  return SpO2;
}

// ------------------ Blynk ------------------
void sendToBlynk() {
  if (Blynk.connected()) {
    Blynk.virtualWrite(V1, (int)(bpmFiltered + 0.5));

    float spo2 = 0.0f;
    if (bufferFilled)
      spo2 = calculateSpO2_simple(redBuffer, irBuffer, BUFFER_SIZE);

    Blynk.virtualWrite(V2, spo2);
  }
}

// ------------------ Conexão ------------------
void checkConnectionStatus() {
  // Verifica WiFi
  wifiConnected = (WiFi.status() == WL_CONNECTED);

  // Tenta reconectar Blynk se possível
  if (wifiConnected && !Blynk.connected()) {
    Blynk.connect(1000);
  }
  blynkConnected = Blynk.connected();

  Serial.print("WiFi: ");
  Serial.print(wifiConnected ? "OK" : "OFF");
  Serial.print(" | Blynk: ");
  Serial.println(blynkConnected ? "OK" : "OFF");
}
