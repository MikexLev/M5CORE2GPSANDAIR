#include <M5Core2.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <cactus_io_BME280_I2C.h>
#include <MiCS6814-I2C.h>

BME280_I2C bme(0x76);  //i2C PA_SDA 32,PA_SCL 33
TinyGPSPlus gps;

double homeLat = 0.0;
double homeLon = 0.0;
static const int RXPin = 13, TXPin = 14;
static const uint32_t GPSBaud = 9600;

const int geigerPin = 26;               // GPIO für Geigerzähler
volatile unsigned long pulseCount = 0;  // Impulszähler
volatile bool drawBitmapFlag = false;   // Flag, um Bitmap zu zeichnen
// Bitmap-Daten
const unsigned short pngegg[484] PROGMEM = {
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0010 (16) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0020 (32) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0030 (48) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x20E4, 0x20E4, 0x20E4, 0x20E4, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0040 (64) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x20E4, 0x39C3, 0x8C02, 0xB542, 0xCE21, 0xC5C1, 0xB542, 0x83C2,   // 0x0050 (80) pixels
0x20E4, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x20E4, 0x7B83, 0xE6C1,   // 0x0060 (96) pixels
0xFF80, 0xFF80, 0xFF80, 0xFF80, 0xFF80, 0xFF80, 0xF720, 0x7BA2, 0x20E4, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0070 (112) pixels
0x0000, 0x0000, 0x20E4, 0x9462, 0x9442, 0x5243, 0xFF80, 0xFF80, 0xFF80, 0xFF80, 0xFF80, 0xF720, 0x5263, 0x9CA2, 0x83E3, 0x20E4,   // 0x0080 (128) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x20E4, 0x9C81, 0x9CA1, 0x20E4, 0x20E4, 0xAD01, 0xFF80, 0xFF80, 0xFF80,   // 0x0090 (144) pixels
0xFF80, 0x9CA2, 0x20E4, 0x20E4, 0xAD01, 0x7342, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x39C3, 0xB562, 0x3143,   // 0x00A0 (160) pixels
0x20E4, 0x20E4, 0x3163, 0xFF80, 0xFF80, 0xFF80, 0xF720, 0x3163, 0x20E4, 0x20E4, 0x20E4, 0xD641, 0x20E4, 0x0000, 0x0000, 0x0000,   // 0x00B0 (176) pixels
0x0000, 0x0000, 0x0000, 0x8C22, 0x6B02, 0x20E4, 0x20E4, 0x20E4, 0x20E4, 0x83E2, 0xFF80, 0xFF80, 0x8402, 0x20E4, 0x20E4, 0x20E4,   // 0x00C0 (192) pixels
0x20E4, 0x7362, 0x7B82, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x20E4, 0xBD81, 0x39A4, 0x20E4, 0x20E4, 0x20E4, 0x20E4, 0x8C02,   // 0x00D0 (208) pixels
0x9442, 0x8C02, 0x9CA2, 0x20E4, 0x20E4, 0x20E4, 0x20E4, 0x3163, 0xB542, 0x20E4, 0x0000, 0x0000, 0x0000, 0x0000, 0x20E4, 0xAD01,   // 0x00E0 (224) pixels
0x20E4, 0x20E4, 0x20E4, 0x20E4, 0x4A23, 0x83C2, 0x20E4, 0x20E4, 0x83E3, 0x39C3, 0x20E4, 0x20E4, 0x20E4, 0x20E4, 0xB561, 0x20E4,   // 0x00F0 (240) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x20E4, 0xCE01, 0xE6C0, 0xE6C0, 0xE6C0, 0xE6C0, 0xE6C0, 0x7BA3, 0x20E4, 0x20E4, 0x7BA3, 0xF720,   // 0x0100 (256) pixels
0xE6C0, 0xE6C0, 0xE6C0, 0xE6C0, 0xBDA1, 0x20E4, 0x0000, 0x0000, 0x0000, 0x0000, 0x20E4, 0xA4C2, 0xFF80, 0xFF80, 0xFF80, 0xFF80,   // 0x0110 (272) pixels
0xFF80, 0xE6C1, 0x7362, 0x6302, 0xE6C0, 0xFF80, 0xFF80, 0xFF80, 0xFF80, 0xFF80, 0xA4C2, 0x20E4, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0120 (288) pixels
0x0000, 0x83C2, 0xFF80, 0xFF80, 0xFF80, 0xFF80, 0xFF80, 0x9442, 0x41E4, 0x4A23, 0xAD01, 0xFF80, 0xFF80, 0xFF80, 0xFF80, 0xFF80,   // 0x0130 (304) pixels
0x7B82, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x39C3, 0xE6C0, 0xFF80, 0xFF80, 0xFF80, 0xE6C0, 0x3163, 0x20E4, 0x20E4,   // 0x0140 (320) pixels
0x3163, 0xEF00, 0xFF80, 0xFF80, 0xFF80, 0xE6C0, 0x39C3, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x20E4, 0x7342, 0xFF80,   // 0x0150 (336) pixels
0xFF80, 0xFF80, 0x83C2, 0x20E4, 0x20E4, 0x20E4, 0x20E4, 0x7362, 0xFF80, 0xFF80, 0xFF80, 0x7362, 0x20E4, 0x0000, 0x0000, 0x0000,   // 0x0160 (352) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x20E4, 0x9443, 0xFF80, 0xCDE1, 0x2104, 0x20E4, 0x20E4, 0x20E4, 0x20E4, 0x20E4, 0xDE61, 0xFF80,   // 0x0170 (368) pixels
0x9C61, 0x20E4, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x20E4, 0x7BA3, 0xC5A1, 0x7362, 0x20E4,   // 0x0180 (384) pixels
0x20E4, 0x20E4, 0x3163, 0x5AA3, 0xD621, 0x7BA2, 0x20E4, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0190 (400) pixels
0x0000, 0x0000, 0x0000, 0x20E4, 0x7BA3, 0xA4C2, 0xAD01, 0xA4E2, 0x9CA2, 0x83E3, 0x41E3, 0x20E4, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x01A0 (416) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x20E4, 0x20E4, 0x20E4, 0x20E4, 0x0000,   // 0x01B0 (432) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x01C0 (448) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x01D0 (464) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x01E0 (480) pixels
0x0000, 0x0000, 0x0000, 0x0000
};
void drawBitmapGeigerSignal() {
    int width = 22;  // Breite des Bitmaps
    int height = 22; // Höhe des Bitmaps
    M5.Lcd.drawBitmap(197, 27, width, height, pngegg);
}

// Interrupt-Funktion: Zähle Geigerzähler-Impulse und setze das Flag
void IRAM_ATTR countPulse() {
    pulseCount++;          // Impuls zählen
    drawBitmapFlag = true; // Bitmap-Zeichen-Flag setzen
}
float doseRate = 0.0;                   // Dosis in µSv/h
float averageDose = 0.0;                // Durchschnittliche Dosis
const float calibrationFactor = 6.0;//108.0;  // Kalibrierung: CPM pro µSv/h
// Historie für Durchschnittswerte
#define RATE_GRAPH_WIDTH 83
#define AVG_GRAPH_WIDTH 17
float avgGraphBuffer[AVG_GRAPH_WIDTH] = { 0 };
int avgGraphIndex = 0;
int rateGraphBuffer[RATE_GRAPH_WIDTH] = { 0 };
int rateGraphIndex = 0;
const int numHistory = 80;              // Historie für 60 Sekunden
float doseHistory[numHistory] = { 0 };  // Speicherung der letzten Werte
int historyIndex = 0;                   // Index für den Historienpuffer

unsigned long lastCount = 0;  // Letzte Impulszählung
unsigned long lastTime = 0;   // Letzte Aktualisierungszeit

float el = 0;
float az = 0;
float el_r = 0;
float az_r = 0;
float e = 0;

const float rad_fac = 0.017453292;
const float pi = 3.1415926536;

int seconds = 0;
int x = 0;
int y = 0;
int q = 0;
int r = 0;
int i = 0;
int j = 0;

// Speichert die vorherigen Zeigerpositionen
int prevHourX, prevHourY, prevHourQ, prevHourR;
int prevMinuteX, prevMinuteY, prevMinuteQ, prevMinuteR;
int prevSecondX, prevSecondY;
int previousSecond = -1;  // Startwert für Sekundenzeiger

void drawClockHands(int cetHour, int minute, int second) {
    float pi = 3.14159265359;

    // **Stundenzeiger**
    int hourY = (32 * cos(pi - (2 * pi) / 60 * ((cetHour * 5) + minute / 12))) + 77;
    int hourX = (32 * sin(pi - (2 * pi) / 60 * ((cetHour * 5) + minute / 12))) + 160;
    int hourQ = (2 * cos(pi - (2 * pi) / 60 * ((cetHour * 5) + minute / 12))) + 77;
    int hourR = (2 * sin(pi - (2 * pi) / 60 * ((cetHour * 5) + minute / 12))) + 160;

    // **Minutenzeiger**
    int minuteY = (42 * cos(pi - (2 * pi) / 60 * minute)) + 77;
    int minuteX = (42 * sin(pi - (2 * pi) / 60 * minute)) + 160;
    int minuteQ = (2 * cos(pi - (2 * pi) / 60 * minute)) + 77;
    int minuteR = (2 * sin(pi - (2 * pi) / 60 * minute)) + 160;

    // **Sekundenzeiger**
    int secondY = (45 * cos(pi - (2 * pi) / 60 * second)) + 77;
    int secondX = (45 * sin(pi - (2 * pi) / 60 * second)) + 160;

    // **Lösche vorherige Zeiger gezielt**
    if (previousSecond >= 0) {
        M5.Lcd.drawLine(160, 76, prevSecondX, prevSecondY, BLACK);  // Sekundenzeiger
    }
    if (minute != prevMinuteY) {
        M5.Lcd.drawLine(prevMinuteR + 1, prevMinuteQ + 1, prevMinuteX, prevMinuteY, BLACK);
        M5.Lcd.drawLine(prevMinuteR - 1, prevMinuteQ - 1, prevMinuteX, prevMinuteY, BLACK);
    }
    if (cetHour != prevHourY) {
        M5.Lcd.drawLine(prevHourR + 2, prevHourQ + 2, prevHourX, prevHourY, BLACK);
        M5.Lcd.drawLine(prevHourR - 2, prevHourQ - 2, prevHourX, prevHourY, BLACK);
    }

    // **Neue Zeiger zeichnen**
    M5.Lcd.drawLine(hourR + 2, hourQ + 2, hourX, hourY, CYAN);  // Stundenzeiger
    M5.Lcd.drawLine(hourR - 2, hourQ - 2, hourX, hourY, CYAN);

    M5.Lcd.drawLine(minuteR + 1, minuteQ + 1, minuteX, minuteY, CYAN);  // Minutenzeiger
    M5.Lcd.drawLine(minuteR - 1, minuteQ - 1, minuteX, minuteY, CYAN);

    M5.Lcd.drawLine(160, 76, secondX, secondY, RED);  // Sekundenzeiger

    // **Neue Positionen speichern**
    prevHourX = hourX;
    prevHourY = hourY;
    prevHourQ = hourQ;
    prevHourR = hourR;

    prevMinuteX = minuteX;
    prevMinuteY = minuteY;
    prevMinuteQ = minuteQ;
    prevMinuteR = minuteR;

    prevSecondX = secondX;
    prevSecondY = secondY;
    previousSecond = second;
}

bool GPSnotReady = false;
bool sensorConnected;

SoftwareSerial ss(RXPin, TXPin);

static const int MAX_SATELLITES = 72;

TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1);
TinyGPSCustom messageNumber(gps, "GPGSV", 2);
TinyGPSCustom satsInView(gps, "GPGSV", 3);
TinyGPSCustom satNumber[4];
TinyGPSCustom elevation[4];
TinyGPSCustom azimuth[4];
TinyGPSCustom snr[4];

struct {
  bool active;
  int elevation;
  int azimuth;
  int snr;
} sats[MAX_SATELLITES];

int sensorPin1 = 34;  // CO-Sensor
int sensorPin2 = 35;  // NH3-Sensor
int sensorPin3 = 27;  // NO2-Sensor
int sensorPin4 = 36;  // EMF-Sensor
int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;
int sensorValue4 = 0;

// Schwellenwerte definieren
const float CO_THRESHOLD = 30.0;     // CO: gefährlich ab 30 ppm
const float NH3_THRESHOLD = 25.0;   // NH3: gefährlich ab 25 ppm
const float NO2_THRESHOLD = 10.0;   // NO2: gefährlich ab 10 ppm
const float EMF_THRESHOLD = 10.0;   // EMF: gefährlich ab 10 
const float RADIATION_THRESHOLD = 10.0; // Strahlung: gefährlich ab 1.0 µSv/h

void checkForAlarms(float CO, float NH3, float NO2, float radiation) {
  bool alarmTriggered = false; // Flag, um zu prüfen, ob ein Alarm ausgelöst wurde

  // CO-Warnung
  if (CO > CO_THRESHOLD) {
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setCursor(13, 79);
    M5.Lcd.print(" !!!DANGER!!!");
    M5.Axp.SetVibration(true); // Vibrationsmotor einschalten
    M5.Axp.SetLed(true);
    delay(10);
    M5.Axp.SetVibration(false); // Vibrationsmotor ausschalten
    M5.Axp.SetLed(false);
    M5.Axp.SetVibration(true); // Vibrationsmotor einschalten
    M5.Axp.SetLed(true);
    delay(10);
    M5.Axp.SetVibration(false); // Vibrationsmotor ausschalten
    M5.Axp.SetLed(false);
    alarmTriggered = true;
  }
   // NH3-Warnung
  if (NH3 > NH3_THRESHOLD) {
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setCursor(13, 90);
    M5.Lcd.print(" !!!DANGER!!!");
    //M5.Spk.SpeakerOn();
    //delay(500);
    //M5.Spk.SpeakerOff();
    M5.Axp.SetVibration(true); // Vibrationsmotor einschalten
    M5.Axp.SetLed(true);
    delay(10);
    M5.Axp.SetVibration(false); // Vibrationsmotor ausschalten
    M5.Axp.SetLed(false);
    M5.Axp.SetVibration(true); // Vibrationsmotor einschalten
    M5.Axp.SetLed(true);
    delay(10);
    M5.Axp.SetVibration(false); // Vibrationsmotor ausschalten
    M5.Axp.SetLed(false);
    alarmTriggered = true;
  }
  // NO2-Warnung
  if (NO2 > NO2_THRESHOLD) {
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setCursor(13, 101);
    M5.Lcd.print(" !!!DANGER!!!");
    M5.Axp.SetVibration(true); // Vibrationsmotor einschalten
    M5.Axp.SetLed(true);
    delay(10);
    M5.Axp.SetVibration(false); // Vibrationsmotor ausschalten
    M5.Axp.SetLed(false);
    M5.Axp.SetVibration(true); // Vibrationsmotor einschalten
    M5.Axp.SetLed(true);
    delay(10);
    M5.Axp.SetVibration(false); // Vibrationsmotor ausschalten
    M5.Axp.SetLed(false);
    alarmTriggered = true;
  }

   // Strahlungswarnung
  if (radiation > RADIATION_THRESHOLD) {
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setCursor(224, 31);
    M5.Lcd.print("!!! DANGER !!!");
    M5.Axp.SetVibration(true); // Vibrationsmotor einschalten
    M5.Axp.SetLed(true);
    delay(10);
    M5.Axp.SetVibration(false); // Vibrationsmotor ausschalten
    M5.Axp.SetLed(false);
    M5.Axp.SetVibration(true); // Vibrationsmotor einschalten
    M5.Axp.SetLed(true);
    delay(10);
    M5.Axp.SetVibration(false); // Vibrationsmotor ausschalten
    M5.Axp.SetLed(false);
    
    alarmTriggered = true;
  }
}

// Speichert die x- und y-Koordinaten sowie die Kreisgröße der Satelliten für das Löschen alter Kreise
int oldX[MAX_SATELLITES] = { 0 };
int oldY[MAX_SATELLITES] = { 0 };
int oldCircleSize[MAX_SATELLITES] = { 0 };

bool displayOn = true;  // Status des Displays

void setup() {
  
  M5.begin();
  pinMode(geigerPin, INPUT);                                              // Setze PIN 26 als Eingang
  attachInterrupt(digitalPinToInterrupt(geigerPin), countPulse, RISING);  // Rufe countPulse() bei steigender Flanke auf
  M5.Lcd.fillScreen(BLACK);
  M5.Axp.SetLed(false);

  Wire.begin(32, 33);

  if (!bme.begin()) {
    Serial.println("BME280 nicht gefunden!");
  } else {
    Serial.println("BME280 erfolgreich verbunden.");
  }

  Serial.begin(115200);
  ss.begin(GPSBaud);

  // Initialisiere den Rate-Puffer mit neutralen Y-Werten (mittlere Höhe)
  for (int i = 0; i < RATE_GRAPH_WIDTH; i++) {
    rateGraphBuffer[i] = 90;  // Setze Startwert auf die Mitte des Bereichs
  }
  if (!SD.begin(TFCARD_CS_PIN, SPI, 40000000UL)) {
    Serial.println("SD-Karte konnte nicht initialisiert werden! Neustart...");
    ESP.restart();
  }

  bme.setTempCal(-1);
  pinMode(34, INPUT);
  pinMode(35, INPUT);
  pinMode(36, INPUT);
  pinMode(27, INPUT);
  Serial.println("Starte GPS...");
  bool pngDrawn = false;         // set this variable to 'false' to ensure that the PNG has not yet been drawn
  while (!pngDrawn) {            // Loop that keeps running until the display is tapped
    if (M5.Touch.ispressed()) {  // If the display was typed, set pngDrawn to 'true'
      pngDrawn = true;
    } else {  // Else draw the PNG
      M5.update();
      M5.Lcd.drawPngFile(SD, "/radar1.png", 0, 0);

      File myFile = SD.open("/home_coordinates.txt", FILE_READ);
      if (myFile) {
        String line = myFile.readStringUntil('\n');
        int commaIndex = line.indexOf(",");
        if (commaIndex != -1) {
          homeLat = line.substring(0, commaIndex).toDouble();
          homeLon = line.substring(commaIndex + 1).toDouble();
        } else {
          Serial.println("Fehler: Dateiinhalt ungültig!");
        }
        myFile.close();
      } else {
        Serial.println("Datei '/home_coordinates.txt' nicht gefunden!");
      }
      delay(100);  // Verhindert übermäßiges Zeichnen

      M5.Lcd.setTextColor(CYAN);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(140, 58);
      M5.Lcd.print("COORDINATES");
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(124, 79);
      //M5.Lcd.print("LATT: N");
      M5.Lcd.print("N");
      M5.Lcd.setTextSize(1);
      M5.Lcd.print("\xF7 ");
      M5.Lcd.setTextSize(2);
      M5.Lcd.print(homeLat, 6);
      M5.Lcd.setCursor(124, 96);
      //M5.Lcd.print("LONG: E");
      M5.Lcd.print("E");
      M5.Lcd.setTextSize(1);
      M5.Lcd.print("\xF7 ");
      M5.Lcd.setTextSize(2);
      M5.Lcd.print(homeLon, 6);
    }
  }
  for (int i = 0; i < 4; ++i) {
    satNumber[i].begin(gps, "GPGSV", 4 + 4 * i);
    elevation[i].begin(gps, "GPGSV", 5 + 4 * i);
    azimuth[i].begin(gps, "GPGSV", 6 + 4 * i);
    snr[i].begin(gps, "GPGSV", 7 + 4 * i);
  }

  //GRAPHIC
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.drawRoundRect(0, 0, 320, 240, 8, CYAN);
  M5.Lcd.drawRoundRect(4, 24, 310, 104, 6, CYAN);
  M5.Lcd.drawRoundRect(4, 132, 310, 102, 6, CYAN);
  M5.Lcd.drawRoundRect(8, 28, 90, 96, 4, BLUE);
  M5.Lcd.drawRoundRect(222, 28, 88, 96, 4, BLUE);
  M5.Lcd.drawRoundRect(8, 136, 90, 96, 4, BLUE);

M5.Lcd.setTextSize(1);
        M5.Lcd.setTextColor(CYAN, BLACK);
        M5.Lcd.setCursor(21, 233);
        M5.Lcd.print("> HOME POS <");
        M5.Lcd.setCursor(116, 233);
        M5.Lcd.print("> CURRENT POS <");
        M5.Lcd.setCursor(230, 233);
        M5.Lcd.print("> SVALBARD <");
        M5.Lcd.setCursor(10, 127);
        M5.Lcd.print("> DISPLAY OFF <");
}
// Funktion zur Berechnung eines Farbverlaufs von Rot (niedrig) nach Grün (hoch)
uint16_t getGradientColor(float value, float minValue, float maxValue) {
    // Normiere den Wert auf den Bereich [0, 1]
    float normalized = (value - minValue) / (maxValue - minValue);
    normalized = constrain(normalized, 0.0, 1.0);  // Begrenze auf [0, 1]

    // Umkehrung der Farben: Niedrige Werte = Rot, Hohe Werte = Grün
    uint8_t red = 255 * (1.0 - normalized);  // Rot wird intensiver bei niedrigeren Werten
    uint8_t green = 255 * normalized;        // Grün wird intensiver bei höheren Werten
    uint8_t blue = 0;                        // Blau bleibt konstant (0)

    // Konvertiere RGB in 16-Bit-Farbe (565-Format)
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
}

 void drawRateGraph(float doseRate) {
    static float sumDoseRate = 0;
    static int countSamples = 0;

    sumDoseRate += doseRate;
    countSamples++;

    if (countSamples >= 7) {  // Alle 7 Sekunden einen neuen Punkt setzen
        int newY = 80 - ((sumDoseRate / countSamples) * 10);  // Berechne neuen Y-Wert
        newY = constrain(newY, 40, 80);  // Begrenze Y-Wert auf Diagrammbereich
        rateGraphBuffer[rateGraphIndex] = newY;  // Speichere neuen Y-Wert
        rateGraphIndex = (rateGraphIndex + 1) % RATE_GRAPH_WIDTH;  // Zyklischer Puffer

        sumDoseRate = 0;
        countSamples = 0;
    }

    // Lösche den Bereich für das Diagramm
    M5.Lcd.fillRect(223, 31, 86, 60, BLACK);  // Startpunkt jetzt bei 223

    // Zeichne **horizontale Skalenlinien** für Dosiswerte
    for (int y = 40; y <= 120; y += 10) {  // Hauptlinien alle 10 µSv/h
        M5.Lcd.drawFastHLine(223, y, 85, 0x0320);
    }

    for (int y = 45; y <= 75; y += 10) {  // Hilfslinien alle 5 µSv/h (gepunktet)
        for (int x = 223; x < 309; x += 4) {
            M5.Lcd.drawPixel(x, y, 0x0320);
        }
    }

    // Zeichne **vertikale Linien** für Zeitachse jetzt alle 8 Pixel
    for (int x = 223; x <= 309; x +=14) {
        M5.Lcd.drawFastVLine(x, 40, 40, 0x0320);
    }

    // Zeichne das Diagramm mit Farbverlauf
    for (int i = 0; i < RATE_GRAPH_WIDTH - 1; i++) {
        int x1 = 224 + i;  // Verschoben nach links
        int y1 = rateGraphBuffer[(rateGraphIndex + i) % RATE_GRAPH_WIDTH];
        int x2 = 224 + (i + 1);
        int y2 = rateGraphBuffer[(rateGraphIndex + i + 1) % RATE_GRAPH_WIDTH];

        // Berechne den Farbverlauf basierend auf dem Y-Wert (unten = Rot, oben = Grün)
        uint16_t color = getGradientColor(y1, 40, 80);

        // Zeichne die Linie mit dem berechneten Farbverlauf
        M5.Lcd.drawLine(x1, y1, x2, y2, color);
    }
}

 void drawAverageGraph(float avgDose) {
    // Buffer aktualisieren: Durchschnittswerte für die letzten 60 Minuten in 16 Säulen
    avgGraphBuffer[avgGraphIndex] = avgDose * 10;  // Skaliere die Dosis (1 µSv/h = 10 Pixel)
    avgGraphBuffer[avgGraphIndex] = constrain(avgGraphBuffer[avgGraphIndex], 0, 68);  // Begrenze Höhe der Säule
    avgGraphIndex = (avgGraphIndex + 1) % AVG_GRAPH_WIDTH;  // Zyklischer Puffer für 16 Säulen 

M5.Lcd.fillRect(224, 90, 84, 33, BLACK);

   // Zeichne die Säulen mit Abstand von 5 Pixeln
    for (int i = 0; i < AVG_GRAPH_WIDTH; i++) {
        int x = 224 + i * 5;  // X-Position für jede Säule (4 Pixel Breite + 1 Pixel Abstand)
        int height = avgGraphBuffer[(avgGraphIndex + i) % AVG_GRAPH_WIDTH];
        int y = 120 - height;  // Y-Position basierend auf der Höhe der Säule

        // Wähle Farbe basierend auf Dosiswert
        uint16_t color;
        float dose = height / 10.0;  // Rückrechnen auf µSv/h
        if (dose < 0.5) {
            color = GREEN;
        } else if (dose < 1.0) {
            color = YELLOW;
        } else if (dose < 2.0) {
            color = ORANGE;
        } else {
            color = RED;
        }
        // Zeichne die Säule
        M5.Lcd.fillRect(x, y, 4, height, color);
    }
}

// Werteanzeige
void displayValues(float doseRate, float averageDose) {
  
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(225, 31);
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.print("DR:");

  M5.Lcd.setTextColor(CYAN, BLACK); 
  M5.Lcd.printf("%.2f uSv/h", doseRate);
  M5.Lcd.setCursor(225, 82);
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.print("AD:");

  uint16_t avgColor = (averageDose < 0.5) ? GREEN : (averageDose < 1.0) ? YELLOW
                                                  : (averageDose < 2.0) ? ORANGE
                                                                        : RED;
  M5.Lcd.setTextColor(avgColor, BLACK);
  M5.Lcd.printf("%.2f uSv/h", averageDose);
}
int calculateCET(TinyGPSDate &date, TinyGPSTime &time) {
  int month = date.month();
  int day = date.day();

  // Wochentag berechnen (0 = Sonntag, 1 = Montag, ...)
  int k = day;
  int m = (month < 3) ? month + 12 : month;  // Januar und Februar auf 13 und 14 verschieben
  int d = date.year() % 100;                // Letzte zwei Stellen des Jahres
  int c = date.year() / 100;                // Hunderter des Jahres
  int weekday = (k + (13 * (m + 1)) / 5 + d + (d / 4) + (c / 4) - (2 * c)) % 7;
  weekday = (weekday + 7) % 7;  // Korrektur für negative Werte

  // Sommerzeit von letztem Sonntag im März bis letztem Sonntag im Oktober
  if (month > 3 && month < 10) {
    return 2;  // Sommerzeit (UTC+2)
  } else if (month == 3) {
    int lastSunday = 31 - weekday;  // Letzter Sonntag im März
    return (day >= lastSunday) ? 2 : 1;  // Sommerzeit ab letztem Sonntag
  } else if (month == 10) {
    int lastSunday = 31 - weekday;  // Letzter Sonntag im Oktober
    return (day < lastSunday) ? 2 : 1;  // Winterzeit nach letztem Sonntag
  } else {
    return 1;  // Winterzeit (UTC+1)
  }
}

void loop() {

  M5.update();  // Touch-Events aktualisieren
// Prüfe, ob das Bitmap-Flag gesetzt wurde
    M5.Lcd.fillRect(198, 25, 23, 23, BLACK); // Lösche alten Wert
    if (drawBitmapFlag) {      
        drawBitmapGeigerSignal(); // Zeichne das Bitmap
        drawBitmapFlag = false;  // Zurücksetzen des Flags
    }
  
  if (M5.Touch.ispressed()) {                            // Prüfe, ob der Bildschirm berührt wird
    TouchPoint_t touchPoint = M5.Touch.getPressPoint();  // Hol die Berührungskoordinaten
    // Bereich unten links (10x10 mm) zum Ein- und Ausschalten
    if (touchPoint.x >= 0 && touchPoint.x <= 100 && touchPoint.y >= 140 && touchPoint.y <= 240) {
       if (displayOn) {
          M5.Lcd.sleep();             // Display ausschalten
          displayOn = false;
      } 
      else {
          M5.Lcd.wakeup();             // Display einschalten
          displayOn = true;
      }
    }
  }

  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  if (now - lastUpdate >= 1000) {
    lastUpdate = now;

    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();

    doseRate = count / calibrationFactor;

    static float sumDose = 0;  // Summe der Dosiswerte
    static int countSamples = 0;  // Anzahl der Samples pro Säule
    sumDose += doseRate;
    countSamples++;

 // Alle 3,75 Minuten (225 Sekunden) aktualisieren
if (countSamples >= 225) {  // Alle 3,75 Minuten
            float avgDose = sumDose / countSamples;
            drawAverageGraph(avgDose);
            sumDose = 0;
            countSamples = 0;
        }     
    drawRateGraph(doseRate);
   if (countSamples > 0) {
            displayValues(doseRate, sumDose / countSamples);
        } else {
            displayValues(doseRate, 0);
        }
    }
  float radiation = doseRate;   // Strahlung in µSv/h

  // Alarmprüfung
  checkForAlarms(CO, NH3, NO2, radiation);
  
  //GPS INI
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  //SATELLITE TRACKER
  if (ss.available() > 0) {
    gps.encode(ss.read());
    if (totalGPGSVMessages.isUpdated()) {
      for (int i = 0; i < 4; ++i) {
        int no = atoi(satNumber[i].value());
        Serial.print(F("SatNumber is "));
        Serial.println(no);
        if (no >= 1 && no <= MAX_SATELLITES) {
          sats[no - 1].elevation = atoi(elevation[i].value());
          sats[no - 1].azimuth = atoi(azimuth[i].value());
          sats[no - 1].snr = atoi(snr[i].value());
          sats[no - 1].active = true;
        }
      }

      int totalMessages = atoi(totalGPGSVMessages.value());
      int currentMessage = atoi(messageNumber.value());
      if (totalMessages == currentMessage)
        Serial.print(F("Sats="));
      Serial.print(gps.satellites.value());
      Serial.print(F(" Nums="));
      for (int i = 0; i < MAX_SATELLITES; ++i) {
        if (sats[i].active) {
          Serial.print(i + 1);
          Serial.print(F(" "));
        }
      }
      GPSnotReady = ((gps.location.lat() == 0) && (gps.location.lng() == 0));
    }
  }

  //LAST SAVED COORDINATES
  File myFile = SD.open("/home_coordinates.txt", FILE_READ);
  if (myFile) {
    String line = myFile.readStringUntil('\n');
    int commaIndex = line.indexOf(",");
    if (commaIndex != -1) {
      homeLat = line.substring(0, commaIndex).toDouble();
      homeLon = line.substring(commaIndex + 1).toDouble();
    }
    myFile.close();
  }
  //TOUCH BUTTONS
  if (M5.BtnA.wasPressed()) {
    M5.Lcd.fillRoundRect(100, 136, 210, 96, 4, BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.setCursor(108, 137);
    M5.Lcd.print("HOME COORDINATES");
    M5.Lcd.setCursor(108, 157);
    M5.Lcd.print("LATT: N");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\xF7 ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("51.861120");
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(108, 177);
    M5.Lcd.print("LONG: E");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\xF7 ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print(" 8.289310");
    delay(3000);
    //M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
    File myFile = SD.open("/home_coordinates.txt", FILE_WRITE);
    if (myFile) {
      myFile.print(51.861120, 6);
      myFile.print(",");
      myFile.println(8.289310, 6);
      myFile.close();
      homeLat = 51.861120;
      homeLon = 8.289310;
    } else {
      Serial.println("ERROR WRITE FILE");
    }
  M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
  }

  if (M5.BtnB.wasPressed()) {
    M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
    homeLat = gps.location.lat();
    homeLon = gps.location.lng();
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.setCursor(108, 137);
    M5.Lcd.print("POSITION SAVED");
    M5.Lcd.setCursor(108, 157);
    M5.Lcd.print("LATT: N");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\xF7 ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print(gps.location.lat(), 6);
    M5.Lcd.setCursor(108, 177);
    M5.Lcd.print("LONG: E");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\xF7   ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print(gps.location.lng(), 6);
    delay(3000);
    //M5.Lcd.fillRoundRect(100, 136, 214, 94, 4, BLACK);

    if (gps.location.isValid()) {
      File myFile = SD.open("/home_coordinates.txt", FILE_WRITE);
      if (myFile) {
        myFile.print(gps.location.lat(), 6);
        myFile.print(",");
        myFile.println(gps.location.lng(), 6);
        myFile.close();
        homeLat = gps.location.lat();
        homeLon = gps.location.lng();
      } else {
        Serial.println("ERROR WRITE FILE");
      }
    }
  M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
  }
  if (M5.BtnC.wasPressed()) {
    M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.setCursor(108, 137);
    M5.Lcd.print("SVALBARD GSV");
    M5.Lcd.setCursor(108, 157);
    M5.Lcd.print("LATT: N");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\xF7 ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("78.235680");
    M5.Lcd.setCursor(108, 177);
    M5.Lcd.print("LONG: E");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\xF7 ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("15.491320");
    delay(3000);
    File myFile = SD.open("/home_coordinates.txt", FILE_WRITE);
    if (myFile) {
      myFile.print(78.235680, 6);
      myFile.print(",");
      myFile.println(15.491320, 6);
      myFile.close();
      homeLat = 78.235680;
      homeLon = 15.491320;
    } else {
      Serial.println("ERROR WRITE FILE");
    }
  M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
  }
  unsigned long distanceToHome =
    (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon);  // 1000;
  printInt(distanceToHome, gps.location.isValid(), 9);
  double courseToHome =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      homeLat,
      homeLon);
  printFloat(courseToHome, gps.location.isValid(), 7, 2);
  const char *cardinalToHome = TinyGPSPlus::cardinal(courseToHome);
  printStr(gps.location.isValid() ? cardinalToHome : "*** ", 6);
  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  smartDelay(930);
 M5.Lcd.drawPngFile(SD, "/radar6.png", 114, 30);
  /*M5.Lcd.drawCircle(160, 77, 47, GREEN);
  M5.Lcd.drawCircle(160, 77, 16, GREEN);
  M5.Lcd.drawCircle(160, 77, 32, GREEN);
  M5.Lcd.drawFastHLine(113, 77, 95, GREEN);
  M5.Lcd.drawFastVLine(160, 30, 95, GREEN);
  M5.Lcd.drawLine(127, 44, 192, 109, GREEN);
  M5.Lcd.drawLine(127, 109, 192, 44, GREEN);
 */
 //Zifferblatt mit Stundenmarkierungen**
    for (int i = 0; i < 12; i++) {
        int y = (47 * cos(PI - (2 * PI) / 12 * i)) + 76;
        int x = (47 * sin(PI - (2 * PI) / 12 * i)) + 160;
        int q = (43 * cos(PI - (2 * PI) / 12 * i)) + 76;
        int r = (43 * sin(PI - (2 * PI) / 12 * i)) + 160;
        M5.Lcd.drawLine(r, q, x, y, GREEN);  // Zeichnet die 12 Stundenstriche
    }
 
 //SATELLITES DISPLAY
  if (!GPSnotReady) {
    int centerX = 160, centerY = 75;
    float rad_fac = 3.14159265359 / 180;

    for (int i = 0; i < MAX_SATELLITES; ++i) {
      if (sats[i].active && sats[i].snr > 1) {
        // Berechnung der Position
        float az = sats[i].azimuth;
        float el = sats[i].elevation / 2;
        float az_r = az * rad_fac;
        float e = 42 * (90 - el) / 100;
        int x = centerX + (sin(az_r) * e);
        int y = centerY - (cos(az_r) * e);

        // Bestimme Farbe basierend auf SNR
        uint16_t circleColor;
        if (sats[i].snr > 61) {
          sats[i].snr = 60;
        } else if (sats[i].snr > 55) {
          circleColor = 0x03E0;
        } else if (sats[i].snr > 50) {
          circleColor = 0x03E0;
        } else if (sats[i].snr > 45) {
          circleColor = 0xAFE5;
        } else if (sats[i].snr > 40) {
          circleColor = 0xFFE0;
        } else if (sats[i].snr > 35) {
          circleColor = 0x07FF;
        } else if (sats[i].snr > 30) {
          circleColor = 0x03EF;
        } else if (sats[i].snr > 25) {
          circleColor = 0xFD20;
        } else if (sats[i].snr > 20) {
          circleColor = 0x001F;
        } else if (sats[i].snr > 19) {
          circleColor = 0x000F;
        } else if (sats[i].snr > 18) {
          circleColor = 0x7BE0;
        } else if (sats[i].snr > 17) {
          circleColor = 0x780F;
        } else if (sats[i].snr > 16) {
          circleColor = 0x7800;
        } else if (sats[i].snr > 15) {
          circleColor = 0xF81F;
        } else if (sats[i].snr > 14) {
          circleColor = 0xC618;
        } else if (sats[i].snr > 13) {
          circleColor = 0xF800;
        } else if (sats[i].snr > 12) {
          circleColor = 0xFFFF;
        }

        // Dynamische Kreisgröße basierend auf SNR
        int circleSize = map(sats[i].snr, 10, 40, 2, 8);

        // Lösche den alten Kreis
        if (oldX[i] != 0 && oldY[i] != 0) {
          M5.Lcd.drawCircle(oldX[i], oldY[i], oldCircleSize[i], BLACK);  // Hintergrundfarbe
        }


        // Zeichne den neuen Kreis
        M5.Lcd.drawCircle(x, y, circleSize, circleColor);

        // Speichere die aktuelle Position und Größe
        oldX[i] = x;
        oldY[i] = y;
        oldCircleSize[i] = circleSize;

   
        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(104, 30);
        M5.Lcd.setTextColor(CYAN, BLACK);
        M5.Lcd.print("SATS");
        M5.Lcd.setCursor(104, 40);
        M5.Lcd.print(gps.satellites.value());
        M5.Lcd.print(" ");
      } else {
        // Falls der Satellit nicht aktiv ist, lösche den alten Kreis
        if (oldX[i] != 0 && oldY[i] != 0) {
          M5.Lcd.drawCircle(oldX[i], oldY[i], oldCircleSize[i], BLACK);  // Hintergrundfarbe
          oldX[i] = 0;
          oldY[i] = 0;
          oldCircleSize[i] = 0;
        }
      }
    }
  }

  M5.Lcd.fillCircle(53, 184, 34, BLACK);
  double relCourse = courseToHome - gps.course.deg();
  if (relCourse < 0) {
    relCourse = relCourse + 360;
  }
  if (relCourse >= 360) {
  }
  relCourse = relCourse - 360;
  if (distanceToHome < 10) {

    M5.Lcd.drawCircle(53, 184, 30, DARKGREY);
    M5.Lcd.drawCircle(53, 184, 23, DARKGREEN);
    M5.Lcd.drawCircle(53, 184, 24, OLIVE);
    M5.Lcd.drawCircle(53, 184, 16, GREENYELLOW);
    M5.Lcd.drawCircle(53, 184, 17, GREEN);
    M5.Lcd.fillTriangle(53, 151, 53, 184, 48, 178, GREEN);
    M5.Lcd.fillTriangle(53, 151, 53, 184, 58, 178, DARKGREEN);
    M5.Lcd.fillTriangle(88, 184, 53, 184, 58, 178, GREEN);
    M5.Lcd.fillTriangle(88, 184, 53, 184, 58, 190, DARKGREEN);
    M5.Lcd.fillTriangle(53, 184, 53, 218, 58, 190, GREEN);
    M5.Lcd.fillTriangle(53, 184, 53, 218, 48, 190, DARKGREEN);
    M5.Lcd.fillTriangle(19, 184, 53, 184, 48, 190, GREEN);
    M5.Lcd.fillTriangle(19, 184, 53, 184, 48, 178, DARKGREEN);
    
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(YELLOW, BLACK);

    M5.Lcd.fillRoundRect(51, 140, 8, 10, 2, BLACK);
    M5.Lcd.setCursor(51, 140);
    M5.Lcd.print("N");
    M5.Lcd.setCursor(11, 181);
    M5.Lcd.print("W");
    M5.Lcd.setCursor(89, 181);
    M5.Lcd.print("E");
  } else {
    x = (sin(relCourse * rad_fac) * 26);
    y = (cos(relCourse * rad_fac) * 26);
    q = sin(relCourse * rad_fac);
    r = cos(relCourse * rad_fac);
    int x1 = 53 + x;
    int y1 = 184 - y;
    int x2 = 53 - x;
    int y2 = 184 + y;
    int q1 = 53 + q;
    int r1 = 184 - r;
    int q2 = 53 - q;
    int r2 = 184 + r;

    double arrowAngle = relCourse + 200;
    x = (sin(arrowAngle * rad_fac) * 48);
    y = (cos(arrowAngle * rad_fac) * 48);
    q = (sin(arrowAngle * rad_fac) * 48);
    r = (cos(arrowAngle * rad_fac) * 48);
    x2 = x1 + x;
    y2 = y1 - y;
    q2 = q1 + q;
    r2 = r1 - r;
    if ((gps.location.lat() == 0) && (gps.location.lng() == 0)) {
      M5.Lcd.drawCircle(53, 184, 30, 0x00AF);
      M5.Lcd.drawCircle(53, 184, 23, 0x00AF);
      M5.Lcd.drawCircle(53, 184, 24, 0x00AF);
      M5.Lcd.drawCircle(53, 184, 16, BLUE);
      M5.Lcd.drawCircle(53, 184, 17, BLUE);
      M5.Lcd.fillTriangle(53, 151, 53, 184, 48, 178, BLUE);
      M5.Lcd.fillTriangle(53, 151, 53, 184, 58, 178, 0x00AF);
      M5.Lcd.fillTriangle(88, 184, 53, 184, 58, 178, BLUE);
      M5.Lcd.fillTriangle(88, 184, 53, 184, 58, 190, 0x00AF);
      M5.Lcd.fillTriangle(53, 184, 53, 218, 58, 190, BLUE);
      M5.Lcd.fillTriangle(53, 184, 53, 218, 48, 190, 0x00AF);
      M5.Lcd.fillTriangle(19, 184, 53, 184, 48, 190, BLUE);
      M5.Lcd.fillTriangle(19, 184, 53, 184, 48, 178, 0x00AF);
    } else {
      M5.Lcd.drawLine(x1, y1, x2, y2, YELLOW);
      M5.Lcd.drawLine(q1, r1, x2, y2, YELLOW);
    }
    arrowAngle = relCourse + 160;
    x = (sin(arrowAngle * rad_fac) * 48);
    y = (cos(arrowAngle * rad_fac) * 48);
    q = (sin(arrowAngle * rad_fac) * 48);
    r = (cos(arrowAngle * rad_fac) * 48);
    x2 = x1 + x;
    y2 = y1 - y;
    q2 = q1 + q;
    r2 = r1 - r;
    if ((gps.location.lat() == 0) && (gps.location.lng() == 0)) {
      M5.Lcd.drawCircle(53, 184, 30, 0x00AF);
      M5.Lcd.drawCircle(53, 184, 23, 0x00AF);
      M5.Lcd.drawCircle(53, 184, 24, 0x00AF);
      M5.Lcd.drawCircle(53, 184, 16, BLUE);
      M5.Lcd.drawCircle(53, 184, 17, BLUE);
      M5.Lcd.fillTriangle(53, 151, 53, 184, 48, 178, BLUE);
      M5.Lcd.fillTriangle(53, 151, 53, 184, 58, 178, 0x00AF);
      M5.Lcd.fillTriangle(88, 184, 53, 184, 58, 178, BLUE);
      M5.Lcd.fillTriangle(88, 184, 53, 184, 58, 190, 0x00AF);
      M5.Lcd.fillTriangle(53, 184, 53, 218, 58, 190, BLUE);
      M5.Lcd.fillTriangle(53, 184, 53, 218, 48, 190, 0x00AF);
      M5.Lcd.fillTriangle(19, 184, 53, 184, 48, 190, BLUE);
      M5.Lcd.fillTriangle(19, 184, 53, 184, 48, 178, 0x00AF);
    } else {
      M5.Lcd.drawLine(x1, y1, x2, y2, YELLOW);
      M5.Lcd.drawLine(q1, r1, x2, y2, YELLOW);
    }
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setCursor(51, 140);
    M5.Lcd.print("N");
    M5.Lcd.setCursor(11, 181);
    M5.Lcd.print("W");
    M5.Lcd.setCursor(89, 181);
    M5.Lcd.print("E");
  }

  //COORDINATES
  if (distanceToHome > 10) {
    M5.Lcd.setCursor(14, 140);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print(relCourse + 360, 0);
    M5.Lcd.println("\xF7  ");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.setCursor(66, 140);
    M5.Lcd.print(TinyGPSPlus::cardinal(relCourse + 360));
    M5.Lcd.print("  ");
  } else {
    M5.Lcd.fillRoundRect(13, 140, 32, 12, 1, BLACK);
    M5.Lcd.fillRoundRect(60, 140, 32, 12, 1, BLACK);
  }

  //CLOCK FACE
// Berechne CET-Offset
int cetOffset = calculateCET(gps.date, gps.time);
int cetHour = gps.time.hour() + cetOffset;

if (cetHour >= 24) cetHour -= 24;  // Überlaufkorrektur
if (cetHour < 0) cetHour += 24;   // Unterlaufkorrektur

  drawClockHands(cetHour, gps.time.minute(), gps.time.second());
      M5.Lcd.fillCircle(160, 77, 4, GREEN);
  //BME280 I2C MODULE
  bme.readSensor();
  M5.Lcd.setTextSize(1);
  M5.Lcd.drawRoundRect(12, 31, 82, 11, 2, 0x00AF);
  M5.Lcd.setCursor(16, 33);
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.print("T:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.print(bme.getTemperature_C(), 1);
  M5.Lcd.print(" \xF7");
  M5.Lcd.println("C  ");
  M5.Lcd.drawRoundRect(12, 42, 82, 11, 2, 0x00AF);
  M5.Lcd.setCursor(16, 44);
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.print("H:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.print(bme.getHumidity(), 0);
  M5.Lcd.println(" %  ");
  M5.Lcd.drawRoundRect(12, 53, 82, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(12, 64, 82, 12, 2, 0x00AF);
  if ((bme.getPressure_HP() / 100) <= 970) {
    M5.Lcd.setCursor(16, 67);
    M5.Lcd.print(">> STORM <<");
  } else if ((bme.getPressure_HP() / 100) <= 1001) {
    if (bme.getTemperature_C() > 1) {
      M5.Lcd.setCursor(16, 67);
      M5.Lcd.print(" >> RAIN <<");
    } else {
      M5.Lcd.setCursor(16, 67);
      M5.Lcd.print(" >> SNOW <<");
    }
  } else if ((bme.getPressure_HP() / 100) <= 1010) {
    M5.Lcd.setCursor(16, 67);
    M5.Lcd.print("> OVERCAST <");
  } else if ((bme.getPressure_HP() / 100) <= 1020) {
    M5.Lcd.setCursor(16, 67);
    M5.Lcd.print(">> CLOUDY <<");
  } else if ((bme.getPressure_HP() / 100) <= 1040) {
    M5.Lcd.setCursor(16, 67);
    M5.Lcd.print(">> CLEAR <<");
  }
  M5.Lcd.setCursor(16, 55);
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.print("P:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.print(bme.getPressure_HP() / 100, 0);
  M5.Lcd.println(" HPa  ");

  //GAS SENSOR CJMCU-6814 MODULE ANALOG OUT
int sensorValue1 = (analogRead(sensorPin1));
  int sensorValue2 = (analogRead(sensorPin2));
  int sensorValue3 = (analogRead(sensorPin3));
  int sensorValue4 = (analogRead(sensorPin4));

  float CO = (sensorValue1) / 10;   //carbon monoxide
  float NH3 = (sensorValue2) / 10;  //ammonia
  float NO2 = (sensorValue3) / 10;  //nitrogen dioxide*/
  float EMF = (sensorValue4) / 10;  //electromagnetic field

  if (CO > 3800) {
    M5.Lcd.drawRoundRect(12, 77, 82, 11, 2, RED);
    M5.Lcd.setCursor(16, 79);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("CO :");
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print(CO / 40.95);
  } else if (CO > 3200) {
    M5.Lcd.drawRoundRect(12, 77, 82, 11, 2, ORANGE);
    M5.Lcd.setCursor(16, 79);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("CO :");
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.print(CO / 40.95);
  } else {
    M5.Lcd.drawRoundRect(12, 77, 82, 11, 2, 0x00AF);
    M5.Lcd.setCursor(16, 79);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("CO :");
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print(CO / 40.95);
  }
  //M5.Lcd.fillRoundRect(13, 89, 80, 9, 2, BLACK);
  if (NH3 > 3000) {
    M5.Lcd.drawRoundRect(12, 88, 82, 11, 2, RED);
    M5.Lcd.setCursor(16, 90);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("NH3:");
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print(NH3 / 40.95);
  } else if (NH3 > 2000) {
    M5.Lcd.drawRoundRect(12, 88, 82, 11, 2, ORANGE);
    M5.Lcd.setCursor(16, 90);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("NH3:");
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.print(NH3 / 40.95);
  } else {
    M5.Lcd.drawRoundRect(12, 88, 82, 11, 2, 0x00AF);
    M5.Lcd.setCursor(16, 90);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("NH3:");
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print(NH3 / 40.95);
  }
  //M5.Lcd.fillRoundRect(13, 100, 80, 9, 2, BLACK);
  if (NO2 > 3000) {
    M5.Lcd.drawRoundRect(12, 99, 82, 11, 2, RED);
    M5.Lcd.setCursor(16, 101);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("NO2:");
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print(NO2 / 40.95);
  } else if (NO2 > 2000) {
    M5.Lcd.drawRoundRect(12, 99, 82, 11, 2, ORANGE);
    M5.Lcd.setCursor(16, 101);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("NO2:");
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.print(NO2 / 40.95);
  } else {
    M5.Lcd.drawRoundRect(12, 99, 82, 11, 2, 0x00AF);
    M5.Lcd.setCursor(16, 101);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("NO2:");
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print(NO2 / 40.95);
  }
  if (EMF > 3000) {
    M5.Lcd.drawRoundRect(12, 110, 82, 11, 2, RED);
    M5.Lcd.setCursor(16, 112);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("EMF:");
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print(EMF / 40.95);
  } else if (EMF > 2000) {
    M5.Lcd.drawRoundRect(12, 110, 82, 11, 2, ORANGE);
    M5.Lcd.setCursor(16, 112);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("EMF:");
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.print(EMF / 40.95);
  } else {
    M5.Lcd.drawRoundRect(12, 110, 82, 11, 2, 0x00AF);
    M5.Lcd.setCursor(16, 112);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("EMF:");
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print(EMF / 40.95);
  }
  //BATTERY
  float batVoltage = M5.Axp.GetBatVoltage();
  float batPercentage = (batVoltage < 3.20) ? 0 : (batVoltage - 3.20) * 100;
  float ACin = M5.Axp.isACIN();

  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(240, 5);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.print(batVoltage, 2);
  M5.Lcd.println("v");
  if ((batPercentage) < 20) {
    M5.Lcd.fillRoundRect(276, 8, 33, 12, 2, BLACK);
    M5.Lcd.fillRoundRect(277, 9, (batPercentage) / 3.2, 10, 2, RED);
    M5.Lcd.setCursor(240, 14);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print(batPercentage, 0);
    M5.Lcd.println("% ");
  } else if ((batPercentage) < 50) {
    M5.Lcd.fillRoundRect(276, 8, 33, 12, 2, BLACK);
    M5.Lcd.fillRoundRect(277, 9, (batPercentage) / 3.2, 10, 2, YELLOW);
    M5.Lcd.setCursor(240, 14);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print(batPercentage, 0);
    M5.Lcd.println("% ");
  } else {
    M5.Lcd.fillRoundRect(276, 8, 33, 12, 2, BLACK);
    M5.Lcd.fillRoundRect(277, 9, (batPercentage) / 3.2, 10, 2, GREEN);
    M5.Lcd.setCursor(240, 14);
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print(batPercentage, 0);
    M5.Lcd.println("% ");
  }

  //LOAD BATTERY
  M5.Lcd.drawRoundRect(275, 7, 35, 14, 2, GREENYELLOW);
  M5.Lcd.drawRoundRect(309, 10, 4, 7, 2, GREENYELLOW);
  if (M5.Axp.isACIN()) {
    M5.Lcd.fillRoundRect(276, 8, 33, 12, 2, BLACK);
    M5.Lcd.fillRoundRect(277, 9, (batPercentage) / 3.2, 10, 1, GREEN);
    M5.Lcd.fillTriangle(291, 14, 288, 14, 295, 7, RED);
    M5.Lcd.fillTriangle(296, 13, 291, 13, 289, 20, RED);
  }

  //GPS
  //M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.setCursor(106, 136);
  M5.Lcd.print("LATT:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  if ((gps.location.lat(), 6) < 0) M5.Lcd.print("S");
  else M5.Lcd.print("N");
  M5.Lcd.setTextSize(1);
  M5.Lcd.print("\xF7 ");
  M5.Lcd.setTextSize(2);
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    M5.Lcd.print(" ---");
  } else {
    M5.Lcd.println(gps.location.lat(), 6);
  }
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.setCursor(106, 156);
  M5.Lcd.print("LONG:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  if ((gps.location.lng(), 6) < 0) M5.Lcd.print("W");
  else M5.Lcd.print("E");
  M5.Lcd.setTextSize(1);
  M5.Lcd.print("\xF7 ");
  M5.Lcd.setTextSize(2);
  if ((gps.location.lng(), 0) < 10) {
    M5.Lcd.print(" ");
  }
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    M5.Lcd.print("---");

  } else {
    M5.Lcd.println(gps.location.lng(), 6);
  }
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.setCursor(106, 176);
  M5.Lcd.print("ALTI:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    M5.Lcd.print("---");
  } else {
    M5.Lcd.print(gps.altitude.meters(), 2);
    M5.Lcd.print("m   ");
  }
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.setCursor(106, 196);
  M5.Lcd.print("SPED:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    M5.Lcd.print("---");
  } else {

    M5.Lcd.print(gps.speed.kmph(), 0);
    M5.Lcd.print("km/h   ");
  }
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.setCursor(106, 215);
  M5.Lcd.print("HOME:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  if ((gps.location.lat() == 0) && (gps.location.lng() == 0)) {
    M5.Lcd.print("---");

  } else if (TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) < 1000) {
    M5.Lcd.print(TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon), 0);
    M5.Lcd.print("m    ");
  } else {
    M5.Lcd.print(TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000, 2);
    M5.Lcd.print("km  ");
  }
  M5.Lcd.fillRect(12, 222, 80, 9, BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(RED);
  M5.Lcd.setCursor(51, 222);
  M5.Lcd.print("S");
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(RED, BLACK);
  M5.Lcd.setCursor(16, 222);

  // how long to the destination
  if (((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) * 60 > 4200) {
    M5.Lcd.fillRect(12, 222, 80, 9, BLACK);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print(((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) / 10, 1);
    M5.Lcd.print(" DAYS");
  } else if (((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) * 60 == 600) {
    M5.Lcd.fillRect(12, 222, 80, 9, BLACK);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print(((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) / 10, 1);
    M5.Lcd.print(" DAY");
  } else if (((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) * 60 >= 600) {
    M5.Lcd.fillRect(12, 222, 80, 9, BLACK);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print(((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) / 10, 1);
    M5.Lcd.print(" DAYS");
  } else if (((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) * 60 == 60) {
    M5.Lcd.fillRect(12, 222, 80, 9, BLACK);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3, 2);
    M5.Lcd.print(" HOUR");
  } else if (((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) * 60 > 60) {
    M5.Lcd.fillRect(12, 222, 80, 9, BLACK);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3, 2);
    M5.Lcd.print(" HOURS");
  } else if (((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) * 60 > 1.5) {
    M5.Lcd.fillRect(12, 222, 80, 9, BLACK);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print(((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) * 60, 0);
    M5.Lcd.print(" MINUTES");
  } else if (((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) * 60 >= 0.5) {
    M5.Lcd.fillRect(12, 222, 80, 9, BLACK);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print(((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) * 60, 0);
    M5.Lcd.print(" MINUTE");
  } else if (((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) * 60 <= 0.5) {
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print(" DESTINATION");
  }
}
//LOOP END

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
static void printFloat(float val, bool valid, int len, int prec) {
  if (!valid) {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  } else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                           : vi >= 10  ? 2
                                       : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
}
static void printInt(unsigned long val, bool valid, int len) {
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
}

//DATE/TIME
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  if (!d.isValid()) {
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.setCursor(6, 6);
    M5.Lcd.print(F(">> NO GPS SIGNAL <<"));
  } else {
    char sz[32];
    sprintf(sz, "%02d.%02d.%02d ", d.year(), d.month(), d.day());
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.setCursor(6, 6);
    M5.Lcd.setTextSize(2);
    M5.Lcd.print(sz);
    if (!t.isValid()) {
      Serial.print(F("***"));
    } else {
      char sz[32];
      sprintf(sz, "%02d:%02d:%02d", t.hour(), t.minute(), t.second());
      M5.Lcd.print(sz);
    }
  }
  printInt(d.age(), d.isValid(), 5);
}
static void printStr(const char *str, int len) {
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
}









