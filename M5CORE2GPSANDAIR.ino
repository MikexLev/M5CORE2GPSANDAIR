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
float doseRate = 0.0;                   // Dosis in µSv/h
float averageDose = 0.0;                // Durchschnittliche Dosis
const float calibrationFactor = 100.0;  // Kalibrierung: CPM pro µSv/h

// Historie für Durchschnittswerte
#define RATE_GRAPH_WIDTH 80
#define AVG_GRAPH_WIDTH 80
float avgGraphBuffer[AVG_GRAPH_WIDTH] = { 0 };
int avgGraphIndex = 0;
int rateGraphBuffer[RATE_GRAPH_WIDTH] = { 0 };
int rateGraphIndex = 0;
const int numHistory = 80;              // Historie für 60 Sekunden
float doseHistory[numHistory] = { 0 };  // Speicherung der letzten Werte
int historyIndex = 0;                   // Index für den Historienpuffer

// Funktion für Interrupt, zählt Geigerimpulse
void IRAM_ATTR countPulse() {
  pulseCount++;  // Interrupt-Routine für Impulszählung
}

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

bool GPSnotReady = false;
bool sensorConnected;

SoftwareSerial ss(RXPin, TXPin);

static const int MAX_SATELLITES = 32;

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
      M5.Lcd.drawPngFile(SD, "/radar7.png", 0, 0);

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
      M5.Lcd.setCursor(60, 150);
      M5.Lcd.print("SAVED COORDINATES");
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(0, 180);
      M5.Lcd.print("LATT: N");
      M5.Lcd.setTextSize(1);
      M5.Lcd.print("\xF7 ");
      M5.Lcd.setTextSize(2);
      M5.Lcd.print(homeLat, 6);
      M5.Lcd.setCursor(0, 210);
      M5.Lcd.print("LONG: E");
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
  M5.Lcd.drawRoundRect(4, 24, 312, 104, 6, 0x00AF);
  M5.Lcd.drawRoundRect(4, 132, 312, 104, 6, 0x00AF);
  M5.Lcd.drawRoundRect(8, 28, 90, 96, 4, BLUE);
  M5.Lcd.drawRoundRect(222, 28, 90, 96, 4, BLUE);
  M5.Lcd.drawRoundRect(8, 136, 90, 96, 4, BLUE);
  M5.Lcd.fillRect(224, 40, 85, 51, BLACK);
  M5.Lcd.fillRect(224, 91, 85, 31, BLACK);
}

// Zeichne den Liniengraphen (Rate)
void drawRateGraph(float doseRate) {
  int newY = 80 - (doseRate * 10);
  newY = constrain(newY, 40, 80);
  rateGraphBuffer[rateGraphIndex] = newY;
  rateGraphIndex = (rateGraphIndex + 1) % RATE_GRAPH_WIDTH;

  M5.Lcd.fillRect(226, 31, 84, 60, BLACK);
  for (int i = 0; i < RATE_GRAPH_WIDTH - 1; i++) {
    int x1 = 226 + i;
    int y1 = rateGraphBuffer[(rateGraphIndex + i) % RATE_GRAPH_WIDTH];
    int x2 = 226 + (i + 1);
    int y2 = rateGraphBuffer[(rateGraphIndex + i + 1) % RATE_GRAPH_WIDTH];

    uint16_t color = (doseRate < 0.5) ? GREEN : (doseRate < 1.0) ? YELLOW
                                              : (doseRate < 2.0) ? ORANGE
                                                                 : RED;
    M5.Lcd.drawLine(x1, y1, x2, y2, color);
  }
}

// Zeichne das Säulendiagramm (Average)
void drawAverageGraph(float avgDose) {
  avgGraphBuffer[avgGraphIndex] = avgDose * 10;
  avgGraphBuffer[avgGraphIndex] = constrain(avgGraphBuffer[avgGraphIndex], 0, 68);
  avgGraphIndex = (avgGraphIndex + 1) % (AVG_GRAPH_WIDTH / 5);

  M5.Lcd.fillRect(226, 91, 84, 31, BLACK);
  for (int i = 0; i < AVG_GRAPH_WIDTH / 5; i++) {
    int x = 227 + i * 5;
    int height = avgGraphBuffer[(avgGraphIndex + i) % (AVG_GRAPH_WIDTH / 5)];
    int y = 122 - height;

    uint16_t color = (height / 10.0 < 0.5) ? GREEN : (height / 10.0 < 1.0) ? YELLOW
                                                   : (height / 10.0 < 2.0) ? ORANGE
                                                                           : RED;
    M5.Lcd.fillRect(x, y, 4, height, color);
  }
}

// Werteanzeige
void displayValues(float doseRate, float averageDose) {
  
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(225, 31);
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.print("DR:");

  uint16_t rateColor = (doseRate < 0.5) ? GREEN : (doseRate < 1.0) ? YELLOW
                                                : (doseRate < 2.0) ? ORANGE
                                                                   : RED;
  M5.Lcd.setTextColor(rateColor, BLACK);
  M5.Lcd.printf("%.2f uSv/h", doseRate);
  
  M5.Lcd.setCursor(225, 83);
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.print("AD:");

  uint16_t avgColor = (averageDose < 0.5) ? GREEN : (averageDose < 1.0) ? YELLOW
                                                  : (averageDose < 2.0) ? ORANGE
                                                                        : RED;
  M5.Lcd.setTextColor(avgColor, BLACK);
  M5.Lcd.printf("%.2f uSv/h", averageDose);
}

void loop() {

  M5.update();  // Touch-Events aktualisieren

  if (M5.Touch.ispressed()) {                            // Prüfe, ob der Bildschirm berührt wird
    TouchPoint_t touchPoint = M5.Touch.getPressPoint();  // Hol die Berührungskoordinaten
    // Bereich unten links (10x10 mm) zum Ein- und Ausschalten
    if (touchPoint.x >= 0 && touchPoint.x <= 100 && touchPoint.y >= 140 && touchPoint.y <= 240) {
      if (displayOn) {
        M5.Lcd.sleep();             // Display ausschalten
        digitalWrite(TFT_BL, LOW);  // Hintergrundbeleuchtung aus
        displayOn = false;
      } 
      else {
        digitalWrite(TFT_BL, HIGH);  // Hintergrundbeleuchtung wieder einschalten
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

    static float sumDose = 0;
    static int countSamples = 0;
    sumDose += doseRate;
    countSamples++;

    if (countSamples == 80) {
      float avgDose = sumDose / 80;
      drawAverageGraph(avgDose);
      sumDose = 0;
      countSamples = 0;
    }

    drawRateGraph(doseRate);
    displayValues(doseRate, sumDose / countSamples);
  }
  

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
    M5.Lcd.fillRoundRect(100, 136, 214, 96, 4, BLACK);
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
    delay(1000);
    M5.Lcd.fillRoundRect(100, 136, 214, 96, 4, BLACK);
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
  }

  if (M5.BtnB.wasPressed()) {
    M5.Lcd.fillRoundRect(100, 136, 214, 96, 4, BLACK);
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
    delay(1000);
    M5.Lcd.fillRoundRect(100, 136, 214, 96, 4, BLACK);

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
  }
  if (M5.BtnC.wasPressed()) {
    M5.Lcd.fillRoundRect(100, 136, 214, 96, 4, BLACK);
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
    delay(1000);
    M5.Lcd.fillRoundRect(100, 136, 214, 96, 4, BLACK);
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
  smartDelay(940);
  M5.Lcd.fillCircle(160, 77, 47, BLACK);

  //SATELLITES DISPLAY

  //M5.Lcd.drawCircle(160, 77, 47, 0x00AF);
  M5.Lcd.drawCircle(160, 77, 47, BLUE);
  M5.Lcd.drawCircle(160, 77, 16, MAROON);
  M5.Lcd.drawCircle(160, 77, 32, MAROON);
  M5.Lcd.drawFastHLine(113, 77, 95, MAROON);
  M5.Lcd.drawFastVLine(160, 29, 95, MAROON);
  M5.Lcd.drawLine(127, 44, 192, 109, MAROON);
  M5.Lcd.drawLine(127, 109, 192, 44, MAROON);

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
        if (sats[i].snr > 51) {
          sats[i].snr = 50;
        } else if (sats[i].snr > 45) {
          circleColor = 0x03E0;
        } else if (sats[i].snr > 40) {
          circleColor = 0x03E0;
        } else if (sats[i].snr > 35) {
          circleColor = 0xAFE5;
        } else if (sats[i].snr > 30) {
          circleColor = 0xFFE0;
        } else if (sats[i].snr > 25) {
          circleColor = 0x07FF;
        } else if (sats[i].snr > 20) {
          circleColor = 0x03EF;
        } else if (sats[i].snr > 19) {
          circleColor = 0xFD20;
        } else if (sats[i].snr > 18) {
          circleColor = 0x001F;
        } else if (sats[i].snr > 17) {
          circleColor = 0x000F;
        } else if (sats[i].snr > 16) {
          circleColor = 0x7BE0;
        } else if (sats[i].snr > 15) {
          circleColor = 0x780F;
        } else if (sats[i].snr > 14) {
          circleColor = 0x7800;
        } else if (sats[i].snr > 13) {
          circleColor = 0xF81F;
        } else if (sats[i].snr > 12) {
          circleColor = 0xC618;
        } else if (sats[i].snr > 11) {
          circleColor = 0xF800;
        } else if (sats[i].snr > 10) {
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

        // Zeige die Satellitennummer oder SNR
        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(102, 34);
        M5.Lcd.setTextColor(CYAN, BLACK);
        M5.Lcd.print("SATS");
        M5.Lcd.setCursor(102, 44);
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
  for (int i = 0; i < 12; i++) {
    y = (47 * cos(pi - (2 * pi) / 12 * i)) + 77;
    x = (47 * sin(pi - (2 * pi) / 12 * i)) + 160;
    q = (43 * cos(pi - (2 * pi) / 12 * i)) + 77;
    r = (43 * sin(pi - (2 * pi) / 12 * i)) + 160;
    M5.Lcd.drawLine(r, q, x, y, CYAN);
  }

  // CET- berechnen
  if (gps.date.month() <= 3 && gps.date.month() >= 10)  // Winterzeit
  {
    gps.time.hour() + 1;

    y = (32 * cos(pi - (2 * pi) / 60 * ((gps.time.hour() * 5) + gps.time.minute() / 12))) + 77;
    x = (32 * sin(pi - (2 * pi) / 60 * ((gps.time.hour() * 5) + gps.time.minute() / 12))) + 160;

    q = (2 * cos(pi - (2 * pi) / 60 * ((gps.time.hour() * 5) + gps.time.minute() / 12))) + 77;
    r = (2 * sin(pi - (2 * pi) / 60 * ((gps.time.hour() * 5) + gps.time.minute() / 12))) + 160;

    M5.Lcd.drawLine(r + 2, q + 2, x, y, CYAN);
    M5.Lcd.drawLine(r - 2, q - 2, x, y, CYAN);
  } else  // Sommerzeit
  {
    y = (32 * cos(pi - (2 * pi) / 60 * (((gps.time.hour() + 2) * 5) + gps.time.minute() / 12))) + 77;
    x = (32 * sin(pi - (2 * pi) / 60 * (((gps.time.hour() + 2) * 5) + gps.time.minute() / 12))) + 160;

    q = (2 * cos(pi - (2 * pi) / 60 * (((gps.time.hour() + 2) * 5) + gps.time.minute() / 12))) + 77;
    r = (2 * sin(pi - (2 * pi) / 60 * (((gps.time.hour() + 2) * 5) + gps.time.minute() / 12))) + 160;

    M5.Lcd.drawLine(r + 2, q + 2, x, y, CYAN);
    M5.Lcd.drawLine(r - 2, q - 2, x, y, CYAN);
  }

  y = (42 * cos(pi - (2 * pi) / 60 * gps.time.minute())) + 77;
  x = (42 * sin(pi - (2 * pi) / 60 * gps.time.minute())) + 160;
  q = (2 * cos(pi - (2 * pi) / 60 * gps.time.minute())) + 77;
  r = (2 * sin(pi - (2 * pi) / 60 * gps.time.minute())) + 160;

  M5.Lcd.drawLine(r + 1, q + 1, x, y, CYAN);
  M5.Lcd.drawLine(r - 1, q - 1, x, y, CYAN);
  y = (45 * cos(pi - (2 * pi) / 60 * gps.time.second())) + 77;
  x = (45 * sin(pi - (2 * pi) / 60 * gps.time.second())) + 160;
  M5.Lcd.drawLine(160, 76, x, y, RED);
  M5.Lcd.fillCircle(160, 77, 5, MAROON);
  M5.Lcd.fillCircle(160, 77, 3, BLACK);

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
  M5.Lcd.drawRoundRect(12, 53, 82, 12, 2, 0x00AF);
  M5.Lcd.drawRoundRect(12, 65, 82, 12, 2, 0x00AF);
  if ((bme.getPressure_HP() / 100) <= 970) {
    M5.Lcd.setCursor(28, 68);
    M5.Lcd.print(" STORM   ");
  } else if ((bme.getPressure_HP() / 100) <= 1001) {
    if (bme.getTemperature_C() > 1) {
      M5.Lcd.setCursor(28, 68);
      M5.Lcd.print("  RAIN  ");
    } else {
      M5.Lcd.setCursor(28, 68);
      M5.Lcd.print("   SNOW  ");
    }
  } else if ((bme.getPressure_HP() / 100) <= 1010) {
    M5.Lcd.setCursor(28, 68);
    M5.Lcd.print("OVERCAST");
  } else if ((bme.getPressure_HP() / 100) <= 1020) {
    M5.Lcd.setCursor(28, 68);
    M5.Lcd.print(" CLOUDY  ");
  } else if ((bme.getPressure_HP() / 100) <= 1040) {
    M5.Lcd.setCursor(28, 68);
    M5.Lcd.print("  CLEAR   ");
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
  //M5.Lcd.fillRoundRect(223, 29, 88, 94, 3, BLACK);
  M5.Lcd.fillRoundRect(13, 78, 80, 9, 2, BLACK);
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
  M5.Lcd.fillRoundRect(13, 89, 80, 9, 2, BLACK);
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
  M5.Lcd.fillRoundRect(13, 100, 80, 9, 2, BLACK);
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
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.setCursor(106, 137);
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
  M5.Lcd.setCursor(106, 157);
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
  M5.Lcd.setCursor(106, 177);
  M5.Lcd.print("ALTI:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    M5.Lcd.print("---");
  } else {
    M5.Lcd.print(gps.altitude.meters(), 2);
    M5.Lcd.print("m   ");
  }
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.setCursor(106, 197);
  M5.Lcd.print("SPED:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    M5.Lcd.print("---");
  } else {

    M5.Lcd.print(gps.speed.kmph(), 0);
    M5.Lcd.print("km/h  ");
  }
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.setCursor(106, 217);
  M5.Lcd.print("HOME:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  if ((gps.location.lat() == 0) && (gps.location.lng() == 0)) {
    M5.Lcd.print("---");

  } else if (TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) < 1000) {
    M5.Lcd.fillRoundRect(164, 217, 150, 14, 2, BLACK);
    M5.Lcd.print(TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon), 0);
    M5.Lcd.print("m    ");
  } else {
    M5.Lcd.fillRoundRect(164, 217, 150, 14, 2, BLACK);
    M5.Lcd.print(TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000, 2);
    M5.Lcd.print("km   ");
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

