#include <M5Core2.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "cactus_io_BME280_I2C.h"
#include <SD.h>
BME280_I2C bme(0x76);
double homeLat = 0.0;
double homeLon = 0.0;
static const int RXPin = 13, TXPin = 14;
static const uint32_t GPSBaud = 9600;
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
boolean GPSnotReady = false;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
static const int MAX_SATELLITES = 40;
TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1);
TinyGPSCustom messageNumber(gps, "GPGSV", 2);
TinyGPSCustom satsInView(gps, "GPGSV", 3);
TinyGPSCustom satNumber[4];
TinyGPSCustom elevation[4];
TinyGPSCustom azimuth[4];
TinyGPSCustom snr[4];
struct
{
  bool active;
  int elevation;
  int azimuth;
  int snr;
} sats[MAX_SATELLITES];
int sensorPin1 = 35;//CO OK
int sensorPin2 = 40;//NO2 OK
int sensorPin3 = 35;//NH3 OK//int sensorPin3 = 34;//NH3 OK
int sensorPin4 = 34;//EMF 12

int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;
int sensorValue4 = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(32, 33);
  M5.begin(true, true, true, true);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.setTextSize(2);
  M5.Axp.SetLed(false);
  bme.begin();
  bme.setTempCal(-1);
  pinMode(35, INPUT);
  pinMode(36, INPUT);
  pinMode(25, INPUT);
  pinMode(26, INPUT);
  ss.begin(GPSBaud);
  for (int i = 0; i < 4; ++i) {
    satNumber[i].begin(gps, "GPGSV", 4 + 4 * i);
    elevation[i].begin(gps, "GPGSV", 5 + 4 * i);
    azimuth[i].begin(gps, "GPGSV", 6 + 4 * i);
    snr[i].begin(gps, "GPGSV", 7 + 4 * i);
 
   File myFile = SD.open("home_coordinates.txt", FILE_READ);
  if (myFile) {
    String coordinates = myFile.readStringUntil('\n');
    int commaIndex = coordinates.indexOf(',');
    homeLat = coordinates.substring(0, commaIndex).toFloat();
    homeLon = coordinates.substring(commaIndex + 1).toFloat();
    myFile.close();
  } else {
    Serial.println("Error opening file");
  }
  }

  //GRAPHIC
  M5.Lcd.drawRoundRect(0, 0, 320, 240, 8, BLUE);
  M5.Lcd.drawRoundRect(4, 24, 312, 104, 6, 0x00AF);
  M5.Lcd.drawRoundRect(4, 132, 312, 104, 6, 0x00AF);
  M5.Lcd.drawRoundRect(8, 28, 90, 96, 4, BLUE);
  M5.Lcd.drawRoundRect(222, 28, 90, 96, 4, BLUE);
  M5.Lcd.drawRoundRect(8, 136, 90, 96, 4, BLUE);
}
void loop() {
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
        GPSnotReady = ((gps.location.lat() == 0) && (gps.location.lng() == 0));
    }
  }
  //TOUCH BUTTONS
  M5.update();
  if (M5.BtnA.wasPressed() == true) {
    //M5.Axp.SetSleep(0);
    M5.Lcd.fillRoundRect(100, 136, 214, 96, 4, BLACK);                // HOME POSITION
    static const double homeLat = 51.8610070, homeLon = 8.2893300;  //YOUR HOME POSITION
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.setCursor(108, 137);
    M5.Lcd.print("HOME POSITION");
    M5.Lcd.setCursor(108, 157);
    M5.Lcd.print("LATT: 51.8610070");
    M5.Lcd.setCursor(108, 177);
    M5.Lcd.print("LONG: 8.2893300");
    delay(5000);
    M5.Lcd.fillRoundRect(100, 136, 214, 96, 4, BLACK);
  }
  if (M5.BtnB.wasPressed() == true) {
    M5.Lcd.fillRoundRect(100, 136, 214, 96, 4, BLACK);  // CURRENT POSITION
    static double homeLat = gps.location.lat(), homeLon = gps.location.lng();  // Speichern der aktuellen Koordinaten
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.setCursor(108, 137);
    M5.Lcd.print("POSITION FIXED");
    M5.Lcd.setCursor(108, 157);
    M5.Lcd.print("LATT:");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print(" \xF7");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print(gps.location.lat(), 6);
    M5.Lcd.setCursor(108, 177);
    M5.Lcd.print("LONG:");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print(" \xF7");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print(gps.location.lng(), 6);
    delay(5000);
    M5.Lcd.fillRoundRect(100, 136, 214, 96, 4, BLACK);

    // Schreibe die Koordinaten in die SD-Karte
   File myFile = SD.open("home_coordinates.txt", FILE_WRITE);
if (myFile) {
  myFile.print(gps.location.lat(), 6);
  myFile.print(",");
  myFile.println(gps.location.lng(), 6);
  myFile.close();
  homeLat = gps.location.lat();
  homeLon = gps.location.lng();
  M5.Lcd.setCursor(108, 137);
  M5.Lcd.print("POSITION FIXED");
  delay(2000);
} else {
  Serial.println("Error opening file");
}
  }

  if (M5.BtnC.wasPressed() == true) {
    M5.Lcd.fillRoundRect(100, 136, 214, 96, 4, BLACK);
    static const double homeLat = 78.2355500, homeLon = 15.491380;
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.setCursor(108, 137);
    M5.Lcd.print("Svalbard GSV");
    M5.Lcd.setCursor(108, 157);
    M5.Lcd.print("LATT: 78.238200");
    M5.Lcd.setCursor(108, 177);
    M5.Lcd.print("LONG: 15.447200");
    delay(5000);
    M5.Lcd.fillRoundRect(100, 136, 214, 96, 4, BLACK);
  }
    File myFile = SD.open("position.txt", FILE_READ);
if (myFile) {
  String line = myFile.readStringUntil('\n');
  int commaIndex = line.indexOf(",");
  if (commaIndex != -1) {
    homeLat = line.substring(0, commaIndex).toDouble();
    homeLon = line.substring(commaIndex+1).toDouble();
  }
  myFile.close();
}
  unsigned long distanceToHome =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      homeLat,
      homeLon);  // 1000;
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
  //SATELLITES DISPLAY
  M5.Lcd.fillCircle(160, 77, 47, BLACK);
  M5.Lcd.drawCircle(160, 77, 16, 0x00AF);
  M5.Lcd.drawCircle(160, 77, 32, 0x00AF);
  M5.Lcd.drawCircle(160, 77, 48, BLUE);
  M5.Lcd.drawFastHLine(113, 77, 95, 0x00AF);
  M5.Lcd.drawFastVLine(160, 29, 95, 0x00AF);
  M5.Lcd.drawLine(127, 44, 192, 109, 0x00AF);
  M5.Lcd.drawLine(127, 109, 192, 44, 0x00AF);
  //SATELITES VISUALISATION
  //j = 0;
  if (!GPSnotReady) {
    for (int i = 0; i < MAX_SATELLITES; ++i)
      if (sats[i].active) {
        if (sats[i].snr > 1) {
          az = (sats[i].azimuth);
          el = (sats[i].elevation) / 2;
          az_r = az * rad_fac;
          e = 40 * ((90 - el)) / 100;
          x = 160 + (sin(az_r) * e);
          y = 75 - (cos(az_r) * e);
          M5.Lcd.drawCircle(x, y, 1, WHITE);

          if ((sats[i].snr) > 20) {
            M5.Lcd.drawCircle(x, y, 6, GREEN);
          } else if ((sats[i].snr) > 19) {
            M5.Lcd.drawCircle(x, y, 5, YELLOW);
          }

          else if ((sats[i].snr) > 16) {
            M5.Lcd.drawCircle(x, y, 4, ORANGE);
          } else if ((sats[i].snr) > 13) {
            M5.Lcd.drawCircle(x, y, 3, RED);
          } else if ((sats[i].snr) > 10) {
            M5.Lcd.drawCircle(x, y, 2, PURPLE);
          }
          // if ( sats[i].snr > 41)
          //{
          // ( sats[i].snr = 40);
          // }
          //else if (j < 32) {
          //M5.Lcd.drawFastHLine(238, 76, 66, DARKCYAN);
          //M5.Lcd.drawRect(240 + (j * 2.5 ), 76 - (sats[i].snr / 1.2 ), 1 , (sats[i].snr / 1.2) * 2 , GREEN);
          //SIGNAL//
          // M5.Lcd.setTextColor(CYAN , BLACK);
          //M5.Lcd.setTextSize(1);
          // M5.Lcd.setCursor(240, 108);
          // M5.Lcd.print("SIGNAL:");
          // M5.Lcd.setTextColor(GREEN , BLACK);
          // M5.Lcd.print(sats[i].snr);
          // M5.Lcd.println(" ");


          //SATELL//
          //M5.Lcd.setTextColor(CYAN , BLACK);
          //M5.Lcd.setTextSize(1);
          //M5.Lcd.fillRect(240, 32, 15, 7, BLACK);
          //M5.Lcd.setCursor(238, 32);
          // M5.Lcd.print("SATELL:");
          // if ((j) < 3) {
          // M5.Lcd.setTextColor(RED , BLACK);
          //M5.Lcd.print(j  );
          // M5.Lcd.println(" ");
          //}
          //else {
          //  M5.Lcd.setTextColor(GREEN , BLACK);
          //  M5.Lcd.print(j);
          //  M5.Lcd.println(" ");
          // }
          //}
          // j = j + 1;
        }
      }
  }
  //smartDelay(10);
  M5.Lcd.fillCircle(53, 184, 34, BLACK);
  double relCourse = courseToHome - gps.course.deg();
  if (relCourse < 0) {
    relCourse = relCourse + 360;
  }
  if (relCourse >= 360) {
  }
  relCourse = relCourse - 360;
  if (distanceToHome < 25) {

    //M5.Lcd.drawCircle(53, 184, 16, YELLOW);
    //M5.Lcd.drawCircle(53, 184, 25, GREEN);

    // M5.Lcd.drawTriangle(53, 151, 53, 184, 48, 178, BLUE); //1
    //M5.Lcd.drawTriangle(53, 151, 53, 184, 58, 178, 0x00AF); //2
    //M5.Lcd.drawTriangle(88, 184, 53, 184, 58, 178, BLUE); //3
    // M5.Lcd.drawTriangle(88, 184, 53, 184, 58, 190, 0x00AF); //4
    //M5.Lcd.drawTriangle(53, 183, 52, 218, 58, 190, BLUE); //5
    // M5.Lcd.drawTriangle(53, 183, 54, 218, 48, 190, 0x00AF); //6
    // M5.Lcd.drawTriangle(19, 184, 53, 184, 48, 190, BLUE); //7
    // M5.Lcd.drawTriangle(19, 184, 53, 184, 48, 178, 0x00AF); //8
    M5.Lcd.fillTriangle(53, 151, 53, 184, 48, 178, BLUE);    //1
    M5.Lcd.fillTriangle(53, 151, 53, 184, 58, 178, 0x00AF);  //2
    M5.Lcd.fillTriangle(88, 184, 53, 184, 58, 178, BLUE);    //3
    M5.Lcd.fillTriangle(88, 184, 53, 184, 58, 190, 0x00AF);  //4
    M5.Lcd.fillTriangle(53, 184, 53, 218, 58, 190, BLUE);    //5
    M5.Lcd.fillTriangle(53, 184, 53, 218, 48, 190, 0x00AF);  //6
    M5.Lcd.fillTriangle(19, 184, 53, 184, 48, 190, BLUE);    //7
    M5.Lcd.fillTriangle(19, 184, 53, 184, 48, 178, 0x00AF);  //8
    //M5.Lcd.setTextSize(1);
    //M5.Lcd.setTextColor(BLACK);
    //M5.Lcd.setCursor(51, 145);
    //M5.Lcd.print("N");
    //M5.Lcd.setCursor(51, 215);
    //M5.Lcd.print("S");
    //M5.Lcd.setCursor(11, 181);
    //M5.Lcd.print("W");
    //M5.Lcd.setCursor(89, 181);
    //M5.Lcd.print("E");
    //M5.Lcd.drawCircle(53, 184, 35, BLUE );
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
    //M5.Lcd.drawLine(x1, y1, x2, y2, GREEN);
    double arrowAngle = relCourse + 200;
    x = (sin(arrowAngle * rad_fac) * 48);
    y = (cos(arrowAngle * rad_fac) * 48);
    q = (sin(arrowAngle * rad_fac) * 48);
    r = (cos(arrowAngle * rad_fac) * 48);
    x2 = x1 + x;
    y2 = y1 - y;
    q2 = q1 + q;
    r2 = r1 - r;
    M5.Lcd.drawLine(x1, y1, x2, y2, YELLOW);
    M5.Lcd.drawLine(q1, r1, x2, y2, YELLOW);
    arrowAngle = relCourse + 160;
    x = (sin(arrowAngle * rad_fac) * 48);
    y = (cos(arrowAngle * rad_fac) * 48);
    q = (sin(arrowAngle * rad_fac) * 48);
    r = (cos(arrowAngle * rad_fac) * 48);
    x2 = x1 + x;
    y2 = y1 - y;
    q2 = q1 + q;
    r2 = r1 - r;
    M5.Lcd.drawLine(x1, y1, x2, y2, YELLOW);
    M5.Lcd.drawLine(q1, r1, x2, y2, YELLOW);
    
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setCursor(51, 141);
    M5.Lcd.print("N");
    M5.Lcd.setCursor(11, 181);
    M5.Lcd.print("W");
    M5.Lcd.setCursor(89, 181);
    M5.Lcd.print("E");
    
  }
  //COORDINATES
  if (distanceToHome > 25) {
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
  for (int i = 0; i < 60; i++) {
    y = (47 * cos(pi - (2 * pi) / 60 * i)) + 77;
    x = (47 * sin(pi - (2 * pi) / 60 * i)) + 160;
    q = (47 * cos(pi - (2 * pi) / 60 * i)) + 77;
    r = (47 * sin(pi - (2 * pi) / 60 * i)) + 160;
    M5.Lcd.drawLine(r, q, x, y, WHITE);
  }

  //ANALOG WATCH
  
  y = (32 * cos(pi - (2 * pi) / 60 * ((gps.time.hour() * 5) + gps.time.minute() / 12))) + 77;
  x = (32 * sin(pi - (2 * pi) / 60 * ((gps.time.hour() * 5) + gps.time.minute() / 12))) + 160;

  q = (2 * cos(pi - (2 * pi) / 60 * ((gps.time.hour() * 5) + gps.time.minute() / 12))) + 77;
  r = (2 * sin(pi - (2 * pi) / 60 * ((gps.time.hour() * 5) + gps.time.minute() / 12))) + 160;
  //******************************************************************************************************************************************
  M5.Lcd.drawLine(r + 2, q + 2, x, y, WHITE);
  M5.Lcd.drawLine(r - 2, q - 2, x, y, WHITE);
  y = (42 * cos(pi - (2 * pi) / 60 * gps.time.minute())) + 77;
  x = (42 * sin(pi - (2 * pi) / 60 * gps.time.minute())) + 160;
  q = (2 * cos(pi - (2 * pi) / 60 * gps.time.minute())) + 77;
  r = (2 * sin(pi - (2 * pi) / 60 * gps.time.minute())) + 160;
  M5.Lcd.drawLine(r + 1, q + 1, x, y, CYAN);
  M5.Lcd.drawLine(r - 1, q - 1, x, y, CYAN);
  y = (45 * cos(pi - (2 * pi) / 60 * gps.time.second())) + 77;
  x = (45 * sin(pi - (2 * pi) / 60 * gps.time.second())) + 160;
  M5.Lcd.drawLine(160, 76, x, y, RED);
  M5.Lcd.fillCircle(160, 77, 5, DARKCYAN);
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
 
  int sensorValue1 = (analogRead(sensorPin1));
  int sensorValue2 = (analogRead(sensorPin2));
  int sensorValue3 = (analogRead(sensorPin3));
  int sensorValue4 = (analogRead(sensorPin4));

  float CO  = (-sensorValue1+ 4095);//float CO = (-sensorValue1 + 4095);
  float NH3 = (-sensorValue3+ 4095);
  float NO2 = (-sensorValue2+ 4095);
  float EMF = (-sensorValue4+ 4095);

  //GAS SENSOR MICS-6814 MODULE ANALOG OUT

  M5.Lcd.fillRoundRect(13, 78, 80, 9, 2, BLACK);
  if (CO  > 3800) {
    M5.Lcd.drawRoundRect(12, 77, 82, 11, 2, RED);
    M5.Lcd.setCursor(16, 79);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("CO :");
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print(CO  / 40.95);
  } else if (CO  > 3200) {
    M5.Lcd.drawRoundRect(12, 77, 82, 11, 2, ORANGE);
    M5.Lcd.setCursor(16, 79);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("CO :");
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.print(CO  / 40.95);
  } else {
    M5.Lcd.drawRoundRect(12, 77, 82, 11, 2, 0x00AF);
    M5.Lcd.setCursor(16, 79);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("CO :");
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print(CO  / 40.95);
  }
  M5.Lcd.fillRoundRect(223, 29, 88, 94, 3, BLACK);
  if (CO  > 4000) {
    M5.Lcd.fillRoundRect(226, 30, 19, 11, 3, RED);
  } else if (CO  > 3800) {
    M5.Lcd.fillRoundRect(226, 40, 19, 11, 3, RED);
  } else if (CO  > 3600) {
    M5.Lcd.fillRoundRect(226, 50, 19, 11, 3, ORANGE);
  } else if (CO  > 3400) {
    M5.Lcd.fillRoundRect(226, 60, 19, 11, 3, ORANGE);
  } else if (CO  > 3300) {
    M5.Lcd.fillRoundRect(226, 70, 19, 11, 3, YELLOW);
  } else if (CO  > 2000) {
    M5.Lcd.fillRoundRect(226, 80, 19, 11, 3, YELLOW);
  } else if (CO  > 1000) {
    M5.Lcd.fillRoundRect(226, 90, 19, 11, 3, GREENYELLOW);
  } else if (CO  > 200) {
    M5.Lcd.fillRoundRect(226, 100, 19, 11, 3, GREEN);
  } else if (CO  > 0) {
    M5.Lcd.fillRoundRect(226, 110, 19, 11, 3, GREEN);
  }
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
    M5.Lcd.setTextColor(GREENYELLOW, BLACK);
    M5.Lcd.print(NH3 / 40.95);
  }
  if (NH3 > 4000) {
    M5.Lcd.fillRoundRect(247, 30, 19, 11, 3, RED);
  } else if (NH3 > 3500) {

    M5.Lcd.fillRoundRect(247, 40, 19, 11, 3, RED);
  } else if (NH3 > 3000) {
    M5.Lcd.fillRoundRect(247, 50, 19, 11, 3, RED);
  } else if (NH3 > 2500) {
    M5.Lcd.fillRoundRect(247, 60, 19, 11, 3, ORANGE);
  } else if (NH3 > 2000) {
    M5.Lcd.fillRoundRect(247, 70, 19, 11, 3, ORANGE);
  } else if (NH3 > 1500) {
    M5.Lcd.fillRoundRect(247, 80, 19, 11, 3, YELLOW);
  } else if (NH3 > 1200) {
    M5.Lcd.fillRoundRect(247, 90, 19, 11, 3, GREENYELLOW);
  } else if (NH3 > 1000) {
    M5.Lcd.fillRoundRect(247, 100, 19, 11, 3, GREEN);
  } else if (NH3 > 0) {
    M5.Lcd.fillRoundRect(247, 110, 19, 11, 3, GREEN);
  }
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
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print(NO2 / 40.95);
  }
  if (NH3 > 4000) {
    M5.Lcd.fillRoundRect(268, 30, 19, 11, 3, RED);
  } else if (NO2 > 3500) {
    M5.Lcd.fillRoundRect(268, 40, 19, 11, 3, RED);
  } else if (NO2 > 3000) {
    M5.Lcd.fillRoundRect(268, 50, 19, 11, 3, RED);
  } else if (NO2 > 2500) {
    M5.Lcd.fillRoundRect(268, 60, 19, 11, 3, ORANGE);
  } else if (NO2 > 1000) {
    M5.Lcd.fillRoundRect(268, 70, 19, 11, 3, ORANGE);
  } else if (NO2 > 500) {
    M5.Lcd.fillRoundRect(268, 80, 19, 11, 3, YELLOW);
  } else if (NO2 > 200) {
    M5.Lcd.fillRoundRect(268, 90, 19, 11, 3, GREENYELLOW);
  } else if (NO2 > 100) {
    M5.Lcd.fillRoundRect(268, 100, 19, 11, 3, GREEN);
  } else if (NO2 > 0) {
    M5.Lcd.fillRoundRect(268, 110, 19, 11, 3, GREEN);
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
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.print(EMF / 40.95);
  }
  if (EMF > 4000) {
    M5.Lcd.fillRoundRect(289, 30, 18, 11, 3, RED);
  } else if (EMF > 3500) {
    M5.Lcd.fillRoundRect(289, 40, 18, 11, 3, RED);
  } else if (EMF > 3000) {
    M5.Lcd.fillRoundRect(289, 50, 18, 11, 3, RED);
  } else if (EMF > 2500) {
    M5.Lcd.fillRoundRect(289, 60, 18, 11, 23, ORANGE);
  } else if (EMF > 2200) {
    M5.Lcd.fillRoundRect(289, 70, 18, 11, 23, ORANGE);
  } else if (EMF > 1000) {
    M5.Lcd.fillRoundRect(289, 80, 18, 11, 3, YELLOW);
  } else if (EMF > 400) {
    M5.Lcd.fillRoundRect(289, 90, 18, 11, 3, GREENYELLOW);
  } else if (EMF > 200) {
    M5.Lcd.fillRoundRect(289, 100, 18, 11, 3, GREEN);
  } else if (EMF > 0) {
    M5.Lcd.fillRoundRect(289, 110, 18, 11, 3, GREEN);
  }
  M5.Lcd.drawRoundRect(225, 110, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(246, 110, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(267, 110, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(288, 110, 21, 11, 2, 0x00AF);

  M5.Lcd.drawRoundRect(225, 100, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(246, 100, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(267, 100, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(288, 100, 21, 11, 2, 0x00AF);

  M5.Lcd.drawRoundRect(225, 90, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(246, 90, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(267, 90, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(288, 90, 21, 11, 2, 0x00AF);

  M5.Lcd.drawRoundRect(225, 80, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(246, 80, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(267, 80, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(288, 80, 21, 11, 2, 0x00AF);

  M5.Lcd.drawRoundRect(225, 70, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(246, 70, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(267, 70, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(288, 70, 21, 11, 2, 0x00AF);

  M5.Lcd.drawRoundRect(225, 60, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(246, 60, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(267, 60, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(288, 60, 21, 11, 2, 0x00AF);

  M5.Lcd.drawRoundRect(225, 50, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(246, 50, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(267, 50, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(288, 50, 21, 11, 2, 0x00AF);

  M5.Lcd.drawRoundRect(225, 40, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(246, 40, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(267, 40, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(288, 40, 21, 11, 2, 0x00AF);

  M5.Lcd.drawRoundRect(225, 30, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(246, 30, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(267, 30, 21, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(288, 30, 21, 11, 2, 0x00AF);
 
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
    M5.Lcd.print("km/h    ");
  }
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.setCursor(106, 217);
  M5.Lcd.print("HOME:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  if ((gps.location.lat() == 0) && (gps.location.lng() == 0)) {
    M5.Lcd.print("---");
  } else if (TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) < 1000) {
    M5.Lcd.print(TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon), 0);
    M5.Lcd.print("m   ");
  } else {
    M5.Lcd.print(TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000, 2);
    M5.Lcd.print("km   ");
  }
  M5.Lcd.fillRect(12, 222, 80, 9, BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(RED);

  // M5.Lcd.setCursor(51, 141);
  // M5.Lcd.print("N");
  M5.Lcd.setCursor(51, 222);
  M5.Lcd.print("S");
  //M5.Lcd.setCursor(11, 181);
  //M5.Lcd.print("W");
  // M5.Lcd.setCursor(89, 181);
  // M5.Lcd.print("E");
 
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(RED, BLACK);
  M5.Lcd.setCursor(16, 222);
  if (((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) * 60 > 43200) {
    M5.Lcd.fillRect(12, 222, 80, 9, BLACK);
    M5.Lcd.print("      S");
    
  } else if (((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) * 60 == 1440) {
    M5.Lcd.fillRect(12, 222, 80, 9, BLACK);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print(((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) / 24, 1);
    M5.Lcd.print(" DAY");
  } else if (((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) * 60 >= 1441) {
    M5.Lcd.fillRect(12, 222, 80, 9, BLACK);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print(((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000) / 3) / 24, 1);
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
    M5.Lcd.print("");
  }

}  //LOOP END
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
    int flen = prec + (val < 0.0 ? 2 : 1);  // . and -
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
//DATE/TIME//
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  if (!d.isValid()) {
    //Serial.print(F("***"));
    M5.Lcd.setCursor(6, 6);
    M5.Lcd.print(F(">> NO GPS SIGNAL <<"));
  } else {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d:", d.year(), d.month(), d.day());
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.setCursor(6, 6);
    M5.Lcd.setTextSize(2);
    M5.Lcd.print(sz);
    if (!t.isValid()) {
      Serial.print(F("***"));
    } else {
      char sz[32];
      sprintf(sz, "%02d:%02d:%02d", t.hour(), t.minute(), t.second());
      //M5.Lcd.setTextColor(CYAN , BLACK);
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
