#include <Arduino.h>
#include "wiring_private.h"  // Pour configurer les pins SERCOM

#include <Adafruit_GPS.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

#define BUZZER_PIN 5
#define BUZZ_DURATION_MS 500
#define NO_FIX_BEEP_INTERVAL_MS 1000
unsigned long lastNoFixBeepTime = 0;

#define DEBUGSerial Serial
const int chipSelect = 7;
const int timezone_offset = 2;

// ----- GPS sur SERCOM3 -----
Uart SerialGPS(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
Adafruit_GPS GPS(&SerialGPS);

// ----- IMU -----
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// ----- FICHIER -----
File dataFile;
bool fileOpened = false;
bool headerWritten = false;
String fileName = "";

uint16_t last_millis_shown = 0xFFFF;

float convertNMEAToDecimal(float val) {
  int deg = int(val / 100);
  float minutes = val - (deg * 100);
  return deg + (minutes / 60.0);
}

void listFiles() {
  DEBUGSerial.println("Files on SD:");
  File root = SD.open("/");
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    DEBUGSerial.print("  ");
    DEBUGSerial.print(entry.name());
    DEBUGSerial.print("  ");
    DEBUGSerial.println(entry.size());
    entry.close();
  }
  root.close();
}

void printCardInfo() {
  File root = SD.open("/");
  if (!root) {
    DEBUGSerial.println("Erreur d'accès au système de fichiers.");
    return;
  }

  uint32_t totalSize = 0;
  uint16_t fileCount = 0;

  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    totalSize += entry.size();
    fileCount++;
    entry.close();
  }

  DEBUGSerial.print("Nombre de fichiers : ");
  DEBUGSerial.println(fileCount);
  DEBUGSerial.print("Taille totale occupée : ");
  DEBUGSerial.print(totalSize / 1024.0, 2);
  DEBUGSerial.println(" Ko");

  root.close();
}

String generateLogFileName() {
  char filename[13];
  for (int i = 0; i < 10000; i++) {
    sprintf(filename, "LOG%04d.CSV", i);
    if (!SD.exists(filename)) {
      return String(filename);
    }
  }
  return "LOGFAIL.CSV";
}

String buildDataLine(uint16_t millis) {
  String line = "";

  float lat = convertNMEAToDecimal(GPS.latitude);
  if (GPS.lat == 'S') lat = -lat;
  float lon = convertNMEAToDecimal(GPS.longitude);
  if (GPS.lon == 'W') lon = -lon;

  line += String(lat, 6) + ",";
  line += String(lon, 6) + ",";
  line += String(GPS.altitude) + ",";
  line += String(GPS.day) + ",";
  line += String(GPS.month) + ",";
  line += String(2000 + GPS.year) + ",";

  int local_hour = (GPS.hour + timezone_offset) % 24;
  if (local_hour < 0) local_hour += 24;
  line += String(local_hour) + ",";
  line += String(GPS.minute) + ",";
  line += String(GPS.seconds) + ",";
  line += String(millis) + ",";
  line += String(GPS.PDOP, 1) + ",";
  line += String(GPS.speed * 1.852, 2); // vitesse km/h

  sensors_event_t orientation, gyro, accel, mag, linear, grav;
  bno.getEvent(&orientation, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&mag, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&linear, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&grav, Adafruit_BNO055::VECTOR_GRAVITY);

  auto addVec = [&](sensors_event_t e) {
    line += "," + String(e.orientation.x);
    line += "," + String(e.orientation.y);
    line += "," + String(e.orientation.z);
  };

  addVec(orientation);
  addVec(gyro);
  addVec(accel);
  addVec(mag);
  addVec(linear);
  addVec(grav);

  int8_t temp = bno.getTemp();
  uint8_t sys, gcal, acal, mcal;
  bno.getCalibration(&sys, &gcal, &acal, &mcal);

  line += "," + String(temp);
  line += "," + String(sys);
  line += "," + String(gcal);
  line += "," + String(acal);
  line += "," + String(mcal);

  return line;
}

void writeHeader(File& out) {
  out.print("lat,long,height,day,month,year,hour,minute,second,millis,pdop,speed_kmh");
  out.print(",ori_x,ori_y,ori_z");
  out.print(",gyro_x,gyro_y,gyro_z");
  out.print(",acc_x,acc_y,acc_z");
  out.print(",mag_x,mag_y,mag_z");
  out.print(",lin_x,lin_y,lin_z");
  out.print(",grav_x,grav_y,grav_z");
  out.print(",temp,cal_sys,cal_gyro,cal_accel,cal_mag");
  out.println();
}

void beepBuzzer(unsigned long duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
}

void setup() {
  DEBUGSerial.begin(115200);
  delay(500);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Configuration des pins pour le GPS SERCOM3
  pinPeripheral(0, PIO_SERCOM);
  pinPeripheral(1, PIO_SERCOM);
  SerialGPS.begin(9600);
  delay(1000);

  DEBUGSerial.println("Initializing GPS...");
  beepBuzzer(BUZZ_DURATION_MS);
  GPS.begin(9600);
  GPS.sendCommand("$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
  delay(100);
  GPS.sendCommand("$PMTK251,115200*1F");
  delay(100);
  SerialGPS.begin(115200);
  GPS.begin(115200);
  GPS.sendCommand("$PMTK220,100*2F");

  DEBUGSerial.println("Initializing IMU...");
  beepBuzzer(BUZZ_DURATION_MS);
  if (!bno.begin()) {
    DEBUGSerial.println("BNO055 not detected.");
    while (1);
  }

  DEBUGSerial.print("Initializing SD card...");
  beepBuzzer(BUZZ_DURATION_MS);
  if (!SD.begin(chipSelect)) {
    DEBUGSerial.println("FAILED!");
    while (1);
  }
  DEBUGSerial.println("OK");
  listFiles();
  printCardInfo();
  beepBuzzer(BUZZ_DURATION_MS);  // Bip final après init OK
}

void loop() {
  GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());

    if (GPS.fix) {
      uint16_t current_millis = GPS.milliseconds;
      if (current_millis == last_millis_shown) return;
      last_millis_shown = current_millis;

      if (!fileOpened) {
        fileName = generateLogFileName();
        dataFile = SD.open(fileName.c_str(), FILE_WRITE);
        if (dataFile) {
          writeHeader(dataFile);
          fileOpened = true;
          DEBUGSerial.print("Started logging to ");
          DEBUGSerial.println(fileName);
        } else {
          DEBUGSerial.println("Failed to open file for writing.");
          return;
        }
      }

      String line = buildDataLine(current_millis);
      dataFile.println(line);
      dataFile.flush();
      DEBUGSerial.println(line);

      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);

    } else {
      if (fileOpened) {
        dataFile.close();
        DEBUGSerial.println("Fix perdu. Fichier fermé.");
        fileOpened = false;
      }

      unsigned long now = millis();
      if (now - lastNoFixBeepTime >= NO_FIX_BEEP_INTERVAL_MS) {
        lastNoFixBeepTime = now;
        beepBuzzer(BUZZ_DURATION_MS);
      }
    }
  }
}

// Handler d'interruption pour le SERCOM3
void SERCOM3_Handler() {
  SerialGPS.IrqHandler();
}
