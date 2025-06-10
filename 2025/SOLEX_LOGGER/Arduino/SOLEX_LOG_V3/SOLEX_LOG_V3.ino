#include <LoRa.h>
#include <Arduino.h>
#include "wiring_private.h"  // Pour configurer les pins SERCOM
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <VescUart.h>

//Define 
#define CHIP_SELECT_SD 7
#define BUZZER_PIN 2

#define BUZZ_DURATION_MS 500
#define NO_FIX_BEEP_INTERVAL_MS 1000

#define DEBUGSerial Serial
#define VESCSerial Serial1

#define LCD_NO_INFO 0
#define LCD_NO_SD 1
#define LCD_SD_OK 2



// Variables globales
bool vesc_status = false;
bool sd_status = false;
bool gps_status = false;
bool imu_status = false;
bool LoRa_status = false;

unsigned long lastNoFixBeepTime = 0; 
const int timezone_offset = 2;


File dataFile;
bool fileOpened = false;

String fileName = "";
uint16_t last_millis_shown = 0xFFFF;

// ----- LCD -----
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ----- GPS sur SERCOM3 -----
Uart SerialGPS(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
Adafruit_GPS GPS(&SerialGPS);

// ----- IMU -----
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// ----- VESC ----- 
VescUart VESC;


float convertNMEAToDecimal(float val) {
  int deg = int(val / 100);
  float minutes = val - (deg * 100);
  return deg + (minutes / 60.0);
}

void beepBuzzer(unsigned long duration) {
  //digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  //digitalWrite(BUZZER_PIN, LOW);
}

void lcd_static() {
  lcd.setCursor(0, 0);
  lcd.print("   C SD:           C");
  lcd.setCursor(0, 1);
  lcd.print("   A GPS:          A");
  lcd.setCursor(0, 3);
  lcd.print("    V     %     km/h");
  lcd_current_g(-1);
  lcd_current_d(-1);
  lcd_temp_g(-1);
  lcd_temp_d(-1);
  lcd_battery_voltage(-1);
  lcd_battery_level(-1);
  lcd_speed(-1);
  lcd_gps_fix(false);
  lcd_sd_status(LCD_NO_INFO);
}

void lcd_current_g(uint current){
  lcd.setCursor(0, 0);
  if(current>200 || current<0) {
    lcd.print("---");
    return;
  }
  char update[4];
  sprintf(update,"%3d",current);
  lcd.print(update);
}
void lcd_current_d(uint current){
  lcd.setCursor(16, 0);
  if(current>200 || current<0) {
    lcd.print("---");
    return;
  }
  char update[4];
  sprintf(update,"%3d",current);
  lcd.print(update);
}
void lcd_temp_g(uint temp){
  lcd.setCursor(0, 1);
  if(temp>150 || temp<0) {
    lcd.print("---");
    return;
  }
  char update[4];
  sprintf(update,"%3d",temp);
  lcd.print(update);
}
void lcd_temp_d(uint temp){
  lcd.setCursor(16, 1);
  if(temp>150 || temp<0) {
    lcd.print("---");
    return;
  }
  char update[4];
  sprintf(update,"%3d",temp);
  lcd.print(update);
}
void lcd_battery_voltage(float voltage) {
  lcd.setCursor(0, 3);
  if(voltage>55 || voltage < 0) {
      lcd.print("---");
      return;
    }
  int int_part = (int)voltage;
  int dec_part = (int)(voltage * 10) % 10;  // pour 1 chiffre après la virgule
  char buffer[8];
  sprintf(buffer, "%2d.%01d", int_part, dec_part);

  lcd.print("    ");        // Efface ancien affichage (8 caractères)
  lcd.setCursor(0, 3);
  lcd.print(buffer);
}
void lcd_battery_level(uint level){
  lcd.setCursor(7, 3);
  if(level>100 || level<0) {
    lcd.print("---");
    return;
  }
  char update[4];
  sprintf(update,"%3d",level);
  lcd.print(update);
}
void lcd_speed(uint speed) {
  lcd.setCursor(13, 3);
  if (speed > 150 || speed < 0) {
    lcd.print("---");
    return;
  }
  char update[4];
  sprintf(update, "%3d", speed);
  lcd.print(update);
}
void lcd_gps_fix(bool fix) {
  lcd.setCursor(9, 1);
  lcd.print(fix ? "FIX" : "---");
}
void lcd_sd_status(int status) {
  lcd.setCursor(8, 0);
  if(status == LCD_NO_SD) lcd.print("NO SD");
  if(status == LCD_NO_INFO) lcd.print("---  ");
  if(status == LCD_SD_OK) lcd.print("OK   ");
}

void lcd_gps_dop(bool fix) {
  lcd.setCursor(9, 1);
  lcd.print(fix ? "FIX" : "---");
}
void lcd_sd_file(char filename[8]) {
  lcd.setCursor(8, 0);
  lcd.print(filename[0]);
  lcd.print(filename[1]);
  lcd.print(filename[2]);
  lcd.print(filename[3]);
  lcd.print(filename[4]);
  lcd.print(filename[5]);
  lcd.print(filename[6]);

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

void writeHeader(File& out) {
  out.println(
    "lat,long,height,day,month,year,hour,minute,second,millis,pdop,speed_kmh"
    ",ori_x,ori_y,ori_z"
    ",gyro_x,gyro_y,gyro_z"
    ",acc_x,acc_y,acc_z"
    ",mag_x,mag_y,mag_z"
    ",lin_x,lin_y,lin_z"
    ",grav_x,grav_y,grav_z"
    ",temp,cal_sys,cal_gyro,cal_accel,cal_mag"
    ",0_vesc_id,0_avgMotorCurrent,0_avgInputCurrent,0_dutyCycleNow,0_rpm,0_inpVoltage"
    ",0_ampHours,0_ampHoursCharged,0_wattHours,0_wattHoursCharged"
    ",0_tachometer,0_tachometerAbs,0_tempMosfet,0_tempMotor,0_pidPos,0_error"
    ",1_vesc_id,1_avgMotorCurrent,1_avgInputCurrent,1_dutyCycleNow,1_rpm,1_inpVoltage"
    ",1_ampHours,1_ampHoursCharged,1_wattHours,1_wattHoursCharged"
    ",1_tachometer,1_tachometerAbs,1_tempMosfet,1_tempMotor,1_pidPos,1_error"

  );
}

void setup() {
  //INIT DEBUG SERIAL
  DEBUGSerial.begin(115200);

  //INIT VESC SERIAL
  VESCSerial.begin(115200);
  VESC.setSerialPort(&VESCSerial);
  //INIT PINOUT
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  //INIT LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd_static();

  //INIT GPS SERIAL
  pinPeripheral(0, PIO_SERCOM);
  pinPeripheral(1, PIO_SERCOM);
  SerialGPS.begin(9600);
  delay(1000);
  DEBUGSerial.println("Initializing GPS...");
  GPS.begin(9600);
  delay(1000);
  GPS.sendCommand("$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
  delay(1000);
  GPS.sendCommand("$PMTK251,115200*1F");
  delay(1000);
  SerialGPS.begin(115200);
  GPS.begin(115200);
  delay(1000);
  GPS.sendCommand("$PMTK220,100*2F");

  //INIT IMU
  DEBUGSerial.println("Initializing IMU...");
  imu_status = bno.begin();
  if (!imu_status) {
    DEBUGSerial.println("BNO055 not detected.");
    beepBuzzer(BUZZ_DURATION_MS);
  }

  //INIT SD
  DEBUGSerial.println("Initializing SD card...");
  sd_status = SD.begin(CHIP_SELECT_SD);
  
  if (!sd_status) {
    lcd_sd_status(LCD_NO_SD);
    DEBUGSerial.println("NO SD!");
    beepBuzzer(BUZZ_DURATION_MS);
  }
  else
  {
    lcd_sd_status(LCD_SD_OK);
    DEBUGSerial.println("SD ON");
    listFiles();
    printCardInfo();
    // Générer un nom de fichier unique
    fileName = generateLogFileName();
    dataFile = SD.open(fileName.c_str(), FILE_WRITE);
    if (dataFile) {
      writeHeader(dataFile);
      fileOpened = true;
      DEBUGSerial.print("Fichier ouvert : ");
      DEBUGSerial.println(fileName);
      lcd_sd_file((char*)fileName.c_str());
    } else {
      DEBUGSerial.println("Erreur d'ouverture fichier.");
      fileOpened = false;
    }

  }
  
  
  //INIT LORA
  DEBUGSerial.println("Initializing LoRa...");
  LoRa_status = LoRa.begin(868E6);
  if (!LoRa_status) {
    DEBUGSerial.println("LoRa init failed. Check your hardware.");
    beepBuzzer(BUZZ_DURATION_MS);
  }
  else
  {
    DEBUGSerial.println("LoRa OK");
  }

}

void loop() {
  
 
  GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
    lcd_gps_fix(GPS.fix);

    if (GPS.fix) {
      lcd_speed((uint)round(GPS.speed * 1.852));

      uint16_t current_millis = GPS.milliseconds;
      if (current_millis == last_millis_shown) return;
      last_millis_shown = current_millis;


      float lat = convertNMEAToDecimal(GPS.latitude);
      if (GPS.lat == 'S') lat = -lat;
      float lon = convertNMEAToDecimal(GPS.longitude);
      if (GPS.lon == 'W') lon = -lon;

      int local_hour = (GPS.hour + timezone_offset) % 24;
      if (local_hour < 0) local_hour += 24;

      sensors_event_t orientation, gyro, accel, mag, linear, grav;
      bno.getEvent(&orientation, Adafruit_BNO055::VECTOR_EULER);
      bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
      bno.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
      bno.getEvent(&mag, Adafruit_BNO055::VECTOR_MAGNETOMETER);
      bno.getEvent(&linear, Adafruit_BNO055::VECTOR_LINEARACCEL);
      bno.getEvent(&grav, Adafruit_BNO055::VECTOR_GRAVITY);

      int8_t temp = bno.getTemp();
      uint8_t sys, gcal, acal, mcal;
      bno.getCalibration(&sys, &gcal, &acal, &mcal);

      if(sd_status){
     
        dataFile.print(lat, 6); dataFile.print(",");
        dataFile.print(lon, 6); dataFile.print(",");
        dataFile.print(GPS.altitude); dataFile.print(",");
        dataFile.print(GPS.day); dataFile.print(",");
        dataFile.print(GPS.month); dataFile.print(",");
        dataFile.print(2000 + GPS.year); dataFile.print(",");
        dataFile.print(local_hour); dataFile.print(",");
        dataFile.print(GPS.minute); dataFile.print(",");
        dataFile.print(GPS.seconds); dataFile.print(",");
        dataFile.print(current_millis); dataFile.print(",");
        dataFile.print(GPS.PDOP, 1); dataFile.print(",");
        dataFile.print(GPS.speed * 1.852, 2);

        auto printVec = [&](sensors_event_t e) {
          dataFile.print(","); dataFile.print(e.orientation.x);
          dataFile.print(","); dataFile.print(e.orientation.y);
          dataFile.print(","); dataFile.print(e.orientation.z);
        };

        printVec(orientation);
        printVec(gyro);
        printVec(accel);
        printVec(mag);
        printVec(linear);
        printVec(grav);

        dataFile.print(","); dataFile.print(temp);
        dataFile.print(","); dataFile.print(sys);
        dataFile.print(","); dataFile.print(gcal);
        dataFile.print(","); dataFile.print(acal);
        dataFile.print(","); dataFile.print(mcal);
        
        if(VESC.getVescValues(0))
        {
          lcd_current_g((int)VESC.data.avgMotorCurrent);
          lcd_temp_g((int)VESC.data.tempMosfet);

          dataFile.print(","); dataFile.print(VESC.data.id);
          dataFile.print(","); dataFile.print(VESC.data.avgMotorCurrent);
          dataFile.print(","); dataFile.print(VESC.data.avgInputCurrent);
          dataFile.print(","); dataFile.print(VESC.data.dutyCycleNow);
          dataFile.print(","); dataFile.print(VESC.data.rpm);
          dataFile.print(","); dataFile.print(VESC.data.inpVoltage);
          dataFile.print(","); dataFile.print(VESC.data.ampHours);
          dataFile.print(","); dataFile.print(VESC.data.ampHoursCharged);
          dataFile.print(","); dataFile.print(VESC.data.wattHours);
          dataFile.print(","); dataFile.print(VESC.data.wattHoursCharged);
          dataFile.print(","); dataFile.print(VESC.data.tachometer);
          dataFile.print(","); dataFile.print(VESC.data.tachometerAbs);
          dataFile.print(","); dataFile.print(VESC.data.tempMosfet);
          dataFile.print(","); dataFile.print(VESC.data.tempMotor);
          dataFile.print(","); dataFile.print(VESC.data.pidPos);
          dataFile.print(","); dataFile.print((int)VESC.data.error);

          
        }
      else
        {
          lcd_current_g(-1);
          lcd_temp_g(-1);
          dataFile.print(",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
        }
      if(VESC.getVescValues(1))
        {
          lcd_current_d((int)VESC.data.avgMotorCurrent);
          lcd_temp_d((int)VESC.data.tempMosfet);

          dataFile.print(","); dataFile.print(VESC.data.id);
          dataFile.print(","); dataFile.print(VESC.data.avgMotorCurrent);
          dataFile.print(","); dataFile.print(VESC.data.avgInputCurrent);
          dataFile.print(","); dataFile.print(VESC.data.dutyCycleNow);
          dataFile.print(","); dataFile.print(VESC.data.rpm);
          dataFile.print(","); dataFile.print(VESC.data.inpVoltage);
          dataFile.print(","); dataFile.print(VESC.data.ampHours);
          dataFile.print(","); dataFile.print(VESC.data.ampHoursCharged);
          dataFile.print(","); dataFile.print(VESC.data.wattHours);
          dataFile.print(","); dataFile.print(VESC.data.wattHoursCharged);
          dataFile.print(","); dataFile.print(VESC.data.tachometer);
          dataFile.print(","); dataFile.print(VESC.data.tachometerAbs);
          dataFile.print(","); dataFile.print(VESC.data.tempMosfet);
          dataFile.print(","); dataFile.print(VESC.data.tempMotor);
          dataFile.print(","); dataFile.print(VESC.data.pidPos);
          dataFile.print(","); dataFile.print((int)VESC.data.error);
        }
      else
        {
          lcd_current_d(-1);
          lcd_temp_d(-1);
          dataFile.print(",1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
        }

        dataFile.println();
        dataFile.flush();
      }

      //Send the LoRa message
      if (LoRa_status) {
        LoRa.beginPacket();

        // GPS
        LoRa.write((uint8_t*)&lat, sizeof(lat));
        LoRa.write((uint8_t*)&lon, sizeof(lon));
        LoRa.write((uint8_t*)&GPS.altitude, sizeof(GPS.altitude));
        LoRa.write((uint8_t*)&GPS.day, sizeof(GPS.day));
        LoRa.write((uint8_t*)&GPS.month, sizeof(GPS.month));

        uint16_t fullYear = 2000 + GPS.year;
        LoRa.write((uint8_t*)&fullYear, sizeof(fullYear));
        LoRa.write((uint8_t*)&local_hour, sizeof(local_hour));
        LoRa.write((uint8_t*)&GPS.minute, sizeof(GPS.minute));
        LoRa.write((uint8_t*)&GPS.seconds, sizeof(GPS.seconds));
        LoRa.write((uint8_t*)&current_millis, sizeof(current_millis));
        LoRa.write((uint8_t*)&GPS.PDOP, sizeof(GPS.PDOP));

        float speedKmh = GPS.speed * 1.852;
        LoRa.write((uint8_t*)&speedKmh, sizeof(speedKmh));

        // IMU : orientation, gyro, accel, mag, linear accel, gravity
        auto writeVec = [&](sensors_event_t e) {
          LoRa.write((uint8_t*)&e.orientation.x, sizeof(float));
          LoRa.write((uint8_t*)&e.orientation.y, sizeof(float));
          LoRa.write((uint8_t*)&e.orientation.z, sizeof(float));
        };

        writeVec(orientation);
        writeVec(gyro);
        writeVec(accel);
        writeVec(mag);
        writeVec(linear);
        writeVec(grav);

        // Température et calibration
        LoRa.write((uint8_t*)&temp, sizeof(temp));
        LoRa.write((uint8_t*)&sys, sizeof(sys));
        LoRa.write((uint8_t*)&gcal, sizeof(gcal));
        LoRa.write((uint8_t*)&acal, sizeof(acal));
        LoRa.write((uint8_t*)&mcal, sizeof(mcal));

        LoRa.endPacket();
      }

      

    } else {
      unsigned long now = millis();
      if (now - lastNoFixBeepTime >= NO_FIX_BEEP_INTERVAL_MS) {
        lastNoFixBeepTime = now;
        beepBuzzer(BUZZ_DURATION_MS);
      }
    }
  }
}

void SERCOM3_Handler() {
  SerialGPS.IrqHandler();
}
