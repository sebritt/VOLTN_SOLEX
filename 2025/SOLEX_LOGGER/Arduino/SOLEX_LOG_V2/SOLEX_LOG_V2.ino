#include <Arduino.h>
#include "wiring_private.h"   // pour pinPeripheral
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <string.h>

//Pin definition
#define PIN_BUZZER 2
#define PIN_GPS_TX 0
#define PIN_GPS_RX 1
#define PIN_VESC_TX 14
#define PIN_VESC_RX 13

//Function declaration
void beep(int num,unsigned long time);
void lcd_static();
void lcd_current_g(uint current);
void lcd_current_d(uint current);
void lcd_temp_g(uint temp);
void lcd_temp_d(uint temp);
void lcd_battery_voltage(float voltage);
void lcd_battery_level(uint level);
void lcd_speed(uint speed);
void lcd_gps_fix(bool fix);
void lcd_sd_status(bool status);
//Class declaration
LiquidCrystal_I2C lcd(0x27,20,4); 


void setup() {
  //debug serial
  Serial.begin(9600);
  //buzzer setup
  pinMode(PIN_BUZZER,OUTPUT);
  beep(2,100);
  //lcd setup
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd_static();
}

void loop() {

  
}


void beep(int num,unsigned long time){
  for(int i=0; i<num;i++){
    digitalWrite(PIN_BUZZER,HIGH);
    delay(time);
    digitalWrite(PIN_BUZZER,LOW);
    delay(time);
  }
}

void lcd_static(){
  lcd.setCursor(0,0);
  lcd.print("   C    SD:        C");
  lcd.setCursor(0,1);
  lcd.print("   A   GPS:        A");
  lcd.setCursor(0,3);
  lcd.print("    V     %     km/h");
  lcd_current_g(-1);
  lcd_current_d(-1);
  lcd_temp_g(-1);
  lcd_temp_d(-1);
  lcd_battery_voltage(-1);
  lcd_battery_level(-1);
  lcd_speed(-1);
  lcd_gps_fix(false);
  lcd_sd_status(false);
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
void lcd_speed(uint speed){
  lcd.setCursor(13, 3);
  if(speed>150 || speed<0) {
    lcd.print("---");
    return;
  }
  char update[4];
  sprintf(update,"%3d",speed);
  lcd.print(update);
}

void lcd_gps_fix(bool fix){
  lcd.setCursor(11,1);
  if(fix){
    
    lcd.print("FIX");
  }
  else
  {
    lcd.print("---");
  }
}
void lcd_sd_status(bool status){
  lcd.setCursor(11,0);
  if(status){
    
    lcd.print("OK ");
  }
  else
  {
    lcd.print("---");
  }
}
