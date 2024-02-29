#include <stdio.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "EEPROM.h"

//Definition des ports entre analogique de la carte 
#define GAZ A0
#define AN1 A1
#define AN2 A2
#define AN3 A3
#define AN4 A6
#define AN5 A7
//Defenition des entre numérique de la carte
#define SW1 2
#define SW2 3
#define SW3 4
//Defenition des sortie numériques de la carte
#define OUT1 6
#define OUT2 7
#define OUT3 8
//Definition des addresses de sauvegarde d'EEPROM
#define addr_eeprom_min_gaz 0
#define addr_eeprom_max_gaz 4

  //Definition des variables globales
  int raw_gaz=0;
  int map_gaz=0;
  int EEPROM_min_gaz=0;   //Address dans la mémoire eeprom de la poigné de min et max de la poigné de gaz
  int EEPROM_max_gaz=4;

//Declaration des prototypes
void calibration_gaz();
void init_pinout();
void init_lcd();
void static_lcd();
void update_Voltage(float vlot);
void update_TM(int TM);
void update_TE(int TE);
void update_Amp(int A);
void update_Mode(int Mode);
void update_KM(int KM);


LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
  Serial.begin(9600);   //Initialisation port serie
  init_pinout();   //Initialisation des pinout
  delay(100);
  init_lcd();
  init_lcd();
  static_lcd();
  calibration_gaz();

}

void loop() {
  raw_gaz = analogRead(GAZ);
  map_gaz=map(raw_gaz,EEPROM_min_gaz,EEPROM_max_gaz,0,255);
  if(map_gaz<0)
      {
        map_gaz=0;
      }
      if(map_gaz>255)
      {
        map_gaz=255;
      }
  Serial.print("sensor = ");
  Serial.println(map_gaz);
  //lcd.setCursor(2,1);
  //lcd.print(map_gaz);
  update_TM(10);
  update_TE(10);
  update_Amp(10);
  update_Mode(10);
  update_KM(10);
  update_Voltage(10.1);
  
  
}


void update_KM(int KM)
{
  lcd.setCursor(11,1);
  lcd.print("115");
}


void update_Mode(int Mode)
{
  lcd.setCursor(9,1);
  lcd.print("3");
}


void update_Amp(int A)
{
  lcd.setCursor(0,1);
  lcd.print("120");
}

void update_TE(int TE)
{
  lcd.setCursor(14,0);
  lcd.print("78");
}

void update_TM(int TM)
{
  lcd.setCursor(8,0);
  lcd.print("110");
}




void update_Voltage(float vlot)
{
  lcd.setCursor(0,0);
  lcd.print("50.1");
}

void static_lcd()
{
  lcd.setCursor(0,0);
  lcd.print("    v Tm    Te  ");
  lcd.setCursor(0,1);
  lcd.print("   A MOD:     km");
}

void init_lcd()
{
  lcd.init();
  lcd.backlight();
  lcd.setCursor(2,0);
  lcd.print("Volt'n Solex");
  delay(1000);
}

void init_pinout()
{
  //Initialisation des SW
  pinMode(SW1,INPUT_PULLUP);
  pinMode(SW2,INPUT_PULLUP);
  pinMode(SW3,INPUT_PULLUP);
  //Initialisation des OUT
  pinMode(OUT1,OUTPUT);
  pinMode(OUT2,OUTPUT);
  pinMode(OUT3,OUTPUT);
  
}

void calibration_gaz()//VALIDE
{
  if(!digitalRead(SW1)) //TEST si le bouton SW1 est pressé au démarrage dans ce cas rentre dans le mode de calibration
  {
      Serial.println("MODE: Calibration");
      Serial.println("Relachez SW1");
      while(!digitalRead(SW1));  //Attente de l'appuie sur le bouton SW1
      
      Serial.println("Calibration: ne pas toucher la poigne de gaz");
      Serial.println("Appuyer sur SW1 pour lancer la calibration");
      while(!digitalRead(SW1));  //Attente de l'appuie sur le bouton SW1
      raw_gaz = analogRead(GAZ);
      Serial.print("min:");
      Serial.println(raw_gaz);
      EEPROM_min_gaz=raw_gaz;
      EEPROM.put(addr_eeprom_min_gaz,raw_gaz);
      while(digitalRead(SW1));  //Attente du relachement sur le bouton SW1

      
      Serial.println("Tournez la poigné de gaz au maximum");
      Serial.println("Appuyer sur SW1 pour finaliser la calibration");
      while(!digitalRead(SW1));  //Attente de l'appuie sur le bouton SW1
      raw_gaz = analogRead(GAZ);
      Serial.print("max:");
      Serial.println(raw_gaz);
      EEPROM_max_gaz=raw_gaz;
      EEPROM.put(addr_eeprom_max_gaz,raw_gaz);
      while(!digitalRead(SW1));  //Attente du relachement sur le bouton SW1
      Serial.println("Calibration: calibration terminié");
      delay(5000);
  }
  else
  {
      EEPROM.get(addr_eeprom_max_gaz,EEPROM_max_gaz);
      EEPROM.get(addr_eeprom_min_gaz,EEPROM_min_gaz);
      
      Serial.print("Recovery of min and max gaz from eeprom min:");
      Serial.print(EEPROM_min_gaz);
      Serial.print(" max:");
      Serial.println(EEPROM_max_gaz);
      
  }
}
