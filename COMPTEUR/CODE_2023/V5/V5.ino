#include <VescUart.h>


#include <Servo.h>

#include "EEPROM.h"


//Library for the Display
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C lcd(0x27,16, 2); 
VescUart vesc;
Servo VESC;


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


#define MAX_VESC_SAVE 1985
#define MIN_VESC_SAVE 990

/*
 * 
 * float avgMotorCurrent;
        float avgInputCurrent;
        float dutyCycleNow;
        float rpm;
        float inpVoltage;
        float ampHours;
        float ampHoursCharged;
        float wattHours;
        float wattHoursCharged;
        long tachometer;
        long tachometerAbs;
        float tempMosfet;
        float tempMotor;
        float pidPos;
        uint8_t id;
        mc_fault_code error; 
 };
 */




  //Definition des variables globales
  int raw_gaz=0;
  int map_gaz=0;
  int EEPROM_min_gaz=0;   //Address dans la mémoire eeprom de la poigné de min et max de la poigné de gaz
  int EEPROM_max_gaz=4;
  int min_vesc=MIN_VESC_SAVE;
  int max_vesc=MAX_VESC_SAVE;
  short MODE=100;
  short HS=1;





//Declaration des prototypes
void calibration_gaz();
void init_pinout();
void init_lcd();
void static_lcd();
void update_Voltage(float volt);
void update_TM(int TM);
void update_TE(int TE);
void update_Amp(int A);
void update_Mode(int Mode);
void update_KM(int KM);
void read_gaz();
void write_gaz();


void setup() {
  init_pinout();      //Initialisation des pinout
  init_lcd();         //Initialisation du lcd
  calibration_gaz();  //Effectue la calibration
  static_lcd();       //Affiche les caractère qui ne changerons pas
  
  Serial.begin(19200); 
  vesc.setSerialPort(&Serial);


  update_Mode(MODE);

}

void loop() {
  
  read_gaz();
  write_gaz();
  mode_read();
  
  if ( vesc.getVescValues() ) {
    
    lcd.setCursor(0,1);
    update_Amp((int)vesc.data.avgMotorCurrent);
    update_Voltage(vesc.data.inpVoltage);
    update_KM(((int)vesc.data.rpm)/14);
    update_TE((int)vesc.data.tempMosfet);
  }
  
}


void mode_read()
{
  if(!digitalRead(SW1))
  {
    while(!digitalRead(SW1));
    MODE=MODE-25;
    if(MODE<25)
    {
      MODE=100;
    }
    if(!HS)
    {
      MODE=100;
    }
    HS=1;
    max_vesc=map(MODE,0,100,MIN_VESC_SAVE,MAX_VESC_SAVE);
    update_Mode(MODE);
  }
}

void write_gaz()
{
  if(digitalRead(SW3) && HS)
  {
  VESC.writeMicroseconds(map_gaz);
  }
  else
  {
      VESC.writeMicroseconds(1);
      lcd.setCursor(7,1); 
      lcd.print("HS ");
      HS=0;
  }
}
void read_gaz()
{
  raw_gaz = analogRead(GAZ);
  map_gaz=map(raw_gaz,EEPROM_min_gaz,EEPROM_max_gaz,min_vesc,max_vesc);
  if(map_gaz<MIN_VESC_SAVE)
      {
        map_gaz=MIN_VESC_SAVE;
      }
      if(map_gaz>MAX_VESC_SAVE)
      {
        map_gaz=MAX_VESC_SAVE;
      }
}

void update_KM(int KM)
{
  lcd.setCursor(11,1);
  lcd.print(KM);
  lcd.print("KM");
  if(KM<100)
  {
    lcd.print(" ");
  }
  if(KM<10)
  {
    lcd.print(" ");    
  }
  
}


void update_Mode(int Mode)
{
  lcd.setCursor(7,1); 
  lcd.print(Mode);
  if(Mode<100)
  {
    lcd.print(" ");
  }
  if(Mode<10)
  {
    lcd.print(" ");    
  }
}


void update_Amp(int A)
{
  lcd.setCursor(0,1);
  lcd.print(A);
  lcd.print("A");
    if(A<10)
    {
      lcd.print(" ");    
    }
}

void update_TE(int TE)
{
  lcd.setCursor(11,0);
  lcd.print("TE:");
  lcd.print(TE);
  if(TE<100)
  {
    lcd.print(" ");
  }
  if(TE<10)
  {
    lcd.print(" ");    
  }
}

void update_TM(int TM)
{
  lcd.setCursor(10,0);
  lcd.print("TM:");
  lcd.print(TM);
  if(TM<100)
  {
    lcd.print(" ");
  }
  if(TM<10)
  {
    lcd.print(" ");    
  }
}




void update_Voltage(float volt)
{
  lcd.setCursor(0,0);
  lcd.print(volt,1);
}

void static_lcd()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("    v");
  lcd.setCursor(0,1);
  lcd.print("     M:      ");
  
}

void init_lcd()
{
  lcd.init();
  lcd.backlight();
  lcd.clear();
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
  //Pin du VESC pour la control
  VESC.attach(5);
  VESC.writeMicroseconds(1500);


  
}

void calibration_gaz()//VALIDE
{
  if(!digitalRead(SW1)) //TEST si le bouton SW1 est pressé au démarrage dans ce cas rentre dans le mode de calibration
  {
      lcd.clear();
      lcd.setCursor(1,0);
      lcd.print("Calibration gaz");
      lcd.setCursor(1,1);
      lcd.print("Relachez SW1");
      while(!digitalRead(SW1));  //Attente de l'appuie sur le bouton SW1

      lcd.clear();
      lcd.setCursor(1,0);
      lcd.print("GAZ MIN !");
      lcd.setCursor(1,1);
      lcd.print("Apppuyer SW1");
      while(!digitalRead(SW1));  //Attente de l'appuie sur le bouton SW1
      raw_gaz = analogRead(GAZ);
      EEPROM_min_gaz=raw_gaz;
      EEPROM.put(addr_eeprom_min_gaz,raw_gaz);
      
      lcd.clear();
      lcd.setCursor(1,0);
      lcd.print("min gaz:");
      lcd.print(EEPROM_min_gaz);
      lcd.setCursor(1,1);
      lcd.print("Relachez SW1");
      while(digitalRead(SW1));  //Attente du relachement sur le bouton SW1


      
      lcd.clear();
      lcd.setCursor(1,0);
      lcd.print("GAZ MAX !");
      lcd.setCursor(1,1);
      lcd.print("Appuyez SW1");
      
      while(!digitalRead(SW1));  //Attente de l'appuie sur le bouton SW1
      raw_gaz = analogRead(GAZ);
      EEPROM_max_gaz=raw_gaz;
      EEPROM.put(addr_eeprom_max_gaz,raw_gaz);
      
      lcd.clear();
      lcd.setCursor(1,0);
      lcd.print("min gaz:");
      lcd.print(EEPROM_min_gaz);
      lcd.setCursor(1,1);
      lcd.print("max gaz:");
      lcd.print(EEPROM_max_gaz);

      while(!digitalRead(SW1));  //Attente du relachement sur le bouton SW1
      delay(2000);
      lcd.clear();
      lcd.setCursor(1,0);
      lcd.print("CALIBRATION OK");
  }
  else
  {
      EEPROM.get(addr_eeprom_max_gaz,EEPROM_max_gaz);
      EEPROM.get(addr_eeprom_min_gaz,EEPROM_min_gaz);
      

      lcd.clear();
      lcd.setCursor(1,0);
      lcd.print("min gaz:");
      lcd.print(EEPROM_min_gaz);
      lcd.setCursor(1,1);
      lcd.print("max gaz:");
      lcd.print(EEPROM_max_gaz);
      delay(1000);
      
  }
}
