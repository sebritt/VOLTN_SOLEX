#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); //I2C address 0x27, 16 column and 2 rows

byte temp_symbol[8] = {
  0b10000,
  0b01111,
  0b11000,
  0b10000,
  0b10000,
  0b11000,
  0b01111,
  0b00000
};

byte batt_symbol[8] = {
  0b01000,
  0b11100,
  0b10101,
  0b10100,
  0b10100,
  0b10101,
  0b10100,
  0b11100
};

byte speed_symbol[8] = {
  0b00000,
  0b00000,
  0b10100,
  0b01010,
  0b00101,
  0b01010,
  0b10100,
  0b00000,
};

byte esc_symbol[8] = {
  0b01110,
  0b11011,
  0b01010,
  0b11011,
  0b01010,
  0b11011,
  0b01110,
  0b00000,
};

byte kph_symbol[8] = {
  0b10100,
  0b11000,
  0b10101,
  0b10010,
  0b00100,
  0b01100,
  0b00111,
  0b00101,
};


void setup() {
  
  //DISPLAY SETUP
  lcd.init();
  lcd.backlight();

  //init all the specials caracters
  lcd.createChar(0, temp_symbol); // create a new custom character
  lcd.createChar(1, batt_symbol); // create a new custom character
  lcd.createChar(2, speed_symbol); // create a new custom character
  lcd.createChar(3, esc_symbol); // create a new custom character
  lcd.createChar(4, kph_symbol); // create a new custom character


  lcd.setCursor(1, 0);       
  lcd.print("M:");
  lcd.setCursor(5, 0);
  lcd.write((byte)0); 

  lcd.setCursor(7, 0);
  lcd.write((byte)1);
  lcd.setCursor(10, 0);
  lcd.print("V");

  lcd.setCursor(12, 0);
  lcd.write((byte)2);
 

  lcd.setCursor(5, 1);
  lcd.print("A");

  lcd.setCursor(10, 1);
  lcd.write((byte)0); 

  lcd.setCursor(12, 1);
  lcd.write((byte)3); 

  lcd.setCursor(15, 1);
  lcd.write((byte)0); 

  lcd.setCursor(15, 0);
  lcd.write((byte)4); 
  //texte pour test

  lcd.setCursor(3, 1);
  lcd.print("70");

  lcd.setCursor(8, 1);
  lcd.print("32");

  lcd.setCursor(8, 0);
  lcd.print("50");

  lcd.setCursor(13, 0);
  lcd.print("78");

  lcd.setCursor(13, 1);
  lcd.print("35");

  lcd.setCursor(3, 0);
  lcd.print("90");





}

void loop() {
  // put your main code here, to run repeatedly:
  
         
        
}