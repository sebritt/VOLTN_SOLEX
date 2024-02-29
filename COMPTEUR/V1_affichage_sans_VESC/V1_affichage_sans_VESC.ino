/**
 * \file V1 bare minimum for screen setup
 * \brief bare minimum to controle a LCD with i2c arduino to display essential values
 * \author Gaspard Carbon / Rittemard Sebastien
 * \version 0.1
 * \date 29 Feb 2024
 *
 * Programme de test pour l'objet de gestion des chaines de caractères Str_t.
 *
 */


// ######################################## DEFINES ########################################

//Define all the analog port on the PCB
#define GAZ A0
#define AN1 A1
#define AN2 A2
#define AN3 A3
#define AN4 A6
#define AN5 A7
//Define all the digial port for switchs on the PCB
#define SW1 2
#define SW2 3
#define SW3 4
//Define all the digial output port on the PCB
#define OUT1 6
#define OUT2 7
#define OUT3 8



// ######################################## Librarys ########################################

#include <LiquidCrystal_I2C.h>


// ################################ METHODES DECLARATION #####################################
int countDigits(int number); 
//########################################### LCD ############################################
LiquidCrystal_I2C lcd(0x27, 16, 2); //I2C address 0x27, 16 column and 2 rows
void LCD_INIT();
void LCD_staticPrint();
//SI BESOIN CHANGER LES INPUT AN STRING POUR AUGMENTER LA RAPIDITE DE CES FONCTIONS !!! donc chancher le LCD_printInt en LCD_printString
void LCD_printInt(short column, short ligne,int value,int min,int max)
void LCD_updateMotorTemp(int temp);
void LCD_updateMotorCurrent(int current);
void LCD_updateBatteryTemp(int temp);
void LCD_updateBatteryVoltage(int voltage);
void LCD_updateSpeed(int speed);
void LCD_updateVescTemp(int temp);
void LCD_test();


// #################################### GLOBAL VARIABLES ####################################

// ################################## LCD GLOBAL VARIABLES ##################################
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
// #########################################################################################

void setup() {
  LCD_INIT(); //Init the LCD module and setup the non changing char on the screen
  LCD_test();
}

void loop() {
  
         
        
}

/**
 * \fn void LCD_INIT()
 * \brief This function init all the LCD and call the function LCD_staticPrint() who display all the static char on the screen
 *
 * \param None
 * \return None
 */
void LCD_INIT()
{
  //DISPLAY SETUP
  lcd.init();
  lcd.backlight();

  //init all the specials caracters
  lcd.createChar(0, temp_symbol); // create a new custom character
  lcd.createChar(1, batt_symbol); // create a new custom character
  lcd.createChar(2, speed_symbol); // create a new custom character
  lcd.createChar(3, esc_symbol); // create a new custom character
  lcd.createChar(4, kph_symbol); // create a new custom character
}

/**
 * \fn void LCD_staticPrint()
 * \brief This funciton print on the screen the non changing caracter
 *
 * \param None
 * \return None
 */
void LCD_staticPrint()
{

  // Motor temp
  lcd.setCursor(1, 0);       
  lcd.print("M:");
  lcd.setCursor(5, 0);
  lcd.write((byte)0); 

  //Battery voltage
  lcd.setCursor(7, 0);
  lcd.write((byte)1);
  lcd.setCursor(10, 0);
  lcd.print("V");

   //Battery temp
  lcd.setCursor(10, 1);
  lcd.write((byte)0); 
 
  //Current consumtion
  lcd.setCursor(5, 1);
  lcd.print("A");
  
 
  //VESC Temp
  lcd.setCursor(12, 1);
  lcd.write((byte)3); 
  lcd.setCursor(15, 0);
  lcd.write((byte)4); 

  //Speed of the solex
  lcd.setCursor(12, 0);
  lcd.write((byte)2);

}

/**
 * \fn void LCD_INIT()
 * \brief For testing use only to display , to update all values possible by the display
 *
 * \param None
 * \return None
 */
void LCD_test()
{
  for (int i = 0; i <= 99; ++i) {
      LCD_updateMotorTemp(i);
      delay(1);
    }

  for (int i = -999; i <= 999; ++i) {
      LCD_updateMotorCurrent(i);
      delay(1);
    }

  for (int i = 0; i <= 99; ++i) {
      LCD_updateBatteryTemp(i);
      delay(1);
    }


  for (int i = 0; i <= 99; ++i) {
      LCD_updateBatteryVoltage(i);
      delay(1);
    }

  for (int i = 0; i <= 99; ++i) {
      LCD_updateSpeed(i);
      delay(1);
    }

  for (int i = 0; i <= 99; ++i) {
      LCD_updateVescTemp(i);
      delay(1);
    }


}


/**
 * \fn void LCD_updateMotorTemp(int temp)
 * \brief Update the motor temp on the display
 *
 * \param int temp: the temp of the motor btw 0 to 999 degrees
 * \return None
 */
void LCD_updateMotorTemp(int temp)
{
  LCD_printInt(5,0,temp, 0, 999);
}



/**
 * \fn void LCD_updateMotorCurrent(int current)
 * \brief Update the motor current on the display
 *
 * \param int current: the current of the motor btw -999 to 999 A
 * \return None
 */
void LCD_updateMotorCurrent(int current)
{
 LCD_printInt(5,1,current, -999, 999);

}


/**
 * \fn void LCD_updateBatteryTemp(int temp)
 * \brief Update the battery temp on the display
 *
 * \param int temp: the Current of the motor btw 0 to 99 C
 * \return None
 */
void LCD_updateBatteryTemp(int temp)
{
  LCD_printInt(10,1,temp, 0, 99);
}


/**
 * \fn void LCD_updateBatteryVoltage(int voltage)
 * \brief Update the battery voltage on the display
 *
 * \param int voltage: the voltage of the motor btw 0 to 99 V
 * \return None
 */
void LCD_updateBatteryVoltage(int voltage)
{
  LCD_printInt(10,0,voltage, 0, 99);
}


/**
 * \fn void LCD_updateSpeed(int speed)
 * \brief Update the speed of the solex on the display
 *
 * \param int speed: the speed of the solex btw 0 to 99 km/h
 * \return None
 */
void LCD_updateSpeed(int speed)
{
  LCD_printInt(15,0,speed, 0, 99);
}


/**
 * \fn void LCD_updateVescTemp(int temp)
 * \brief Update the temp of the vesc on the display
 *
 * \param int temp: the temp of the vesc btw 0 to 99 C
 * \return None
 */
void LCD_updateVescTemp(int temp)
{
  LCD_printInt(15,1,temp, 0, 99);
}


/**
 * \fn void LCD_printInt(short column, short ligne,int value,int min,int max)
 * \brief Update the LCD at the column and line specified, and manage shifting to display at the right place.
          
 *
 * \param short column => the colum from the start, short ligne => the lign from the start,int value => the value from to print ,int min,int max => to check min and max
 * \return None
 */
void LCD_printInt(short column, short ligne,int value,int min,int max)
{

  if((value>=min) && (value<=max))  //Test if the value is in the correct range if not print X
  {
    if(min <0) //display numbre that can be negative
    {
      lcd.setCursor(column-countDigits(value)-1, ligne);
      lcd.print(String(value));
    }
    else  //diplay positive number
    {
      lcd.setCursor(column-countDigits(value), ligne);
      lcd.print(String(value));
    }
    
  }
  else
  {
      lcd.setCursor(column-countDigits(value), ligne);
      lcd.print("X");
  }


}


int countDigits(int number) {
    int count = 0;
    if (number == 0) {
        return 1; // Edge case for 0
    }
    while (number != 0) {
        number /= 10; // Divide the number by 10, chopping off the last digit
        count++;      // Increment the digit count
    }
    return count;
}