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


// ######################################## Librarys ########################################

#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <VescUart.h>
#include <buffer.h>
#include <crc.h>
#include <datatypes.h>



//Define motor settings
#define NB_POLE 7
#define WHEEL_DIAMETER 0.43 //17 pouces
#define REDUCTION 16/185

// ################################ METHODES DECLARATION #####################################
int countDigits(int number); 

//########################################### LCD ############################################
LiquidCrystal_I2C lcd(0x27, 16, 2); //I2C address 0x27, 16 column and 2 rows
void LCD_INIT();
void LCD_staticPrint();
//SI BESOIN CHANGER LES INPUT AN STRING POUR AUGMENTER LA RAPIDITE DE CES FONCTIONS !!! donc chancher le LCD_printInt en LCD_printString
void LCD_printInt(short column, short ligne,int value,int min,int max);
void LCD_updateMotorTemp(float temp);
void LCD_updateMotorCurrent(float current);
void LCD_updateBatteryTemp(float temp);
void LCD_updateBatteryVoltage(float voltage);
void LCD_updateSpeed(float speed);
void LCD_updateVescTemp(float temp);
void LCD_test();

//########################################### VESC ############################################
/** Initiate VescUart class */
VescUart UART;

/** Initiate SoftwareSerial class */
SoftwareSerial vescSerial(10, 9);

float VESC_getMotorCurrent();
float VESC_getMotorTemp();
float VESC_getBatteryVoltage();
float VESC_getSpeed();
float VESC_VescTemp();

void VESC_test();
void getValues();




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
  0b01110,
  0b11111,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111
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

// ################################## VESC GLOBAL VARIABLES ##################################


struct Data_struct
{
  int speed           = 0;
  int batteryVoltage  = 0;
  int batteryTemp     = 0;
  int motorCurrent    = 0;
  int motorTemp       = 0; 
  int vescTemp        = 0;
};

Data_struct Data;
//#########################################################################################



void setup() {
Serial.begin(115200);


//Init the LCD module and setup the non changing char on the screen
  LCD_INIT(); 
  LCD_staticPrint();



//VESC init
  vescSerial.begin(115200);

  /** Define which ports to use as UART */
  UART.setSerialPort(&vescSerial);
  
  
  //Display values from ESC
  VESC_test();





}


void loop() {
  
         /** Call the function getVescValues() to acquire data from VESC */
  if ( UART.getVescValues() ) {

  LCD_updateMotorTemp(VESC_getMotorTemp());
  LCD_updateMotorCurrent(VESC_getMotorCurrent());
  LCD_updateBatteryTemp(0);
  LCD_updateBatteryVoltage(VESC_getBatteryVoltage());
  LCD_updateSpeed(VESC_getSpeed());
  LCD_updateVescTemp(VESC_VescTemp());
  LCD_staticPrint();

  }
  else
  {
    Serial.println("Failed to get data!");
  }

  delay(50); 
        
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
 * \param float temp: the temp of the motor btw 0 to 999 degrees
 * \return None
 */
void LCD_updateMotorTemp(float temp)
{
  int temp_int=int(temp);
  LCD_printInt(5,0,temp_int, 0, 999);
}



/**
 * \fn void LCD_updateMotorCurrent(int current)
 * \brief Update the motor current on the display
 *
 * \param float current: the current of the motor btw -999 to 999 A
 * \return None
 */
void LCD_updateMotorCurrent(float current)
{
  int current_int= int(current);
  LCD_printInt(5,1,current, 0, 999);
 
}


/**
 * \fn void LCD_updateBatteryTemp(int temp)
 * \brief Update the battery temp on the display
 *
 * \param float temp: the Current of the motor btw 0 to 99 C
 * \return None
 */
void LCD_updateBatteryTemp(float temp)
{
  int temp_int=int(temp);
  LCD_printInt(10,1,temp_int, 0, 99);
}


/**
 * \fn void LCD_updateBatteryVoltage(int voltage)
 * \brief Update the battery voltage on the display
 *
 * \param float voltage: the voltage of the motor btw 0 to 99 V
 * \return None
 */
void LCD_updateBatteryVoltage(float voltage)
{
  int voltage_int=int(voltage);
  LCD_printInt(10,0,voltage_int, 0, 99);
}


/**
 * \fn void LCD_updateSpeed(int speed)
 * \brief Update the speed of the solex on the display
 *
 * \param float speed: the speed of the solex btw 0 to 99 km/h
 * \return None
 */
void LCD_updateSpeed(float speed)
{
  int speed_int=int(speed);
  LCD_printInt(15,0,speed_int, 0, 99);
}


/**
 * \fn void LCD_updateVescTemp(int temp)
 * \brief Update the temp of the vesc on the display
 *
 * \param float temp: the temp of the vesc btw 0 to 99 C
 * \return None
 */
void LCD_updateVescTemp(float temp)
{
  int temp_int=int(temp);
  LCD_printInt(15,1,temp_int, 0, 99);
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
    //Affichage des caractères
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

//######################################################################################


/**
 * \fn float VESC_getMotorCurrent()
 * \brief Return the average motor current from the vesc
 *
 * \param None
 * \return average motor current
 */
float VESC_getMotorCurrent(){
  return UART.data.avgMotorCurrent;
}

/**
 * \fnfloat VESC_getMotorTemp()
 * \brief Return temp of the motor
 *
 * \param None
 * \return motor temp
 */
float VESC_getMotorTemp(){
  return UART.data.tempMotor;
}

/**
 * float VESC_getBatteryVoltage()
 * \brief Return Battery voltage
 * \param None
 * \return Battery voltage
 */
float VESC_getBatteryVoltage(){
  return UART.data.inpVoltage;
}

/**
 * float VESC_getSpeed()
 * \brief Return speed computed from RPM
 * \param None
 * \return speed in kmph
 */
float VESC_getSpeed(){
  return UART.data.rpm*((2*3.14)/60)*WHEEL_DIAMETER*REDUCTION; // v(km/h)=Rayon(m)*(2PI/60)*RPM(tr/min)*Reduction*3,6(conv en km/h)
}

/**
 * float VESC_VescTemp()
 * \brief Return Mosfet temperature
 * \param None
 * \return Vesc temp
 */
float VESC_VescTemp(){
  return UART.data.tempMosfet;
}


/**
 * \fn void getValues()
 * \brief Store values in Data_struct
 *
 * \param None
 * \return None
 */
void getValues(){
  Data.motorTemp = int(VESC_getMotorTemp());
  Data.motorCurrent = int(VESC_getMotorCurrent());
 // LCD_updateBatteryTemp(float temp);
  Data.batteryVoltage = int(VESC_getBatteryVoltage());
  Data.speed = int(VESC_getSpeed());
  Data.vescTemp = int(VESC_VescTemp());
}

/**
 * \fn void VESC_test()
 * \brief For testing vesc value on I2C display
 *
 * \param None
 * \return None
 */
void VESC_test(){
  LCD_updateMotorTemp(VESC_getMotorTemp());
  LCD_updateMotorCurrent(VESC_getMotorCurrent());
 // LCD_updateBatteryTemp(float temp);
  LCD_updateBatteryVoltage(VESC_getBatteryVoltage());
  LCD_updateSpeed(VESC_getSpeed());
  LCD_updateVescTemp(VESC_VescTemp());
}











