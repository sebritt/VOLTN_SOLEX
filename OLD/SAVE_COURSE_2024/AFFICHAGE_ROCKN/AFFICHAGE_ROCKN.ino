#include <LiquidCrystal_I2C.h>
#include <VescUart.h>
#include <SoftwareSerial.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
VescUart vesc;
SoftwareSerial vescSerial(10, 11); //RX TX => attention a bien inverser les fils


#define WHEEL_DIAMETER 0.43 //17 pouces
#define REDUCTION 36/38


//MENU 1 ---------------------------
#define MOTOR_TEMP_X 2
#define MOTOR_TEMP_Y 0
#define MOTOR_TEMP_W 3

#define MOTOR_AMP_X 7
#define MOTOR_AMP_Y 0
#define MOTOR_AMP_W 4

#define BATERY_TEMP_X 2
#define BATERY_TEMP_Y 1
#define BATERY_TEMP_W 3

#define BATERY_VOLT_X 9
#define BATERY_VOLT_Y 1
#define BATERY_VOLT_W 2

#define ESC_TEMP_X 2
#define ESC_TEMP_Y 2
#define ESC_TEMP_W 3

#define GAZ_X 17
#define GAZ_Y 0
#define GAZ_W 3

#define FRG_X 17 
#define FRG_Y 1
#define FRG_W 3

#define FRD_X 17
#define FRD_Y 2
#define FRD_W 3


#define SPD_X 17
#define SPD_Y 3
#define SPD_W 3

#define F1_X 3
#define F1_Y 3
#define F1_W 3


#define F2_X 9
#define F2_Y 3
#define F2_W 3


//GLOBAL VARIABLES

struct MENU{
  char L0[21];
  char L1[21];
  char L2[21];
  char L3[21];
};
struct MENU menu[2];  

char buffer[10];


//Prototypes
void init_menu();
void print_menu(int numero_menu);
void print_int(int number, unsigned short x, unsigned short y, unsigned short width);




void setup() {
    vescSerial.begin(115200);
      vesc.setSerialPort(&vescSerial);
    Serial.begin(115200);


  // put your setup code here, to run once:
  lcd.init();
  lcd.clear();                      // initialize the lcd 
  init_menu();
  // Print a message to the LCD.
  lcd.backlight();
  print_menu(1);
  delay(1000);
  print_menu(0);
  /*
  print_int(99,MOTOR_TEMP_X,MOTOR_TEMP_Y,MOTOR_TEMP_W);
  print_int(99,MOTOR_AMP_X,MOTOR_AMP_Y,MOTOR_AMP_W);

  print_int(99,BATERY_TEMP_X,BATERY_TEMP_Y,BATERY_TEMP_W);
  print_int(99,BATERY_VOLT_X,BATERY_VOLT_Y,BATERY_VOLT_W);

  print_int(99,ESC_TEMP_X,ESC_TEMP_Y,ESC_TEMP_W);

  print_int(99,GAZ_X,GAZ_Y,GAZ_W);
  print_int(99,SPD_X,SPD_Y,SPD_W);

  print_int(99,F1_X,F1_Y,F1_W);
  print_int(99,F2_X,F2_Y,F2_W);
  */
  


}

void loop() {
  // put your main code here, to run repeatedly:
  if ( vesc.getVescValues() ) {
  print_int(int(vesc.data.tempMotor),MOTOR_TEMP_X,MOTOR_TEMP_Y,MOTOR_TEMP_W);
  print_int(int(vesc.data.avgMotorCurrent),MOTOR_AMP_X,MOTOR_AMP_Y,MOTOR_AMP_W);

  print_int(0,BATERY_TEMP_X,BATERY_TEMP_Y,BATERY_TEMP_W);
  print_int(int(vesc.data.inpVoltage),BATERY_VOLT_X,BATERY_VOLT_Y,BATERY_VOLT_W);

  print_int(int(vesc.data.tempMosfet),ESC_TEMP_X,ESC_TEMP_Y,ESC_TEMP_W);

  print_int(0,GAZ_X,GAZ_Y,GAZ_W);
  print_int(int(vesc.data.rpm)*((2*3.14)/60)*WHEEL_DIAMETER*REDUCTION,SPD_X,SPD_Y,SPD_W);

  print_int(0,F1_X,F1_Y,F1_W);
  print_int(0,F2_X,F2_Y,F2_W);
  print_int(0,FRG_X,FRG_Y,FRG_W);
  print_int(0,FRD_X,FRD_Y,FRD_W);
  //FOR DEBUG ON SERIAL PORT
  /*
   Serial.println(vesc.data.rpm);
    Serial.println(vesc.data.inpVoltage);
    Serial.println(vesc.data.ampHours);
    Serial.println(vesc.data.tachometerAbs);
  */
  }
}
w
void print_int(int number, unsigned short x, unsigned short y, unsigned short width)
{
    switch (width) {
    case 0:
      sprintf(buffer, "%d", number);
      break;
    case 1:
      sprintf(buffer, "%1d", number);
      break;
    case 2:
      sprintf(buffer, "%2d", number);
      break;
    case 3:
      sprintf(buffer, "%3d", number);
      break;
    case 4:
      sprintf(buffer, "%4d", number);
      break;
    case 5:
      sprintf(buffer, "%5d", number);
      break;
    default:
      // Gérer le cas où la largeur n'est pas valide
      sprintf(buffer, "%d", number); // Utilisation par défaut
      break;
  }
  if(strlen(buffer)>width)
  {
    lcd.setCursor(x, y);
    for(int i=0;i!=width;i++)
    {
      lcd.print("-");
    }
  }
  else
  {
    lcd.setCursor(x, y);
    lcd.print(buffer);
  }
  

}


void print_menu(int numero_menu)
{
  lcd.setCursor(0,0);
  lcd.print(menu[numero_menu].L0);
  lcd.setCursor(0,1);
  lcd.print(menu[numero_menu].L1);
  lcd.setCursor(0,2);
  lcd.print(menu[numero_menu].L2);
  lcd.setCursor(0,3);
  lcd.print(menu[numero_menu].L3);
}

void init_menu()
{
  //Menu 0 init
  strcpy(menu[0].L0, "M:   C     A GAZ:   ");
  strcpy(menu[0].L1, "B:   C     V FRG:   ");
  strcpy(menu[0].L2, "E:   C       FRD:   ");
  strcpy(menu[0].L3, "F1:   F2:    SPD:   ");

  //Menu 1 init
  strcpy(menu[1].L0, "SW1:  A1:    A4:    ");
  strcpy(menu[1].L1, "SW2:  A2:    A5:    ");
  strcpy(menu[1].L2, "SW3:  A3:    A6:    ");
  strcpy(menu[1].L3, "O1:  O2:  O3:  O3:  ");
}
