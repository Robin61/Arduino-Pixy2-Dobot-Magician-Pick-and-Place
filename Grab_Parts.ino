#include <Pixy2.h>
#include <EEPROM.h>  //for reading and writing data to and from EEPROM

//Define constants for input ports used in this project
const int buttonUpPin = 22;
const int buttonDownPin = 23;
const int buttonLeftPin = 24;
const int buttonRightPin = 25;
const int button1Pin = 26;

const int signaturePart1 = 1;  //signature number chosen inPixy software for 1st color (part) signature
const int signaturePart2 = 2;  //signature number chosen inPixy software for 1st color (part) signature
const int signaturePart3 = 3;  //signature number chosen inPixy software for 1st color (part) signature
const int signatureCalib = 7;  //signature number chosen in Pixy software for the calibration dots

//Variables to hold current button state
int upButtonState = 0;
int downButtonState = 0;
int leftButtonState = 0;
int rightButtonState = 0;
int button1State = 0;
//Variables to hold Previous button state (to see if button was already pushed when prevously going through the loop
int prevUpButtonState = 0;
int prevDownButtonState = 0;
int prevLeftButtonState = 0;
int prevRightButtonState = 0;
int prevButton1State = 0;

// variables for coordinate transformation from calibration sheet to dobot coordinates
float a1X = 0;       float b1X =0;      float c1X = 0; //X calibration dots on A4 sheet in Pixy coordinate system
float a1Y = 0;       float b1Y =0;      float c1Y = 0; //Y calibration dots on A4 sheet in Pixy coordinate system
float a2X = 0;       float b2X =0;      float c2X = 0; //X calibration dots in Dobot coordinate system
float a2Y = 0;       float b2Y =0;      float c2Y = 0; //Y calibration dots in Dobot coordinate system
float scaleX = 0;    float scaleY = 0;
float r1 = 0;        float r2 =0;  // length of a-b line segment (used as radius in calculation of angle
float tempFloatVar = 0; // variable used for temporarily storing value (used for swapping values of variables)
float tempFloatVarX = 0; float tempFloatVarY = 0; int tempIntVarSignature = 0;// variables used for temporarily storing value (used for swapping values of variables)

//other variables
float startPosX = 200.00;       float startPosY = 0.00;       float startPosZ = 0.00;       float startPosR = 0.00;       //Start position when powering up
float calXpos = 200.00;         float calYpos = 0.00;         float calZpos = 100.00;       float calRpos = 0.00;         //Arm position for reading calibration sheet
float dropLocation1X = 200.00;  float dropLocation1Y = 0.00;  float dropLocation1Z = 0.00;  float dropLocation1R = 0.00;  //Drop location for parts with signature 1
float dropLocation2X = 200.00;  float dropLocation2Y = 0.00;  float dropLocation2Z = 0.00;  float dropLocation2R = 0.00;  //Drop location for parts with signature 2
float dropLocation3X = 200.00;  float dropLocation3Y = 0.00;  float dropLocation3Z = 0.00;  float dropLocation3R = 0.00;  //Drop location for parts with signature 3
float currentX = startPosX;     float currentY = startPosY;   float currentZ = startPosZ;   float currentR = startPosR;   bool currentVac = false;
bool endeffectorGripper = false; // indicate type of end effector: true for gripper, false for vacuum cup

float zDownPos = 0;
float pixyPartSignature = 0; float pixyPartX = 0; float pixyPartY = 0; //part found by pixy camera
int signature1Found = 0;  // counter for how many Signature 1 dots were found
int signature2Found = 0;  // counter for how many Signature 2 dots were found
int signature3Found = 0;  // counter for how many Signature 3 dots were found
int activeSignature =0;   // signature number currently being used for pick and place
float partWidth = 0; float partHeight = 0; //dimensions of part found by pixy camera
float dobotPartX =0; float dobotPartY = 0;
float angle1 = 0;         // angle1: angle of rotation calibration sheet to pixy coordinate system
float angle2 = 0;         // angle2: angle of rotation calibration sheet to dobot coordinate system
float intermediateX = 0; float intermediateY = 0; // intermediate variable used in calculations

float moveInterval = 1.0; //step increment in mm used for jogging
int activeAxis = 1;       //active axis for jogging or other movement commands

bool drawMenu = true;     // variable to indicate is menu needs to be drawn the first time it is entered
int menuChoice = 1;       //variable for going through and selecting items in menus

int eePromIntVar = 0;
float eePromFloatVar =0;

//EEPROM address usage for this program: (used space of 4 bytes for each variable since floats use 4 bytes. Also used 4 bytes for integers for consistency in numbering, although this is not necessary for integers.
int eeAddressStartPosAvailable = 0; //Startpos avaliable? (set value of this address to 999 if the start position has been written previously to EEPROM
float eeAddressStartPosX = 4;
float eeAddressStartPosY = 8;
float eeAddressStartPosZ = 12;
float eeAddressStartPosR = 16;
int eeAddressCalPosAvailable = 20; //calibration Position available? (set value of this address to 999 when calibartion position has been written previously to EEPROM)
float eeAddressCalXpos = 24;
float eeAddressCalYpos = 28;
float eeAddressCalZpos = 32;
float eeAddressCalRpos = 36;
int eeAddressA1Available = 40; //A1 calibration coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressA1x = 44;
float eeAddressA1y = 48;
int eeAddressB1Available = 52; //B1 calibration coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressB1x = 56;
float eeAddressB1y = 60;
int eeAddressC1Available = 64; //C1 calibration coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressC1x = 68;
float eeAddressC1y = 72;
int eeAddressA2Available = 76; //A2 calibration coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressA2x = 80;
float eeAddressA2y = 84;
int eeAddressB2Available = 88; //B2 calibration coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressB2x = 92;
float eeAddressB2y = 96;
int eeAddressC2Available = 100; //C2 calibration coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressC2x = 104;
float eeAddressC2y = 108;
int eeAddressZdownAvailable = 112; //Z-down position (Z pos for grabbing parts) available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressZdown = 116;
int eeAddressDropLocation1Available = 120; //Drop location 1 coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressDropLocation1X = 124;
float eeAddressDropLocation1Y = 128;
float eeAddressDropLocation1Z = 132;
float eeAddressDropLocation1R = 136;
int eeAddressDropLocation2Available = 140; //Drop location 2 coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressDropLocation2X = 144;
float eeAddressDropLocation2Y = 148;
float eeAddressDropLocation2Z = 152;
float eeAddressDropLocation2R = 156;
int eeAddressDropLocation3Available = 160; //Drop location 3 coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressDropLocation3X = 164;
float eeAddressDropLocation3Y = 168;
float eeAddressDropLocation3Z = 172;
float eeAddressDropLocation3R = 176;
int eeAddressEndefectorAvailable = 180; //Drop location 3 coordinates available? (set value of this address to 999 when calibartion coordinate has been written previously to EEPROM)
float eeAddressEndeffector = 184;

// Library and variables for LCD shield ****************************************
#include <LiquidCrystal.h>
/*******************************************************
This program will test the LCD panel and the buttons
********************************************************/
// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// define some values used by the panel and buttons
int lcd_key = 0;
int adc_key_in = 0;
int testCounter =0;
int prevKeyState = 0;
int menuItem = 0;
#define btnRIGHT 0
#define btnUP 1
#define btnDOWN 2
#define btnLEFT 3
#define btnSELECT 4
#define btnNONE 5


// **** LCD  shield read buttons ***********************
// read the buttons
int read_LCD_buttons()
{
adc_key_in = analogRead(0); // read the value from the sensor
// my buttons when read are centered at these valies: 0, 144, 329, 504, 741
// we add approx 50 to those values and check to see if we are close
if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result

if (adc_key_in < 50) return btnRIGHT; 
// VMA203
// V. 02 Ã¢â‚¬â€œ 16/01/2019 5 Ã‚Â©Velleman nv
if (adc_key_in < 195) return btnUP;
if (adc_key_in < 380) return btnDOWN;
if (adc_key_in < 555) return btnLEFT;
if (adc_key_in < 790) return btnSELECT;
return btnNONE; // when all others fail, return this...
}

// **** Function allowing printing to the display with less code **********
void displayFunction(int column, int row, int number)
{
lcd.setCursor(column,row);
lcd.print("    ");
lcd.setCursor(column,row);
lcd.print(number);
}


/****************************************Copyright(c)*****************************************************
**                            Shenzhen Yuejiang Technology Co., LTD.
**
**                                 http://www.dobot.cc
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           main.cpp
** Latest modified Date:2016-10-24
** Latest Version:      V2.0.0
** Descriptions:        main body
**
**--------------------------------------------------------------------------------------------------------
** Modify by:           Edward
** Modified date:       2016-11-25
** Version:             V1.0.0
** Descriptions:        Modified,From DobotDemoForSTM32
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
#include "stdio.h"
#include "Protocol.h"
#include "command.h"
#include "FlexiTimer2.h"

//Set Serial TX&RX Buffer Size
#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 256

//#define JOG_STICK 
/*********************************************************************************************************
** Global parameters
*********************************************************************************************************/
EndEffectorParams gEndEffectorParams;

JOGJointParams  gJOGJointParams;
JOGCoordinateParams gJOGCoordinateParams;
JOGCommonParams gJOGCommonParams;
JOGCmd          gJOGCmd;

PTPCoordinateParams gPTPCoordinateParams;
PTPCommonParams gPTPCommonParams;
PTPCmd          gPTPCmd;

uint64_t gQueuedCmdIndex;

/*********************************************************************************************************
** Function name:       setup
** Descriptions:        Initializes Serial
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/


Pixy2 pixy;

void setup() {  
Serial.println(" ");
Serial.println("===========Startup sequence, read settings from EEPROM=================");
Serial.println(" ");
// **** setup miscellaneous ********************************** 
    Serial.begin(115200);
    Serial1.begin(115200); 
    printf_begin();
    //Set Timer Interrupt
    FlexiTimer2::set(100,Serialread); 
    FlexiTimer2::start();

// EEPROM load data ******************************************************
// In this section previously stored data is retrieved from the EEPROM if available. If data is not available nothing will be done or a default value will be used

EEPROM.get(eeAddressStartPosAvailable, eePromIntVar);
  if (eePromIntVar != 999) {startPosX = 200.00; startPosY = 0.00; startPosZ = 0.00; startPosR = 0.00; Serial.println("Start position NOT previously saved, using default values:");
     Serial.print(" X"); Serial.print(startPosX); Serial.print(" Y"); Serial.print(startPosY); Serial.print(" Z"); Serial.print(startPosZ); Serial.print(" R"); Serial.println(startPosR);}
  if (eePromIntVar == 999) {EEPROM.get(eeAddressStartPosX, startPosX); EEPROM.get(eeAddressStartPosY, startPosY); EEPROM.get(eeAddressStartPosZ, startPosZ); EEPROM.get(eeAddressStartPosR, startPosR); 
     Serial.print("Start position read from EEPROM: X"); Serial.print(startPosX); Serial.print(" Y"); Serial.print(startPosY); Serial.print(" Z"); Serial.print(startPosZ); Serial.print(" R"); Serial.println(startPosR); }

EEPROM.get(eeAddressCalPosAvailable, eePromIntVar);
  if (eePromIntVar != 999) {calXpos = 200.00; calYpos = -45.00; calZpos = 160.00; calRpos = 0.00; Serial.println("Calibration position NOT previously saved, using default values:");
     Serial.print(" X"); Serial.print(calXpos); Serial.print(" Y"); Serial.print(calYpos); Serial.print(" Z"); Serial.print(calZpos); Serial.print(" R"); Serial.println(calRpos);}
  if (eePromIntVar == 999) {EEPROM.get(eeAddressCalXpos, calXpos); EEPROM.get(eeAddressCalYpos, calYpos); EEPROM.get(eeAddressCalZpos, calZpos); EEPROM.get(eeAddressCalRpos, calRpos); 
     Serial.print("Calibration position read from EEPROM: X"); Serial.print(calXpos); Serial.print(" Y"); Serial.print(calYpos); Serial.print(" Z"); Serial.print(calZpos); Serial.print(" R"); Serial.println(calRpos); }

EEPROM.get(eeAddressA1Available, eePromIntVar);
  if (eePromIntVar != 999) {a1X = 0.00; a1Y = 0.00; Serial.println("A1 calibration dot position NOT previously saved, using default values:"); Serial.print(" X"); Serial.print(a1X); Serial.print(" Y"); Serial.print(a1Y); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressA1x, a1X); EEPROM.get(eeAddressA1y, a1Y); Serial.print("A1 Calibration position read from EEPROM: X"); Serial.print(a1X); Serial.print(" Y"); Serial.println(a1Y); }

EEPROM.get(eeAddressB1Available, eePromIntVar);
  if (eePromIntVar != 999) {b1X = 0.00; a1Y = 0.00; Serial.println("B1 calibration dot position NOT previously saved, using default values:"); Serial.print(" X"); Serial.print(b1X); Serial.print(" Y"); Serial.print(b1Y); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressB1x, b1X); EEPROM.get(eeAddressB1y, b1Y); Serial.print("B1 Calibration position read from EEPROM: X"); Serial.print(b1X); Serial.print(" Y"); Serial.println(b1Y); }

EEPROM.get(eeAddressC1Available, eePromIntVar);
  if (eePromIntVar != 999) {c1X = 0.00; c1Y = 0.00; Serial.println("C1 calibration dot position NOT previously saved, using default values:"); Serial.print(" X"); Serial.print(c1X); Serial.print(" Y"); Serial.print(c1Y); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressC1x, c1X); EEPROM.get(eeAddressC1y, c1Y); Serial.print("C1 Calibration position read from EEPROM: X"); Serial.print(c1X); Serial.print(" Y"); Serial.println(c1Y); }

EEPROM.get(eeAddressA2Available, eePromIntVar);
  if (eePromIntVar != 999) {a2X = 200.00; a2Y = -50; Serial.println("A2 calibration dot position NOT previously saved, using default values:"); Serial.print(" X"); Serial.print(a2X); Serial.print(" Y"); Serial.print(a2Y); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressA2x, a2X); EEPROM.get(eeAddressA2y, a2Y); Serial.print("A2 Calibration position read from EEPROM: X"); Serial.print(a2X); Serial.print(" Y"); Serial.println(a2Y); }

EEPROM.get(eeAddressB2Available, eePromIntVar);
  if (eePromIntVar != 999) {b2X = 200.00; a2Y = 50; Serial.println("B2 calibration dot position NOT previously saved, using default values:"); Serial.print(" X"); Serial.print(b2X); Serial.print(" Y"); Serial.print(b2Y); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressB2x, b2X); EEPROM.get(eeAddressB2y, b2Y); Serial.print("B2 Calibration position read from EEPROM: X"); Serial.print(b2X); Serial.print(" Y"); Serial.println(b2Y); }

EEPROM.get(eeAddressC2Available, eePromIntVar);
  if (eePromIntVar != 999) {c2X = 250.00; c2Y = -50; Serial.println("C2 calibration dot position NOT previously saved, using default values:"); Serial.print(" X"); Serial.print(c2X); Serial.print(" Y"); Serial.print(c2Y); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressC2x, c2X); EEPROM.get(eeAddressC2y, c2Y); Serial.print("C2 Calibration position read from EEPROM: X"); Serial.print(c2X); Serial.print(" Y"); Serial.println(c2Y); }

EEPROM.get(eeAddressZdownAvailable, eePromIntVar);
  if (eePromIntVar != 999) {zDownPos = 0.00; Serial.print("Z-Down position NOT previously saved, using default values:"); Serial.print(" Z"); Serial.println(zDownPos); }
  if (eePromIntVar == 999) {EEPROM.get(eeAddressZdown, zDownPos); Serial.print("Z-Down position read from EEPROM: Z"); Serial.println(zDownPos); }

EEPROM.get(eeAddressDropLocation1Available, eePromIntVar);
  if (eePromIntVar != 999) {dropLocation1X = 220.00; dropLocation1Y = 120.00; dropLocation1Z = 50.00; dropLocation1R = 0.00; Serial.println("Drop position 1 NOT previously saved, using default values:");
     Serial.print(" X"); Serial.print(dropLocation1X); Serial.print(" Y"); Serial.print(dropLocation1Y); Serial.print(" Z"); Serial.print(dropLocation1Z); Serial.print(" R"); Serial.println(dropLocation1R);}
  if (eePromIntVar == 999) {EEPROM.get(eeAddressDropLocation1X, dropLocation1X); EEPROM.get(eeAddressDropLocation1Y, dropLocation1Y); EEPROM.get(eeAddressDropLocation1Z, dropLocation1Z); EEPROM.get(eeAddressDropLocation1R, dropLocation1R); 
     Serial.print("Drop location 1 read from EEPROM: X"); Serial.print(dropLocation1X); Serial.print(" Y"); Serial.print(dropLocation1Y); Serial.print(" Z"); Serial.print(dropLocation1Z); Serial.print(" R"); Serial.println(dropLocation1R); }

EEPROM.get(eeAddressDropLocation2Available, eePromIntVar);
  if (eePromIntVar != 999) {dropLocation2X = 200.00; dropLocation2Y = 0.00; dropLocation2Z = 0.00; dropLocation2R = 0.00; Serial.println("Drop position 2 NOT previously saved, using default values:");
     Serial.print(" X"); Serial.print(dropLocation2X); Serial.print(" Y"); Serial.print(dropLocation2Y); Serial.print(" Z"); Serial.print(dropLocation2Z); Serial.print(" R"); Serial.println(dropLocation2R);}
  if (eePromIntVar == 999) {EEPROM.get(eeAddressDropLocation2X, dropLocation2X); EEPROM.get(eeAddressDropLocation2Y, dropLocation2Y); EEPROM.get(eeAddressDropLocation2Z, dropLocation2Z); EEPROM.get(eeAddressDropLocation2R, dropLocation2R); 
     Serial.print("Drop location 2 read from EEPROM: X"); Serial.print(dropLocation2X); Serial.print(" Y"); Serial.print(dropLocation2Y); Serial.print(" Z"); Serial.print(dropLocation2Z); Serial.print(" R"); Serial.println(dropLocation2R); }

EEPROM.get(eeAddressDropLocation3Available, eePromIntVar);
  if (eePromIntVar != 999) {dropLocation3X = 200.00; dropLocation3Y = 0.00; dropLocation3Z = 0.00; dropLocation3R = 0.00; Serial.println("Drop position 3 NOT previously saved, using default values:");
     Serial.print(" X"); Serial.print(dropLocation3X); Serial.print(" Y"); Serial.print(dropLocation3Y); Serial.print(" Z"); Serial.print(dropLocation3Z); Serial.print(" R"); Serial.println(dropLocation3R);}
  if (eePromIntVar == 999) {EEPROM.get(eeAddressDropLocation3X, dropLocation3X); EEPROM.get(eeAddressDropLocation3Y, dropLocation3Y); EEPROM.get(eeAddressDropLocation3Z, dropLocation3Z); EEPROM.get(eeAddressDropLocation3R, dropLocation3R); 
     Serial.print("Drop location 3 read from EEPROM: X"); Serial.print(dropLocation3X); Serial.print(" Y"); Serial.print(dropLocation3Y); Serial.print(" Z"); Serial.print(dropLocation3Z); Serial.print(" R"); Serial.println(dropLocation3R); }

EEPROM.get(eeAddressEndefectorAvailable, eePromIntVar);
  if (eePromIntVar != 999) {endeffectorGripper = false; Serial.println("End effector type NOT previously saved, using default: Vacuum Cup");}
  if (eePromIntVar == 999) {EEPROM.get(eeAddressEndeffector, endeffectorGripper); Serial.print("End effector type read from EEPROM: ");
      if (endeffectorGripper == 1) {Serial.println("Gripper");} else Serial.println("Vacuum Cup");}


// end EEPROM load data section*******************************************

// **** initialize LCD shield ********************************
lcd.begin(16, 2); // start the library
lcd.setCursor(0,0);
lcd.print("Dobot Pixy2 Demo");
lcd.setCursor(0,1);
lcd.print("Press any key");
// **** initialize PIXY camera *******************************
Serial.println("pixy.init");
pixy.init();
// **** setup arduino pins ***********************************
pinMode(buttonUpPin, INPUT);
pinMode(buttonDownPin, INPUT);
pinMode(buttonLeftPin, INPUT);
pinMode(buttonRightPin, INPUT);
pinMode(button1Pin, INPUT);


}

/*********************************************************************************************************
** Function name:       Serialread
** Descriptions:        import data to rxbuffer
** Input parametersnone:
** Output parameters:   
** Returned value:      
*********************************************************************************************************/
void Serialread()
{
  while(Serial1.available()) {
        uint8_t data = Serial1.read();
        if (RingBufferIsFull(&gSerialProtocolHandler.rxRawByteQueue) == false) {
            RingBufferEnqueue(&gSerialProtocolHandler.rxRawByteQueue, &data);
        }
  }
}

/*********************************************************************************************************
** Function name:       Serial_putc
** Descriptions:        Remap Serial to Printf
** Input parametersnone:
** Output parameters:   
** Returned value:      
*********************************************************************************************************/
int Serial_putc( char c, struct __file * )
{
    Serial.write( c );
    return c;
}

/*********************************************************************************************************
** Function name:       printf_begin
** Descriptions:        Initializes Printf
** Input parameters:    
** Output parameters:
** Returned value:      
*********************************************************************************************************/

void printf_begin(void)
{
    fdevopen( &Serial_putc, 0 );
}

/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/

void InitRAM(void)
{
    //Set JOG Model
    gJOGJointParams.velocity[0] = 100;
    gJOGJointParams.velocity[1] = 100;
    gJOGJointParams.velocity[2] = 100;
    gJOGJointParams.velocity[3] = 100;
    gJOGJointParams.acceleration[0] = 80;
    gJOGJointParams.acceleration[1] = 80;
    gJOGJointParams.acceleration[2] = 80;
    gJOGJointParams.acceleration[3] = 80;

    gJOGCoordinateParams.velocity[0] = 100;
    gJOGCoordinateParams.velocity[1] = 100;
    gJOGCoordinateParams.velocity[2] = 100;
    gJOGCoordinateParams.velocity[3] = 100;
    gJOGCoordinateParams.acceleration[0] = 80;
    gJOGCoordinateParams.acceleration[1] = 80;
    gJOGCoordinateParams.acceleration[2] = 80;
    gJOGCoordinateParams.acceleration[3] = 80;

    gJOGCommonParams.velocityRatio = 50;
    gJOGCommonParams.accelerationRatio = 50;
   
    gJOGCmd.cmd = AP_DOWN;
    gJOGCmd.isJoint = JOINT_MODEL;

    

    //Set PTP Model
    gPTPCoordinateParams.xyzVelocity = 100;
    gPTPCoordinateParams.rVelocity = 100;
    gPTPCoordinateParams.xyzAcceleration = 80;
    gPTPCoordinateParams.rAcceleration = 80;

    gPTPCommonParams.velocityRatio = 50;
    gPTPCommonParams.accelerationRatio = 50;

    gPTPCmd.ptpMode = MOVL_XYZ;
    
gQueuedCmdIndex = 0;

//Set initial pose (start position)
moveArm(startPosX, startPosY, startPosZ, startPosR, false);
}

/*********************************************************************************************************
** Function name:       loop
** Descriptions:        Program entry
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/

void moveArm(float x, float y, float z, float r, bool vacuumOn)
{

  gPTPCmd.x = x;
  gPTPCmd.y = y;
  gPTPCmd.z = z;
  gPTPCmd.r = r;

Serial.print("move to x:"); Serial.print(gPTPCmd.x); Serial.print(" y:"); Serial.print(gPTPCmd.y); Serial.print(" z:"); Serial.println(gPTPCmd.r);

SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);

if (endeffectorGripper == true) {
  if (vacuumOn == false && vacuumOn != currentVac) {
      Serial.println("Open GRIPPER");
      //delay(1000);
      SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
      ProtocolProcess(); //have command(s) executed by dobot
      delay(500);
      SetEndEffectorGripper(true, true, &gQueuedCmdIndex); // open gripper (compressed air on);
      ProtocolProcess(); //have command(s) executed by dobot
      delay(500);
      SetEndEffectorGripper(false, true, &gQueuedCmdIndex);  // stop activating gripper when it is openend (compressed air off)
      delay(500); 
      }
}
else{
if (vacuumOn == false) SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
}

if (vacuumOn == true && vacuumOn != currentVac)SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
ProtocolProcess();

currentX = x;
currentY = y;
currentZ = z;
currentR = r;
currentVac = vacuumOn;
}



void loop() 
{
    
    InitRAM();

    ProtocolInit();
    
    SetJOGJointParams(&gJOGJointParams, true, &gQueuedCmdIndex);
    
    SetJOGCoordinateParams(&gJOGCoordinateParams, true, &gQueuedCmdIndex);
    
    SetJOGCommonParams(&gJOGCommonParams, true, &gQueuedCmdIndex);
    
    printf("\r\n======Main Program loop started======\r\n");

    SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
    ProtocolProcess(); 
  
    for(; ;) //start infinite loop
    {


lcd.setCursor(0,1); // move to the begining of the second line

// **** MENU 0 (start screen) ***************************************************************************
if (menuItem == 0) {
lcd_key = read_LCD_buttons(); // read the buttons
switch (lcd_key) // depending on which button was pushed, we perform an action
{
 case btnRIGHT:
  {if (prevKeyState == 0){ 
  menuItem ++; drawMenu = true;
 prevKeyState = 1;}
 break; }
  case btnLEFT:
 {if (prevKeyState == 0){ 
  menuItem ++; drawMenu = true;
 prevKeyState = 1;}
 break; }
case btnDOWN:
 {if (prevKeyState == 0){ 
  menuItem ++; drawMenu = true;
 prevKeyState = 1;}
 break; }
 case btnSELECT:
 {if (prevKeyState == 0){ 
  menuItem ++; drawMenu = true;
 prevKeyState = 1;}
 break; }
 case btnNONE:
 {
prevKeyState = 0;
  break;
 }
}
}
// ***************************************************************************************
// **** MENU 1 ***************************************************************************
// ***************************************************************************************
if (menuItem == 1) {
  if (drawMenu) {
lcd.clear(); displayFunction(0,0,menuItem); lcd.setCursor(1,0); lcd.print ("- Manual Jog  ");
lcd.setCursor(0,1); lcd.print("Inc:   "); lcd.setCursor(4,1); moveInterval = 1.0; lcd.print("1.0");
activeAxis = 1; lcd.setCursor(9,1); lcd.print("X"); lcd.setCursor(10,1); lcd.print("      "); lcd.setCursor(10,1); lcd.print(gPTPCmd.x);
drawMenu = false;
  }
lcd_key = read_LCD_buttons(); // read the buttons
switch (lcd_key) // depending on which button was pushed, we perform an action
{
 case btnRIGHT:
  {if (prevKeyState == 0){  activeAxis ++; 
  if (activeAxis == 6) {activeAxis = 1;}
  if (activeAxis == 1) {lcd.setCursor(9,1); lcd.print("X"); lcd.setCursor(10,1); lcd.print("      "); lcd.setCursor(10,1); lcd.print(gPTPCmd.x);}//displayFunction(10,1,gPTPCmd.x);}
  if (activeAxis == 2) {lcd.setCursor(9,1); lcd.print("Y"); lcd.setCursor(10,1); lcd.print("      "); lcd.setCursor(10,1); lcd.print(gPTPCmd.y);}//displayFunction(10,1,gPTPCmd.y);}
  if (activeAxis == 3) {lcd.setCursor(9,1); lcd.print("Z"); lcd.setCursor(10,1); lcd.print("      "); lcd.setCursor(10,1); lcd.print(gPTPCmd.z);}//displayFunction(10,1,gPTPCmd.z);}
  if (activeAxis == 4) {lcd.setCursor(9,1); lcd.print("R"); lcd.setCursor(10,1); lcd.print("      "); lcd.setCursor(10,1); lcd.print(gPTPCmd.r);}//displayFunction(10,1,gPTPCmd.r);}
  if (activeAxis == 5) {lcd.setCursor(9,1); lcd.print("Vac"); lcd.setCursor(12,1); lcd.print("      "); lcd.setCursor(13,1); lcd.print(currentVac);}//displayFunction(10,1,gPTPCmd.r);}
  }
 prevKeyState = 1;
 break;  }
  case btnLEFT:
 {if (prevKeyState == 0){  
   /*
    * switch (moveInterval) {
      case 100: {moveInterval = 1000; lcd.setCursor(0,1); lcd.print("Inc:  "); lcd.setCursor(4,1); lcd.print(moveInterval/1000); break;}
      case 1000:  {moveInterval = 10000; lcd.setCursor(0,1); lcd.print("Inc:  "); lcd.setCursor(4,1); lcd.print(moveInterval/1000); break;}
      case 10000: {moveInterval = 20000;  lcd.setCursor(0,1); lcd.print("Inc:  "); lcd.setCursor(4,1); lcd.print(moveInterval/1000); break;}
      case 20000: {moveInterval = 100;  lcd.setCursor(0,1); lcd.print("Inc:  "); lcd.setCursor(4,1); lcd.print(moveInterval/1000); break;}
    */
   
      if (moveInterval == 0.1) {moveInterval = 1.0; lcd.setCursor(0,1); lcd.print("Inc:    "); lcd.setCursor(4,1); lcd.print("1.0");}
      else if (moveInterval == 1.0) {moveInterval = 10.0; lcd.setCursor(0,1); lcd.print("Inc:    "); lcd.setCursor(4,1); lcd.print("10.0");}
      else if (moveInterval == 10.0) {moveInterval = 20.0;  lcd.setCursor(0,1); lcd.print("Inc:    "); lcd.setCursor(4,1); lcd.print("20.0");}
      else if (moveInterval == 20.0) {moveInterval = 0.1;  lcd.setCursor(0,1); lcd.print("Inc:    "); lcd.setCursor(4,1); lcd.print("0.1");}
   
 prevKeyState = 1;}
 break;  }
 case btnUP:
 {if (prevKeyState == 0){ 
    if (activeAxis == 1){moveArm((gPTPCmd.x += moveInterval), currentY, currentZ, currentR, currentVac); lcd.setCursor(10,1); lcd.print("      "); lcd.setCursor(10,1); lcd.print(gPTPCmd.x); prevKeyState = 1;}
    if (activeAxis == 2){moveArm(currentX, (gPTPCmd.y += moveInterval), currentZ, currentR, currentVac); lcd.setCursor(10,1); lcd.print("      "); lcd.setCursor(10,1); lcd.print(gPTPCmd.y); prevKeyState = 1;}
    if (activeAxis == 3){moveArm(currentX, currentY, (gPTPCmd.z += moveInterval), currentR, currentVac); lcd.setCursor(10,1); lcd.print("      "); lcd.setCursor(10,1); lcd.print(gPTPCmd.z); prevKeyState = 1;}
    if (activeAxis == 4){moveArm(currentX, currentY, currentZ, (gPTPCmd.r += moveInterval), currentVac); lcd.setCursor(10,1); lcd.print("      "); lcd.setCursor(10,1); lcd.print(gPTPCmd.r); prevKeyState = 1;}
    if (activeAxis == 5){moveArm(currentX, currentY, currentZ, currentR, true); lcd.setCursor(12,1); lcd.print("      "); lcd.setCursor(13,1); lcd.print(currentVac); prevKeyState = 1;}
 }
 break;  }
case btnDOWN:
 {if (prevKeyState == 0){ 
    if (activeAxis == 1){moveArm((gPTPCmd.x -= moveInterval), currentY, currentZ, currentR, currentVac); lcd.setCursor(10,1); lcd.print("      "); lcd.setCursor(10,1); lcd.print(gPTPCmd.x); prevKeyState = 1;}
    if (activeAxis == 2){moveArm(currentX, (gPTPCmd.y -= moveInterval), currentZ, currentR, currentVac); lcd.setCursor(10,1); lcd.print("      "); lcd.setCursor(10,1); lcd.print(gPTPCmd.y); prevKeyState = 1;}
    if (activeAxis == 3){moveArm(currentX, currentY, (gPTPCmd.z -= moveInterval), currentR, currentVac); lcd.setCursor(10,1); lcd.print("      "); lcd.setCursor(10,1); lcd.print(gPTPCmd.z); prevKeyState = 1;}
    if (activeAxis == 4){moveArm(currentX, currentY, currentZ, (gPTPCmd.r -= moveInterval), currentVac); lcd.setCursor(10,1); lcd.print("      "); lcd.setCursor(10,1); lcd.print(gPTPCmd.r); prevKeyState = 1;}
    if (activeAxis == 5){moveArm(currentX, currentY, currentZ, currentR, false); lcd.setCursor(12,1); lcd.print("      "); lcd.setCursor(13,1); lcd.print(currentVac); prevKeyState = 1;}
 }
 break; } 
 case btnSELECT:
 {if (prevKeyState == 0){ 
  menuItem ++; drawMenu = true;
 prevKeyState = 1;}
 break; }
 case btnNONE:
 {
prevKeyState = 0;
  break;
 }
}
}
// **************************************************************************************
// **** MENU 2 **************************************************************************
// **************************************************************************************
if (menuItem == 2) {
  if (drawMenu) {
lcd.clear(); displayFunction(0,0,menuItem); lcd.setCursor(1,0); lcd.print ("- Pick & Place");
lcd.setCursor(0,1); lcd.print("Pick Part?   [>]");
drawMenu = false;
  }
lcd_key = read_LCD_buttons(); // read the buttons
switch (lcd_key)
{
 case btnRIGHT:
  {if (prevKeyState == 0){}
do{
  moveArm(calXpos, calYpos, calZpos, currentR, currentVac);
  lcd.setCursor(0,1); lcd.print("Moving arm... 3 ");  delay(1000); // give arm time to move to calibration position before capturing capturing image
  lcd.setCursor(0,1); lcd.print("Moving arm... 2 ");  delay(1000);
  lcd.setCursor(0,1); lcd.print("Moving arm... 1 ");  delay(1000);
  lcd.setCursor(0,1); lcd.print("Moving arm... 0 ");  delay(1000); 
  static int i; static int j = 0; static int k = 0; // counters for loops
  // grab blocks!
  pixy.ccc.getBlocks();

  signature1Found = 0; 
  signature2Found = 0; 
  signature3Found = 0;
  Serial.print("pixy.ccc.numBlocks = "); Serial.println(pixy.ccc.numBlocks);
  for (i=0; i<pixy.ccc.numBlocks; i++){ 
  if (pixy.ccc.blocks[i].m_signature == signaturePart1){signature1Found++; j = i; activeSignature = 1; //check if signature is a part 1 signature and set j counter to array address if true;
  pixy.ccc.blocks[i].print();} //print found signature on serial monitor
  else if (pixy.ccc.blocks[i].m_signature == signaturePart2){signature2Found++; j = i; activeSignature = 2; //check if signature is a part 1 signature and set j counter to array address if true;
  pixy.ccc.blocks[i].print();} //print found signature on serial monitor
  else if (pixy.ccc.blocks[i].m_signature == signaturePart3){signature3Found++; j = i; activeSignature = 3; //check if signature is a part 1 signature and set j counter to array address if true;
  pixy.ccc.blocks[i].print();} //print found signature on serial monitor
  }
if (signature1Found > 0 || signature2Found > 0 || signature3Found > 0) {
  pixyPartX = pixy.ccc.blocks[j].m_x; //Select first part found in array
  pixyPartY = pixy.ccc.blocks[j].m_y;
Serial.println("The following part will be picked:");
pixy.ccc.blocks[j].print();
Serial.print("pixyPartX: "); Serial.println(pixyPartX);
Serial.print("pixyPartY: "); Serial.println(pixyPartY);
 //Flip Y axis (Pixy Y=0 is at the top of the image with positive direction downwards, while Dobot Y=0 is at the bottom with positive direction upwards)
  pixyPartY = (207 - pixyPartY); // Flip Y coordinate by subtracting it from max pixel number of pixy camera (207 pixels) 
  Serial.print("pixyPartY: after flipping Y axis with 207 - pixyPartY: "); Serial.println(pixyPartY);
  //calculate scale between sheet and Dobot coordinates   
  scaleX = (sqrt(sq(b2X-a2X)+sq(b2Y-a2Y)))/(sqrt(sq(b1X-a1X)+sq(b1Y-a1Y)));    Serial.print("scaleX: "); Serial.println(scaleX);
  scaleY = (sqrt(sq(c2X-a2X)+sq(c2Y-a2Y)))/(sqrt(sq(c1X-a1X)+sq(c1Y-a1Y)));    Serial.print("scaleY: "); Serial.println(scaleY);
  r1 = (sqrt(sq(b1X-a1X)+sq(b1Y-a1Y)));     Serial.print("r1: "); Serial.println(r1);
  r2 = (sqrt(sq(b2X-a2X)+sq(b2Y-a2Y)));     Serial.print("r2: "); Serial.println(r2);
  //translate to origin
  dobotPartX = pixyPartX - a1X;             Serial.print("dobotPartX after translate to origin: "); Serial.println(dobotPartX);       //translate to origin
  dobotPartY = pixyPartY - a1Y;             Serial.print("dobotPartY after translate to origin: "); Serial.println(dobotPartY);       //translate to origin 
  angle1     = (acos((b1X-a1X)/r1));        Serial.print("angle1: "); Serial.println(angle1);                                    //angle between calibration sheet and pixy coordinate system
  dobotPartX = (dobotPartX*cos(-1*angle1))+(dobotPartY*sin(-1*angle1)); Serial.print("dobotPartX after rotate: "); Serial.println(dobotPartX);//rotate X coordinate from sheet to Pixy coordinate system
  dobotPartY = ((-1*dobotPartX)*sin(-1*angle1))+(dobotPartY*cos(-1*angle1));  Serial.print("dobotPartY after rotate: "); Serial.println(dobotPartY);//rotate Y coordinate from sheet to Pixy coordinate system
  dobotPartX = dobotPartX*scaleX;           Serial.print("dobotPartX after scale: "); Serial.println(dobotPartX);                //scale X between Pixy and Dobot
  dobotPartY = dobotPartY*scaleY;           Serial.print("dobotPartY after scale: "); Serial.println(dobotPartY);                //scale Y between Pixy and Dobot
  angle2     = (acos((b2X-a2X)/r2));        Serial.print("angle2: "); Serial.println(angle2);                                    //angle between Pixy and Dobot coordinate systems
  intermediateX = dobotPartX; intermediateY = dobotPartY;               // write to intermediate variable to avoid miscalculation in next step
  dobotPartX = (intermediateX*cos(angle2))+(intermediateY*sin(angle2));       Serial.print("dobotPartX after rotate to Dobot coordinate system: "); Serial.println(dobotPartX);// rotate X coordinate to Dobot coordinate system
  dobotPartY = ((-1*intermediateX)*sin(angle2))+(intermediateY*cos(angle2));  Serial.print("dobotPartY dobotPartX after rotate to Dobot coordinate system: "); Serial.println(dobotPartY);// rotate Y coordinate to Dobot coordinate system
  dobotPartX = dobotPartX + a2X;            Serial.print("dobotPartX after translate: "); Serial.println(dobotPartX);           //translate from origin to destination
  dobotPartY = dobotPartY + a2Y;            Serial.print("dobotPartY after translate: "); Serial.println(dobotPartY);           //translate from origin to destination

lcd.setCursor(0,1); lcd.print("                ");
lcd.setCursor(0,1); lcd.print("X"); lcd.setCursor(1,1); lcd.print(dobotPartX);
lcd.setCursor(8,1); lcd.print("Y"); lcd.setCursor(9,1); lcd.print(dobotPartY);

moveArm(dobotPartX, dobotPartY, (zDownPos+20), currentR, currentVac);
delay(2000);
moveArm(dobotPartX, dobotPartY, zDownPos, currentR, currentVac);
delay(1500);
moveArm(dobotPartX, dobotPartY, zDownPos, currentR, true);
delay(500);
switch (activeSignature) {
  case 1:
    moveArm(dobotPartX, dobotPartY, dropLocation1Z, currentR, currentVac);
    moveArm(dropLocation1X, dropLocation1Y, dropLocation1Z, currentR, currentVac);
    delay(3500);
    moveArm(dropLocation1X, dropLocation1Y, dropLocation1Z, currentR, false);
    delay(1300);
    break;
  case 2:
    moveArm(dobotPartX, dobotPartY, dropLocation2Z, currentR, currentVac);
    moveArm(dropLocation2X, dropLocation2Y, dropLocation2Z, currentR, currentVac);
    delay(3500);
    moveArm(dropLocation2X, dropLocation2Y, dropLocation2Z, currentR, false);
    delay(500);
    break;
  case 3:
    moveArm(dobotPartX, dobotPartY, dropLocation3Z, currentR, currentVac);
    moveArm(dropLocation3X, dropLocation3Y, dropLocation3Z, currentR, currentVac);
    delay(3500);
    moveArm(dropLocation3X, dropLocation3Y, dropLocation3Z, currentR, false);
    delay(500);
    break;
}
}



else {
  Serial.println(" ");
  Serial.print("Error: no parts found, at least 1 part required to start pick and place action.");
  Serial.println("Pick and Place aborted");
  lcd.setCursor(0,1); lcd.print("ZERO parts found"); 
  }
  //delay(3000);
  
  }while (signature1Found > 0 || signature2Found > 0 || signature3Found > 0);
  
 break;  }
  case btnLEFT:
 {if (prevKeyState == 0){}
 break;  }
 case btnUP:
 {if (prevKeyState == 0){}
 break;  }
case btnDOWN:
 {if (prevKeyState == 0){}
 break; } 
 case btnSELECT:
 {if (prevKeyState == 0){ 
  menuItem ++; drawMenu = true;
 prevKeyState = 1;}
 break; }
 case btnNONE:
 {
prevKeyState = 0;
  break;
 }
}
}
// **************************************************************************************
// **** MENU 3 **************************************************************************
// **************************************************************************************
if (menuItem == 3) {
  if (drawMenu) {
      displayFunction(0,0,menuItem);  lcd.setCursor(1,0); lcd.print ("- Settings     ");
      menuChoice = 1; lcd.setCursor(0,1); lcd.print("Set cal pos? [>]");
      drawMenu = false;
      }
lcd_key = read_LCD_buttons(); // read the buttons
switch (lcd_key)
{
 case btnRIGHT:
  {if (prevKeyState == 0){   
  if (menuChoice == 1) {lcd.setCursor(0,1); lcd.print("Cal Pos stored! ");
  calXpos = gPTPCmd.x; calYpos = gPTPCmd.y; calZpos = gPTPCmd.z; calRpos = gPTPCmd.r;

EEPROM.put(eeAddressCalXpos, calXpos); EEPROM.get(eeAddressCalXpos, calXpos); Serial.print("calXpos stored in eeprom: "); Serial.println(calXpos);
EEPROM.put(eeAddressCalYpos, calYpos); EEPROM.get(eeAddressCalYpos, calYpos); Serial.print("calYpos stored in eeprom ");  Serial.println(calYpos);
EEPROM.put(eeAddressCalZpos, calZpos); EEPROM.get(eeAddressCalZpos, calZpos); Serial.print("calZpos stored in eeprom ");  Serial.println(calZpos);
EEPROM.put(eeAddressCalRpos, calRpos); EEPROM.get(eeAddressCalRpos, calRpos); Serial.print("calRpos stored in eeprom");   Serial.println(calRpos);
EEPROM.put(eeAddressCalPosAvailable, 999); // setting this address to 999 means calibration position has been stored and and can be safely retreived when arduino restarts.
EEPROM.get(eeAddressCalPosAvailable, eePromIntVar); Serial.print("calPosAvailable stored in EEPROM: "); Serial.println(eePromIntVar);
  
  }
  if (menuChoice == 2) {lcd.setCursor(0,1); lcd.print("Now in Cal Pos! ");
  moveArm(calXpos, calYpos, calZpos, currentR, currentVac);
  }
  if (menuChoice == 3) { EEPROM.put (eeAddressZdown, gPTPCmd.z); EEPROM.put (eeAddressZdownAvailable, 999); 
  EEPROM.get(eeAddressZdown, zDownPos); Serial.print("Z-Down stored in eeprom: Z"); Serial.println(zDownPos); 
  lcd.setCursor(0,1); lcd.print("Z-down set!     ");
  }
  if (menuChoice == 4) {lcd.setCursor(0,1); lcd.print("Now in Z-down!  ");
  moveArm(currentX, currentY, zDownPos, currentR, currentVac);
  }
if (menuChoice == 5) { EEPROM.put (eeAddressA2x, gPTPCmd.x); EEPROM.put (eeAddressA2y, gPTPCmd.y); EEPROM.put (eeAddressA2Available, 999); 
  EEPROM.get(eeAddressA2x, a2X); Serial.print("a2X stored in eeprom: "); Serial.println(a2X); EEPROM.get(eeAddressA2y, a2Y); Serial.print("a2Y stored in eeprom: "); Serial.println(a2Y);
  lcd.setCursor(0,1); lcd.print("A_xy set!       ");
  }
  if (menuChoice == 6) {lcd.setCursor(0,1); lcd.print("Now in A_xy!    ");
  moveArm(a2X, a2Y, currentZ, currentR, currentVac);
  }
  if (menuChoice == 7) {EEPROM.put (eeAddressB2x, gPTPCmd.x); EEPROM.put (eeAddressB2y, gPTPCmd.y); EEPROM.put (eeAddressB2Available, 999);
  EEPROM.get(eeAddressB2x, b2X); Serial.print("b2X stored in eeprom: "); Serial.println(b2X); EEPROM.get(eeAddressB2y, b2Y); Serial.print("b2Y stored in eeprom: "); Serial.println(b2Y);
  lcd.setCursor(0,1); lcd.print("B_xy set!       ");
  
  }
  if (menuChoice == 8) {lcd.setCursor(0,1); lcd.print("Now in B_xy!    ");
  moveArm(b2X, b2Y, currentZ, currentR, currentVac);
  }
  if (menuChoice == 9) {EEPROM.put (eeAddressC2x, gPTPCmd.x); EEPROM.put (eeAddressC2y, gPTPCmd.y); EEPROM.put (eeAddressC2Available, 999);
  EEPROM.get(eeAddressC2x, c2X); Serial.print("c2X stored in eeprom: "); Serial.println(c2X); EEPROM.get(eeAddressC2y, c2Y); Serial.print("c2Y stored in eeprom: "); Serial.println(c2Y);
  lcd.setCursor(0,1); lcd.print("C_xy set!       ");
  
  }
  if (menuChoice == 10) {lcd.setCursor(0,1); lcd.print("Now in C_xy!    ");
  moveArm(c2X, c2Y, currentZ, currentR, currentVac);
  }

  if (menuChoice == 11) {EEPROM.put (eeAddressStartPosX, gPTPCmd.x); EEPROM.put (eeAddressStartPosY, gPTPCmd.y); EEPROM.put (eeAddressStartPosZ, gPTPCmd.z); EEPROM.put (eeAddressStartPosR, gPTPCmd.r); EEPROM.put (eeAddressStartPosAvailable, 999);
  EEPROM.get(eeAddressStartPosX, startPosX); Serial.print("startPosX stored in eeprom: "); Serial.println(startPosX); EEPROM.get(eeAddressStartPosY, startPosY); Serial.print("startPosY stored in eeprom: "); Serial.println(startPosY);
  EEPROM.get(eeAddressStartPosZ, startPosZ); Serial.print("startPosZ stored in eeprom: "); Serial.println(startPosZ); EEPROM.get(eeAddressStartPosR, startPosR); Serial.print("startPosR stored in eeprom: "); Serial.println(startPosR);
  lcd.setCursor(0,1); lcd.print("Start stored!   ");
  }

  if (menuChoice == 12) {lcd.setCursor(0,1); lcd.print("In start pos!   ");
  moveArm(startPosX, startPosY, startPosZ, startPosR, currentVac);
  }

if (menuChoice == 13) {EEPROM.put (eeAddressDropLocation1X , gPTPCmd.x); EEPROM.put (eeAddressDropLocation1Y , gPTPCmd.y); EEPROM.put (eeAddressDropLocation1Z , gPTPCmd.z); EEPROM.put (eeAddressDropLocation1R , gPTPCmd.r); 
  EEPROM.put (eeAddressDropLocation1Available , 999);
  EEPROM.get(eeAddressDropLocation1X, dropLocation1X); Serial.print("Drop location1 X stored in eeprom: "); Serial.println(startPosX); EEPROM.get(eeAddressDropLocation1Y, dropLocation1Y); Serial.print("Drop location1 Y stored in eeprom: "); Serial.println(startPosY);
  EEPROM.get(eeAddressDropLocation1Z, dropLocation1Z); Serial.print("Drop location1 Z stored in eeprom: "); Serial.println(startPosZ); EEPROM.get(eeAddressDropLocation1R, dropLocation1R); Serial.print("Drop location1 R stored in eeprom: "); Serial.println(startPosR);
  lcd.setCursor(0,1); lcd.print("DropPos1 stored!");
  }

  if (menuChoice == 14) {lcd.setCursor(0,1); lcd.print("In Drop location1!   ");
  moveArm(dropLocation1X, dropLocation1Y, dropLocation1Z, dropLocation1R, currentVac);
  }

if (menuChoice == 15) {EEPROM.put (eeAddressDropLocation2X , gPTPCmd.x); EEPROM.put (eeAddressDropLocation2Y , gPTPCmd.y); EEPROM.put (eeAddressDropLocation2Z , gPTPCmd.z); EEPROM.put (eeAddressDropLocation2R , gPTPCmd.r); 
  EEPROM.put (eeAddressDropLocation2Available , 999);
  EEPROM.get(eeAddressDropLocation2X, dropLocation2X); Serial.print("Drop location2 X stored in eeprom: "); Serial.println(startPosX); EEPROM.get(eeAddressDropLocation2Y, dropLocation2Y); Serial.print("Drop location2 Y stored in eeprom: "); Serial.println(startPosY);
  EEPROM.get(eeAddressDropLocation2Z, dropLocation2Z); Serial.print("Drop location2 Z stored in eeprom: "); Serial.println(startPosZ); EEPROM.get(eeAddressDropLocation2R, dropLocation2R); Serial.print("Drop location2 R stored in eeprom: "); Serial.println(startPosR);
  lcd.setCursor(0,1); lcd.print("DropPos2 stored!");
  }

  if (menuChoice == 16) {lcd.setCursor(0,1); lcd.print("In Drop location2!   ");
  moveArm(dropLocation2X, dropLocation2Y, dropLocation2Z, dropLocation2R, currentVac);
  }

if (menuChoice == 17) {EEPROM.put (eeAddressDropLocation3X , gPTPCmd.x); EEPROM.put (eeAddressDropLocation3Y , gPTPCmd.y); EEPROM.put (eeAddressDropLocation3Z , gPTPCmd.z); EEPROM.put (eeAddressDropLocation3R , gPTPCmd.r); 
  EEPROM.put (eeAddressDropLocation3Available , 999);
  EEPROM.get(eeAddressDropLocation3X, dropLocation3X); Serial.print("Drop location3 X stored in eeprom: "); Serial.println(startPosX); EEPROM.get(eeAddressDropLocation3Y, dropLocation3Y); Serial.print("Drop location3 Y stored in eeprom: "); Serial.println(startPosY);
  EEPROM.get(eeAddressDropLocation3Z, dropLocation3Z); Serial.print("Drop location3 Z stored in eeprom: "); Serial.println(startPosZ); EEPROM.get(eeAddressDropLocation3R, dropLocation3R); Serial.print("Drop location3 R stored in eeprom: "); Serial.println(startPosR);
  lcd.setCursor(0,1); lcd.print("DropPos3 stored!");
  }

  if (menuChoice == 18) {lcd.setCursor(0,1); lcd.print("In Drop location3!   ");
  moveArm(dropLocation3X, dropLocation3Y, dropLocation3Z, dropLocation3R, currentVac);
  }

if (menuChoice == 19) {lcd.setCursor(0,1); lcd.print("Calib started   ");

 
  //Move arm to calibration position
  moveArm(calXpos, calYpos, calZpos, currentR, currentVac);
  lcd.setCursor(0,1); lcd.print("Moving arm... 3 "); 
  delay(1000); // give arm time to move to calibration position before capturing capturing image
  lcd.setCursor(0,1); lcd.print("Moving arm... 2 ");
  delay(1000);
  lcd.setCursor(0,1); lcd.print("Moving arm... 1 ");
  delay(1000);
  lcd.setCursor(0,1); lcd.print("Moving arm... 0 ");
  delay(1200); 
  static int i; static int j = 0; static int k = 0; // counters for loops
  static int CalibDotFound = 0; // counter for how many calibration dots were found
  // grab blocks!
  pixy.ccc.getBlocks();

CalibDotFound = 0;
Serial.print("pixy.ccc.numBlocks = "); Serial.println(pixy.ccc.numBlocks);
for (i=0; i<pixy.ccc.numBlocks; i++){ 
  if (pixy.ccc.blocks[i].m_signature == signatureCalib){CalibDotFound++;} //check if signature is a calibration dot and add to counter if true
  pixy.ccc.blocks[i].print(); //print found signature on serial monitor
  
}
  // If there are blocks detected, print them!
  if (CalibDotFound == 3)
  {
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
      //Serial.print("Part Signature: "); Serial.println(pixyPartSignature);
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      pixy.ccc.blocks[i].print();
      }
    //sort array with found blocks from smallest to largest
    // note that the X and Y are added for each point to easily sort without having to evaluate X and Y individually.
    // smallest value is coordinate C (top left), middle is A (bottom left) and highest number is B (bottom right).
    for (j=0; j<pixy.ccc.numBlocks; j++){
      for (k=0; k<(pixy.ccc.numBlocks-1); k++){
        if ((pixy.ccc.blocks[k].m_x + pixy.ccc.blocks[k].m_y) > (pixy.ccc.blocks[k+1].m_x + pixy.ccc.blocks[k+1].m_y)) {
         tempFloatVarX = pixy.ccc.blocks[k].m_x; tempFloatVarY = pixy.ccc.blocks[k].m_y; tempIntVarSignature = pixy.ccc.blocks[k].m_signature;
         pixy.ccc.blocks[k].m_x = pixy.ccc.blocks[k+1].m_x; pixy.ccc.blocks[k].m_y = pixy.ccc.blocks[k+1].m_y; pixy.ccc.blocks[k].m_signature = pixy.ccc.blocks[k+1].m_signature;
         pixy.ccc.blocks[k+1].m_x = tempFloatVarX; pixy.ccc.blocks[k+1].m_y = tempFloatVarY; pixy.ccc.blocks[k+1].m_signature = tempIntVarSignature;
        }
      }
    }
    
    k=0;
    for (j=0; j<pixy.ccc.numBlocks; j++){
      Serial.print(pixy.ccc.blocks[j].m_signature);
      if (pixy.ccc.blocks[j].m_signature == signatureCalib){
    //for (k=0; k<3; k++){
    //calibration point A is defined as the lower left coordinate, which has the mid value for the sum of its X and Y coordinates and therefore position 2 in the array
    if (k==1){a1X = pixy.ccc.blocks[j].m_x;} 
    if (k==1){a1Y = (207 - pixy.ccc.blocks[j].m_y);} //flip Y coordinate from Pixy by subtracting it from the max number of Y pixels on camera (207)
    //caliobration point B is defined as the lower right coordinate, which has the highest value for the sum of its X and Y coordinates and therefore position 3 in the array
    if (k==2){b1X =pixy.ccc.blocks[j].m_x;}
    if (k==2){b1Y = (207 - pixy.ccc.blocks[j].m_y);} //flip Y coordinate from Pixy by subtracting it from the max number of Y pixels on camera (207)
    //calibration point C is defined as the top left coordinate, which has the lowest value for the sum of its X and Y coordinates and therefore position 1 in the array
    if (k==0){c1X = pixy.ccc.blocks[j].m_x;}
    if (k==0){c1Y = (207 - pixy.ccc.blocks[j].m_y);} //flip Y coordinate from Pixy by subtracting it from the max number of Y pixels on camera (207)
    k++;
    //}
    }
    }
    //}

      Serial.println(" ");
      Serial.println("3 Calibration points found. Writing to EEPROM:");

      EEPROM.put (eeAddressA1x, a1X); EEPROM.put (eeAddressA1y, a1Y); EEPROM.put (eeAddressA1Available, 999); 
      EEPROM.get(eeAddressA1x, a1X); Serial.print("a1X stored in eeprom: "); Serial.println(a1X); EEPROM.get(eeAddressA1y, a1Y); Serial.print("a1Y stored in eeprom: "); Serial.println(a1Y);

      EEPROM.put (eeAddressB1x, b1X); EEPROM.put (eeAddressB1y, b1Y); EEPROM.put (eeAddressB1Available, 999);
      EEPROM.get(eeAddressB1x, b1X); Serial.print("b1X stored in eeprom: "); Serial.println(b1X); EEPROM.get(eeAddressB1y, b1Y); Serial.print("b1Y stored in eeprom: "); Serial.println(b1Y);

      EEPROM.put (eeAddressC1x, c1X); EEPROM.put (eeAddressC1y, c1Y); EEPROM.put (eeAddressC1Available, 999);
      EEPROM.get(eeAddressC1x, c1X); Serial.print("c1X stored in eeprom: "); Serial.println(c1X); EEPROM.get(eeAddressC1y, c1Y); Serial.print("c1Y stored in eeprom: "); Serial.println(c1Y);

    
  lcd.setCursor(0,1); lcd.print("Calib. done!    "); 
}  
  else{
    Serial.println(" ");
    Serial.print("Error: incorrect numer of blocks found (3 required, found: "); Serial.println(CalibDotFound);
    Serial.println("Calibration aborted");
    lcd.setCursor(0,1); lcd.print("Calib. FAILED   "); 
  }

  
  }

  if (menuChoice == 20) {
    if (endeffectorGripper == true) {endeffectorGripper = false; lcd.setCursor(0,1); lcd.print("Vacuum Cup set! ");
  EEPROM.put (eeAddressEndeffector , endeffectorGripper); EEPROM.put (eeAddressEndefectorAvailable , 999);
  EEPROM.get(eeAddressEndeffector, endeffectorGripper); Serial.print("End effector type stored in eeprom: "); 
  Serial.print(endeffectorGripper); Serial.println("( Gripper)");  
    }
    else {endeffectorGripper = true; lcd.setCursor(0,1); lcd.print("Gripper set!    ");
  EEPROM.put (eeAddressEndeffector , endeffectorGripper); EEPROM.put (eeAddressEndefectorAvailable , 999);
  EEPROM.get(eeAddressEndeffector, endeffectorGripper); Serial.print("End effector type stored in eeprom: "); 
  Serial.print(endeffectorGripper); Serial.println("( Vacuum Cup)"); }
  }
  
prevKeyState = 1;  }
break;}

  case btnLEFT:
 {if (prevKeyState == 0){
    menuChoice ++;   
   if (menuChoice == 21) {menuChoice = 1;}
  if (menuChoice == 1) {lcd.setCursor(0,1); lcd.print("Set cal pos  [>]");}
  if (menuChoice == 2) {lcd.setCursor(0,1); lcd.print("Goto cal pos [>]");}
  if (menuChoice == 3) {lcd.setCursor(0,1); lcd.print("Set Z-down   [>]");}
  if (menuChoice == 4) {lcd.setCursor(0,1); lcd.print("Goto Z-down  [>]");}
  if (menuChoice == 5) {lcd.setCursor(0,1); lcd.print("Set Cal A_xy [>]");}
  if (menuChoice == 6) {lcd.setCursor(0,1); lcd.print("Goto Cal A_xy[>]");}
  if (menuChoice == 7) {lcd.setCursor(0,1); lcd.print("Set Cal B_xy [>]");}
  if (menuChoice == 8) {lcd.setCursor(0,1); lcd.print("Goto Cal B_xy[>]");}
  if (menuChoice == 9) {lcd.setCursor(0,1); lcd.print("Set Cal C_xy [>]");}
  if (menuChoice == 10) {lcd.setCursor(0,1); lcd.print("Goto Cal C_xy[>]");}
  if (menuChoice == 11) {lcd.setCursor(0,1); lcd.print("Set Start pos[>]");}
  if (menuChoice == 12) {lcd.setCursor(0,1); lcd.print("Goto Startpos[>]");} 
  if (menuChoice == 13) {lcd.setCursor(0,1); lcd.print("Set DropPos1 [>]");}
  if (menuChoice == 14) {lcd.setCursor(0,1); lcd.print("Goto DropPos1[>]");}
  if (menuChoice == 15) {lcd.setCursor(0,1); lcd.print("Set DropPos2 [>]");}
  if (menuChoice == 16) {lcd.setCursor(0,1); lcd.print("Goto DropPos2[>]");}
  if (menuChoice == 17) {lcd.setCursor(0,1); lcd.print("Set DropPos3 [>]");}
  if (menuChoice == 18) {lcd.setCursor(0,1); lcd.print("Goto DropPos3[>]");}
  if (menuChoice == 19) {lcd.setCursor(0,1); lcd.print("Start Calib. [>]");} 
  if (menuChoice == 20) {lcd.setCursor(0,1); lcd.print("Cup/Gripper  [>]");} 
  
prevKeyState = 1;}
 break;  }
 case btnUP:{
 {}
 break;  }
case btnDOWN:{
 {}
 break; } 
 case btnSELECT:
 {if (prevKeyState == 0){ 
 menuItem = 1; drawMenu = true;
 prevKeyState = 1;}
 break; }
 case btnNONE:
 {
prevKeyState = 0;
  break;
 }
}
}

/*
//section for actions to be performed when buttons (seperate from shield) are pressed
   button1State = digitalRead(button1Pin);
   upButtonState = digitalRead(buttonUpPin);
   downButtonState = digitalRead(buttonDownPin);
   leftButtonState = digitalRead(buttonLeftPin);
   rightButtonState = digitalRead(buttonRightPin);

 if ((button1State != prevButton1State) && (button1State == HIGH))
 {
  int i; 
  static int j = 0; //indicator for which line in pixy array to use, 0 is first line
  // grab blocks!
  pixy.ccc.getBlocks();
pixyPartSignature = pixy.ccc.blocks[i].m_signature;
pixyPartX = pixy.ccc.blocks[j].m_x;
pixyPartY = pixy.ccc.blocks[j].m_y;
partWidth = pixy.ccc.blocks[j].m_width;
partHeight = pixy.ccc.blocks[j].m_height;

  
  // If there are blocks detected, print them!
  if (pixy.ccc.numBlocks)
  {
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
      Serial.print("Part Signature: "); Serial.println(pixyPartSignature);
      Serial.print(" pixyPartX: "); Serial.print(pixyPartX);
      Serial.print(" pixyPartY: "); Serial.print(pixyPartY);
      Serial.print(" partWidth: "); Serial.print(partWidth);
      Serial.print(" partHeight: "); Serial.println(partHeight);
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      pixy.ccc.blocks[i].print();
      //start test move robot arm to x location of found block ****************************************************
      //gPTPCmd.x = partX;
      //moveArm();
      // end test**********************************************************************
    }
  }  
 }

revButton1State = button1State;

if ((upButtonState != prevUpButtonState) && (upButtonState == HIGH))
{
Serial.println("Up button Pressed");
moveArm((gPTPCmd.x += 10), currentY, currentZ, currentR, currentVac);
}
prevUpButtonState = upButtonState;

if ((downButtonState != prevDownButtonState) && (downButtonState == HIGH))
{
Serial.println("Down button Pressed");
moveArm((gPTPCmd.x -= 10), currentY, currentZ, currentR, currentVac);
}
prevDownButtonState = downButtonState;

if ((leftButtonState != prevLeftButtonState) && (leftButtonState == HIGH))
{
Serial.println("Left button Pressed");
moveArm(currentX, (gPTPCmd.y += 10), currentZ, currentR, currentVac);
}
prevLeftButtonState = leftButtonState;

if ((rightButtonState != prevRightButtonState) && (rightButtonState == HIGH))
{
Serial.println("Right button Pressed");
moveArm(currentX, (gPTPCmd.y -= 10), currentZ, currentR, currentVac);
}
prevRightButtonState = rightButtonState;
*/

delay(10);

    }     // end infinite loop
}   
