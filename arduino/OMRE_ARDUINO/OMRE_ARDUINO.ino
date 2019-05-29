
/*****************************************************************************************************************************/
/******************************************        ROBOT SETUP      **********************************************************/
/*****************************************************************************************************************************/
#include <HardwareSerial.h>
#include <SimpleTimer.h>

#include <SharpIR.h>
SharpIR IR_Sensor0(SharpIR::GP2Y0A21YK0F, A0);
SharpIR IR_Sensor1(SharpIR::GP2Y0A21YK0F, A1);
SharpIR IR_Sensor2(SharpIR::GP2Y0A21YK0F, A2);
SharpIR IR_Sensor3(SharpIR::GP2Y0A21YK0F, A3);

#define FORWARD  LOW
#define BACKWARD HIGH

volatile long encoderCounts[]              = {0, 0, 0}; // variables accesed inside of an interrupt need to be volatile

const int encoder_interrupt_pin_0          = 18;
const int encoder_interrupt_pin_1          = 2;         //can change to 19
const int encoder_interrupt_pin_2          = 3;         //can change to 20

const int motorPWMPins[3]                  = {8, 9, 10};
const int motorDirPins[3]                  = {29, 28, 27};

const int ultrasonicSensorTrigPins[]       = {30, 32, 34, 36, 38, 40};
const int ultrasonicSensorEchoPins[]       = {31, 33, 35, 37, 39, 41};
const int infraredSensorPins[]             = {0, 1, 2, 3};

double Kp[] = {0.5, 0.5, 0.5};
double Ki[] = {0.1, 0.1, 0.1};
double Kd[] = {0, 0, 0};

double ITerm[3]               = {0, 0, 0};
double lastInput[3]           = {0, 0, 0};
double error[3]               = {0, 0, 0};
double dInput[3]              = {0, 0, 0};
double rpm_setpoint[3]        = {0, 0, 0};
double pwm_pid[3]             = {0, 0, 0};
int    rpms[3]                = {0, 0, 0};

unsigned long now[3]          = {0, 0, 0};
unsigned long lastTime[3]     = {0, 0, 0};
unsigned long timeChange[3]   = {0, 0, 0};
int SampleTime                = 1000;

int rpmValues[3]              = {0, 0, 0};
double pastEncoderValues[3]   = {0, 0, 0};
unsigned long pastTimes[3]    = {0, 0, 0};// millis() works for up to 50days! we'll need an unsigned long for it

char rcv_buffer[64];  // holds commands recieved
char TXBuffer[64];    // temp storage for large data sent

long duration_US[6] = {0, 0, 0, 0, 0, 0};
long cm_US[6] = {0, 0, 0, 0, 0, 0};

int  cm_IR[4] = {0,0,0,0};

void motor(int, int);

double changeInEncoders[3]    = {0, 0, 0};
double changeInRevolutions[3] = {0, 0, 0};
double changeInTimeSeconds[3] = {0, 0, 0};
 
char pidSwitch          = '1';
char  ultrasonicSwitch  = '0';
char  IRSwitch          = '0';
char printSwitch        = '0';

/*****************************************************************************************************************************/
/*******************************************************         PRINT DATA SETUP        **************************************************************/
/*****************************************************************************************************************************/
#define PRINT_ULTRASOUND       0
#define PRINT_IR               1   
#define PRINT_RPM              0
/*****************************************************************************************************************************/


/*****************************************************************************************************************************/
/*******************************************************         MAIN  SETUP             *********************************************/
/*****************************************************************************************************************************/
void setup() {
  Serial.begin(115200);

  // UltraSound
  for (int i = 0; i < 6; i++)
  {
    pinMode(ultrasonicSensorTrigPins[i], OUTPUT);
    pinMode(ultrasonicSensorEchoPins[i], INPUT);
  }

  // Motor
  for (int i = 0; i < 3; i++)
  {
    pinMode(motorPWMPins[i], OUTPUT);
    pinMode(motorDirPins[i], OUTPUT); //LOW=CCW HIGH=CW
  }

  pinMode(18, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  buffer_Flush(rcv_buffer);

  while (! Serial);

  attachInterrupt(digitalPinToInterrupt(encoder_interrupt_pin_0), encoder0_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_interrupt_pin_1), encoder1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_interrupt_pin_2), encoder2_ISR, CHANGE);
}
/*****************************************************************************************************************************/


/***************************************************************************************************************************************/
/********************************************************       MAIN LOOP             ********************************************************/
/*****************************************************************************************************************************/
void loop()
{
  /////////////////////////////////////    RPM
  updateRPM();
  /////////////////////////////////////    Determines if we have any serial commands and interpruts them
  receiveBytes();
  /////////////////////////////////////    PID for Speed
  if (pidSwitch == '1')
  {
    speed_pid();
  }
  ////////////////////////////////////     Ultrasound
  if (ultrasonicSwitch == '1')
  {
    UsSensor();
  }
  ////////////////////////////////////     IR
  if (IRSwitch == '1')
  {
    irSensor();
  }
  ////////////////////////////////////     Printing
  if (printSwitch == '1')
  {
    printdata(); //print data IMU
  }
}
/***************************************************************************************************************************************/
