/******************************************        ROBOT SETUP      **********************************************************/
#include <SharpIR.h>
SharpIR IR_Sensor0(SharpIR::GP2Y0A21YK0F, A0);  // Infrared setup
SharpIR IR_Sensor1(SharpIR::GP2Y0A21YK0F, A1);
SharpIR IR_Sensor2(SharpIR::GP2Y0A21YK0F, A2);
SharpIR IR_Sensor3(SharpIR::GP2Y0A21YK0F, A3);

#include <HardwareSerial.h>
#include <SimpleTimer.h>
#define FORWARD  HIGH
#define BACKWARD LOW

volatile long encoderCounts[]              = {0, 0, 0}; // variables accesed inside of an interrupt need to be volatile
const int encoder_interrupt_pin_0          = 18;
const int encoder_interrupt_pin_1          = 19;         //can change to 19,2
const int encoder_interrupt_pin_2          = 20;         //can change to 20,3

const int motorPWMPins[3]                  = {8, 9, 10};
const int motorDirPins[3]                  = {29, 28, 27};

const int motorCurrentPins[3] = {A8, A9, A10};
///////////////////////////////////////////////////////////////////// Voltage Sensor
float del = 0.5;
float tau = 2;
float alpha = del/tau;

const int motorVolPins[3] = {A12, A13, A14};
const int avgSamples = 50;
float vOUT[3] = {0.0, 0.0, 0.0};
float m_vol[3] = {0.0, 0.0, 0.0};
float R1 = 30000.0;
float R2 = 7500.0;
int sensorValue[3] = {0, 0, 0};
int last_sensorValue[3] = {0, 0, 0};
///////////////////////////////////////////////////////////////////// Current Sensor
const int CurrentPin = A12;
const int c_avgSamples = 30;
int c_sensorValue = 0;
float sensitivity = -0.034 / 60;                    //45
float Vref = 2580;
float cur_s = 0.0;
float c_voltage = 0.0;
float last_c_sensorValue = 0;
/////////////////////////////////////////////////////////////////////

const int ultrasonicSensorTrigPins[]       = {30, 32, 34, 36, 38, 40};
const int ultrasonicSensorEchoPins[]       = {31, 33, 35, 37, 39, 41};
const int infraredSensorPins[]             = {0, 1, 2, 3};

double rpm[3]                              = {0, 0, 0};
double rpm_setpoint[3]                     = {0, 0, 0};
int    rpmValues[3]                        = {0, 0, 0};
float    m_cur[3]                          = {0, 0, 0};
int    vol_sen                             = 0;

double cur[3]                              = {0, 0, 0};
double cur_setpoint[3]                     = {0, 0, 0};

double duration_US[6]                      = {0, 0, 0, 0, 0, 0};
double m_US[6]                             = {0, 0, 0, 0, 0, 0};
double m_IR[4]                             = {0, 0, 0, 0};

float accel_x = 0;
float accel_y = 0;
float accel_z = 0;
float accel_filter_x;
float accel_filter_y;
float accel_filter_z;
float vx;
float vy;
float vz;
float v;
float pos_x;
float pos_y;
float pos_z;
float pos;

char rcv_buffer[64];  // holds commands recieved
//char TXBuffer[64];    // temp storage for large data sent

char pidSwitch                             = '0';
char pidCurSwitch                          = '0';
char ultrasonicSwitch                      = '0';
char IRSwitch                              = '0';
char MCURSwitch                            = '0';
char MVOLSwitch                            = '0';
char CURSSwitch                            = '0';
char printSwitch                           = '0';
char pwm_dir[3]                            = {'0', '0', '0'};

/*****************************************        PRINT DATA SETUP        **************************************************************/
char PRINT_ULTRASOUND                      = '0';
char PRINT_IR                              = '0';
char PRINT_RPM                             = '0';
char PRINT_MCUR                            = '0';
char PRINT_MVOL                            = '0';
char PRINT_CURS                            = '0';
char PRINT_DYNM                            = '0';
/*****************************************         MAIN  SETUP             *********************************************/
void setup()
{
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
  pinMode(19, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);

  buffer_Flush(rcv_buffer);

  while (!Serial);

  attachInterrupt(digitalPinToInterrupt(encoder_interrupt_pin_0), encoder0_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_interrupt_pin_1), encoder1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_interrupt_pin_2), encoder2_ISR, CHANGE);

}

/*****************************************            MAIN LOOP             ********************************************************/
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
  /////////////////////////////////////    Ultrasound
  if (ultrasonicSwitch == '1')
  {
    UsSensor();
  }
  /////////////////////////////////////    IR
  if (IRSwitch == '1')
  {
    irSensor();
  }
  /////////////////////////////////////    Printing
  if (printSwitch == '1')
  {
    printdata(); //print data IMU
  }
  /////////////////////////////////////    Current on board
  if (MCURSwitch == '1')
  {
    m_current();
  }
  /////////////////////////////////////    Voltage
  if (MVOLSwitch == '1')
  {
    m_voltage();
  }
  /////////////////////////////////////    Current Sensor
  if (CURSSwitch == '1')
  {
    current_sensor();
  }
  /////////////////////////////////////    PID for current
  if (pidCurSwitch == '1')
  {
    cur_pid();
  }
}
/***************************************************************************************************************************************/
