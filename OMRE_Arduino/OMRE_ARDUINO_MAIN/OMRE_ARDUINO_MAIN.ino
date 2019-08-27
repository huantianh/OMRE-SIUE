/******************************************        ROBOT SETUP      **********************************************************/
#include <SharpIR.h>
SharpIR IR_Sensor0(SharpIR::GP2Y0A21YK0F, A0);  // Infrared setup
SharpIR IR_Sensor1(SharpIR::GP2Y0A21YK0F, A1);
SharpIR IR_Sensor2(SharpIR::GP2Y0A21YK0F, A2);
SharpIR IR_Sensor3(SharpIR::GP2Y0A21YK0F, A3);

#include <Wire.h>                               // IMU setup
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#include <HardwareSerial.h>
#include <SimpleTimer.h>
#define FORWARD  HIGH
#define BACKWARD LOW

volatile long encoderCounts[]              = {0, 0, 0}; // variables accesed inside of an interrupt need to be volatile
const int encoder_interrupt_pin_0          = 18;
const int encoder_interrupt_pin_1          = 2;         //can change to 19
const int encoder_interrupt_pin_2          = 3;         //can change to 20

const int motorPWMPins[3]                  = {8, 9, 10};
const int motorDirPins[3]                  = {29, 28, 27};

const int ultrasonicSensorTrigPins[]       = {30, 32, 34, 36, 38, 40};
const int ultrasonicSensorEchoPins[]       = {31, 33, 35, 37, 39, 41};
const int infraredSensorPins[]             = {0, 1, 2, 3};

double rpm_setpoint[3]                     = {0, 0, 0};
int rpmValues[3]                           = {0, 0, 0};

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
char ultrasonicSwitch                      = '0';
char IRSwitch                              = '0';
char IMUSwitch                             = '0';
char printSwitch                           = '0';

/*****************************************        PRINT DATA SETUP        **************************************************************/
char PRINT_ULTRASOUND                      = '0';
char PRINT_IR                              = '0';
char PRINT_RPM                             = '0';
char PRINT_IMU                             = '0';
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
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  buffer_Flush(rcv_buffer);

  while (!Serial);

  attachInterrupt(digitalPinToInterrupt(encoder_interrupt_pin_0), encoder0_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_interrupt_pin_1), encoder1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_interrupt_pin_2), encoder2_ISR, CHANGE);

  //IMU
  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    while (1);
  }
  // helper to just set the default scaling we want, see above!
  setupIMU();

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
  /////////////////////////////////////    IMU
  if (IMUSwitch == '1')
  {
    updatePos();
    lsm.read();
    /* Get a new sensor event */
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);

    accel_x = a.acceleration.x;
    accel_y = a.acceleration.y;
    accel_z = a.acceleration.z;
  }
  /////////////////////////////////////    Printing
  if (printSwitch == '1')
  {
    printdata(); //print data IMU
  }
}
/***************************************************************************************************************************************/
