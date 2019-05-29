/************************************************************   ADA_IMU Library   ********************************************/
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

/*************************************************************     ROBOT SETUP      **********************************************************/
#include <HardwareSerial.h>
#include <SimpleTimer.h>
#define FORWARD LOW
#define BACKWARD HIGH
#define INFRARED_SENSOR_0 A0
#define INFRARED_SENSOR_1 A1
#define INFRARED_SENSOR_2 A2
#define INFRARED_SENSOR_3 A3

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

long duration, cm, start;

void motor(int, int);
void ultrasound_read(int);

double changeInEncoders[3]    = {0, 0, 0};
double changeInRevolutions[3] = {0, 0, 0};
double changeInTimeSeconds[3] = {0, 0, 0};

char pidSwitch = '1';


/**********************************************   Pos_IMU Variables ************************************************************/
float accel_x;
float accel_y;
float accel_z;
float accel_filter_x;
float accel_filter_y;
float accel_filter_z;

float gyro_x;
float last_accel_x;
float last_accel_y;
float last_accel_z;

int countx   = 0;
int county   = 0;
int countz   = 0;
int countv   = 0;

double alpha = 0.1;
double prev_accel;

float vx;
float vy;
float vz;
float v;

float pos_x;
float pos_y;
float pos_z;
float pos;

unsigned long present       = 0;
unsigned long last          = 0;

float filter_accel;
float dt = 0.5;



/********************************         PRINT DATA SETUP        **************************************************************/
char print_setup = '0';
#define PRINT_IMU        1
#define PRINT_VELO_IMU   0
#define PRINT_RPM        0


/*****************************************************************************************************************************/

/*************************************************************      MAIN  SETUP   *********************************************/
void setup() {
  Serial.begin(115200);

  // UltraSound
  for (int i = 0; i < 6; i++)
  {
    pinMode(ultrasonicSensorTrigPins[i], OUTPUT);
    pinMode(ultrasonicSensorEchoPins[i], INPUT);
  }

  // INFRARED SENSORS
  pinMode(INFRARED_SENSOR_0, INPUT);
  pinMode(INFRARED_SENSOR_1, INPUT);
  pinMode(INFRARED_SENSOR_2, INPUT);
  pinMode(INFRARED_SENSOR_3, INPUT);

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
  /******************************************** ADA_IMU Setup ***************************************************/
  {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  // helper to just set the default scaling we want, see above!
  setupSensor();

}
/***************************************************************************************************************************************/

/**********************************************       MAIN LOOP             ********************************************************/
void loop()
{
  updateRPM();
  // determines if we have any serial commands and interpruts them
  receiveBytes();
  // proportional integral controller
  if (pidSwitch == '1')
  {
    speed_pid();
  }


  /*******************************************        ADA_IMU Loop             ****************************/
  lsm.read(); //read Ada_IMU
  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  if (print_setup == '1')
  {
    printdata(); //print data IMU
  }
  updatePos();

  //  Serial.print(micros() * 0.000001);
  //  Serial.print("  ,  ");
  //  Serial.print(encoderCounts[0]);
  //  Serial.print("  ,  ");
  //  Serial.print(encoderCounts[1]);
  //  Serial.print("  ,  ");
  //  Serial.println(encoderCounts[2]);
}
/***************************************************************************************************************************************/


///////////////////////////////////////////////////////////////////             RECEIVE BYTE
void receiveBytes()
{
  static byte index = 0;
  char terminator = '\r'; // what tells us our command is done
  while (Serial.available() > 0)
  {
    rcv_buffer[index] = Serial.read(); // read in our serial commands
    if (rcv_buffer[index] == terminator) // main loop for processing our command
    {
      index = 0;
      parseCommand();
      buffer_Flush(rcv_buffer);
    }
    else
    {
      index++;
      if (index >= 64)
      {
        Serial.println("buffer overflow");
        index = 0;
        buffer_Flush(rcv_buffer);
      }
    }
  }
}

///////////////////////////////////////////////////////////////////            BUFFER FLUSH
void buffer_Flush(char *ptr)
{
  for (int i = 0; i < 64; i++)
  {
    ptr[i] = 0;
  }
}

/////////////////////////////////////////////////////////////////               GIVING COMMAND
void parseCommand()
{
  char command = rcv_buffer[0]; // our first byte tells us the command char is equivalent to byte
  switch (command)
  {
    ///////////////////////////////////////////////////////////////             ENCODER
    case 'E':
    case 'e':
      int encoderNum;
      sscanf(&rcv_buffer[1], " %d \r", &encoderNum);
      float counts;
      counts = encoderCounts[encoderNum];
      //itoa(encoderCounts[encoderNum],TXBuffer,10);
      // serial.print can not handle printing a 64bit int so we turn it  into a string
      Serial.println(encoderCounts[encoderNum]);
      break;

    ///////////////////////////////////////////////////////////////             POSITION IMU
    case 'x':
    case 'X':
      Serial.println(pos_x);
      break;


    ////////////////////////////////////////////////////////////////             MOTOR
    case 'M':
    case 'm':
      int  motorNumber;
      int  motorPWM;
      int  motorDirection;
      sscanf(&rcv_buffer[1], " %d %d \r", &motorNumber, &motorPWM);
      motor(motorNumber, motorPWM);
      break;

    ////////////////////////////////////////////////////////////////             ULTRASOUND
    case 'u':
    case 'U':
      int ultrasonicNumber;
      sscanf(&rcv_buffer[1], " %d \r", &ultrasonicNumber);

      digitalWrite(ultrasonicSensorTrigPins[ultrasonicNumber], LOW);
      delayMicroseconds(2);
      digitalWrite(ultrasonicSensorTrigPins[ultrasonicNumber], HIGH);
      delayMicroseconds(10);
      digitalWrite(ultrasonicSensorTrigPins[ultrasonicNumber], LOW);

      //      start = micros();
      //      while (digitalRead(ultrasonicSensorEchoPins[ultrasonicNumber]) == LOW);
      //      start = micros();
      //
      //      while (micros() - start <= 6000)
      //      {
      //        if (digitalRead(ultrasonicSensorEchoPins[ultrasonicNumber]) == LOW)
      //        {
      //          duration = micros() - start;
      //          break;
      //        }
      //duration = micros()-start;

      duration = pulseIn(ultrasonicSensorEchoPins[ultrasonicNumber], HIGH);
      cm = (duration*0.034) / 2;
  
      //Serial.println(duration);
      //inches = (duration/2) /
      Serial.println(cm);

      break;

    ////////////////////////////////////                                  INFARED
    case 'i':
    case 'I':
      uint16_t value;
      int infraredNumber;
      double distance;

      sscanf(&rcv_buffer[1], " %d \r", &infraredNumber);
      value = analogRead(infraredNumber);
      distance = get_IR(value);
      Serial.println (distance);
      break;

    ////////////////////////////////////                                   ENTER RPM_GOAL
    case 'v':
    case 'V':

      int rpm0;
      int rpm1;
      int rpm2;
      sscanf(&rcv_buffer[1], "%d %d %d \r", &rpm0, &rpm1, &rpm2);

      rpm_setpoint[0] = rpm0;
      rpm_setpoint[1] = rpm1;
      rpm_setpoint[2] = rpm2;
      break;

    ////////////////////////////////////////////////////////////////////  PID Switch ON/OFF
    case 'p':
    case 'P':
      rpm_setpoint[0] = 0;
      rpm_setpoint[1] = 0;
      rpm_setpoint[2] = 0;
      motor(0, 0);
      motor(1, 0);
      motor(2, 0);
      sscanf(&rcv_buffer[1], " %c \r", &pidSwitch);
      break;

    /////////////////////////////////////////////////////////////////   Check  RPM
    case 'r':
    case 'R':
      int rpmNum;
      sscanf(&rcv_buffer[1], " %d \r", &rpmNum);
      printDouble(rpmValues[rpmNum], 1000000000);
      break;

    ///////////////////////////////////////////////////////////////    ENTER GAINS K
    case 'k':
    case 'K':
      char  pValue[20];
      char  iValue[20];
      sscanf(&rcv_buffer[1], " %s %s \r", &pValue, &iValue);
      char *ptr;
      for (int i = 0; i < 3; i++)
      {
        Kp[i] = strtod(pValue, &ptr);
        Ki[i] = strtod(iValue, &ptr);
      }
      break;

    ///////////////////////////////////////////////////////////////////  STOP
    case 's':
    case 'S':
      print_setup = 0;
      for (int i = 0; i < 3; i++)
      {
        motor(i, 0);
        rpm_setpoint[i] = 0;
      }
      break;

    //////////////////////////////////////////////////////////////////     PRINT DATA
    case 'a':
    case 'A':
      sscanf(&rcv_buffer[1], " %c \r", &print_setup);
      break;

    /////////////////////////////////////////////////////////////////      Move Robot Forward
    case 'f':
    case 'F':

      rpm_setpoint[0] = 0;
      rpm_setpoint[1] = -50;
      rpm_setpoint[2] = 50;
      break;

    /////////////////////////////////////////////////////////////////      Move Robot Backward
    case 'b':
    case 'B':

      rpm_setpoint[0] = 0;
      rpm_setpoint[1] = 50;
      rpm_setpoint[2] = -50;
      break;
      //    default:
      //      Serial.println("Error: Serial input incorrect");
  }
}

double get_IR(uint16_t value) {
  if (value < 16)  value = 16;
  //return 4800.0 / (value - 1120.0);
  return 4800.0 / (value - 20.0);
}

//supporting function to print doubles precicely
void printDouble( double val, unsigned int precision) {
  // prints val with number of decimal places determine by precision
  // NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
  // example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

  Serial.print (int(val));  //prints the int part
  Serial.print("."); // print the decimal point
  unsigned int frac;
  if (val >= 0)
    frac = (val - int(val)) * precision;
  else
    frac = (int(val) - val ) * precision;
  Serial.println(frac, DEC) ;
}
