/******************************   ADA_IMU Library   ********************************************/
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



float accel_x;
float accel_filter_x;
float gyro_x;
float last_accel_x;
int countx   = 0;

double alpha = 0.02;
double prev_accel;

double vx;
unsigned long present       = 0;
unsigned long last          = 0;

float filter_accel;
float dt = 0.5;


void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

/*************************************************************    ROBOT SETUP  ************************/

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

//double Kp = 1.9;
//double Ki = 0.09;
//double Kd = 0.2;

double Kp[] = {0.5, 0.5, 0.5};
double Ki[] = {0.1, 0.1, 0.1};
double Kd[] = {0.7, 0.7, 0.7};

double ITerm[3]       = {0, 0, 0};
double lastInput[3]   = {0, 0, 0};
double error[3]       = {0, 0, 0};
double dInput[3]      = {0, 0, 0};
double rpm_setpoint[3]    = {0, 0, 0};
double pwm_pid[3]     = {0, 0, 0};
int rpms[3]           = {0, 0, 0};

unsigned long now[3]        = {0, 0, 0};
unsigned long lastTime[3]   = {0, 0, 0};
unsigned long timeChange[3] = {0, 0, 0};
int SampleTime              = 1000;

int rpmValues[3]             = {0, 0, 0};
double pastEncoderValues[3]  = {0, 0, 0};
unsigned long pastTimes[3]   = {0, 0, 0};// millis() works for up to 50days! we'll need an unsigned long for it

char rcv_buffer[64];  // holds commands recieved
char TXBuffer[64];    // temp storage for large data sent

void motor(int, int);

double changeInEncoders[3]    = {0, 0, 0};
double changeInRevolutions[3] = {0, 0, 0};
double changeInTimeSeconds[3] = {0, 0, 0};

char pidSwitch = '1';

/******************************** PRINT DATA SETUP *************************************************/
int print_setup = 0;
#define PRINT_IMU  0
#define PRINT_VELO  1
/***************************************************************************************************/

/*************************************************************          MAIN  SETUP FUNCTION    *********************************************/  
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 6; i++)
  {
    pinMode(ultrasonicSensorTrigPins[i], OUTPUT);
    pinMode(ultrasonicSensorEchoPins[i], INPUT);
  }

  //INFRARED SENSORS
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
  while (!Serial)
  {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  Serial.println("LSM9DS1 data read demo");

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();

}
/***************************************************************************************************************************************/

/*****************************************************************   MAIN LOOP  ********************************************************/
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
  //    Serial.print(micros() * 0.000001);
  //    Serial.print("  ,  ");
  //    Serial.print(rpmValues[0]);
  //    Serial.print("  ,  ");
  //    Serial.print(rpmValues[1]);
  //    Serial.print("  ,  ");
  //    Serial.println(rpmValues[2]);

  /*********************************************************************  ADA_IMU Loop ****************************/
  lsm.read(); //read Ada_IMU
  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  if (print_setup == 1)
  {
    printdata(); //print data IMU
  }

  updatePos();

}
/***************************************************************************************************************************************/

/**********************************************************************  Update Pos_IMU ************************************************/
void updatePos()
{
  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  // alpha = 0:1;
  accel_x = a.acceleration.x;
  accel_filter_x = alpha *  accel_x + (1 - alpha) * last_accel_x;

  if (( accel_filter_x  <= 0.01) && ( accel_filter_x >= -0.01)) //Discrimination window applied
  {
    accel_filter_x = 0; // to the X axis acceleration
  }

  present = micros() * 0.000001;
  dt = (present - last);

  vx += accel_filter_x * dt;

  if (accel_filter_x  == 0) //we count the number of acceleration samples that equals zero
  {
    countx++;
  }
  else
  {
    countx = 0;
  }

  if (countx >= 25) //if this number exceeds 25, we can assume that velocity is zero
  {
    vx = 0;
    countx = 0;
  }

  last = present;
  accel_x = last_accel_x;

  //  Serial.println(countx);
  //  Serial.print(a.acceleration.x);
  //  Serial.print("   ,   ");
  //  Serial.print(accel_filter_x);
  //  Serial.print("   ,   ");
  //  Serial.print(vx);
  //  Serial.println();
  //  delay(20);
}

void comp_filter(float new_accel, float new_gyro)
{
  float filterTerm0;
  float filterTerm1;
  float filterTerm2;
  float timeConstant;

  timeConstant = 0.5; // default 1.0

  filterTerm0 = (new_accel - prev_accel) * timeConstant * timeConstant;
  filterTerm2 += filterTerm0 * dt;
  filterTerm1 = filterTerm2 + ((new_accel - prev_accel) * 2 * timeConstant) + new_gyro;
  filter_accel = (filterTerm1 * dt) + prev_accel;

  filter_accel = prev_accel;
}
/***************************************************************************************************************************************/

/***********************************************************************    Update RPM    ********************************************/
void updateRPM()
{
  for ( int i = 0; i < 3; i++)
  {
    changeInEncoders[i] = encoderCounts[i] - pastEncoderValues[i];
    changeInTimeSeconds[i] = ((micros() - pastTimes[i]) * 0.000001); // *.001 to convert to seconds
    changeInRevolutions[i] = changeInEncoders[i] / 2248.6;

    rpmValues[i] = (changeInRevolutions[i] / (changeInTimeSeconds[i])) * 60; // *60 to get Revolutions per MINUTE

    // update our values to be used next time around
    pastTimes[i] = micros();
    pastEncoderValues[i] = encoderCounts[i];
    //    Serial.print(micros() * 0.000001);
    //    Serial.print("  ,  ");
    //    Serial.print(rpmValues[0]);
    //    Serial.print("  ,  ");
    //    Serial.print(rpmValues[1]);
    //    Serial.print("  ,  ");
    //    Serial.println(rpmValues[2]);

  }
}
/***************************************************************************************************************************************/

/*********************************************************   PID for Motor    ****************************************************************/      
void speed_pid()
{
  for (int i = 0; i < 3; i++)
  {
    if (rpm_setpoint[i])
    {
      //updateRPM();
      now[i] = micros();
      timeChange[i] = (now[i] - lastTime[i]);

      if (timeChange[i] >= SampleTime)
      {

        ///////////////////////////////////////////        Error Variables
        error[i] = rpm_setpoint[i] - rpmValues[i];
        ITerm[i] += (Ki[i] * error[i]);

        if (ITerm[i] > 255)
        {
          ITerm[i] = 255;
        }
        if (ITerm[i] < -255)
        {
          ITerm[i] = -255;
        }

        dInput[i] = (rpmValues[i] - lastInput[i]);

        ////////////////////////////////////////////       PID output
        pwm_pid[i] = (Kp[i] * error[i]) + ITerm[i] - (Kd[i] * dInput[i]);

        if (pwm_pid[i] > 255)
        {
          pwm_pid[i] = 255;
        }
        if (pwm_pid[i] < -255)
        {
          pwm_pid[i] = -255;
        }

        ///////////////////////////////////////////        Run Motor
        motor(i, pwm_pid[i]);

        //////////////////////////////////////////         Remember Variables for next time
        lastInput[i] = rpmValues[i];
        lastTime[i] = now[i];

        //        Serial.print(rpmValues[0]);
        //        Serial.print("  ,  ");
        //        Serial.print(rpmValues[1]);
        //        Serial.print("  ,  ");
        //        Serial.println(rpmValues[2]);

        delay(2);
      }
    }
    if (rpm_setpoint[i] == 0)
    {
      motor(i, 0);
    }
  }
}
/***************************************************************************************************************************************/

////////////////////////////////////////////////////////////            COUNTING ENCODERS
void encoder0_ISR() // encoder0 interrupt service routine
{
  noInterrupts();
  if (digitalRead(motorDirPins[0]) == LOW)
    encoderCounts[0]++;
  else
    encoderCounts[0]--;
  interrupts();
  //  Serial.println(encoderCounts[0]);
}
void encoder1_ISR()
{
  noInterrupts();
  if (digitalRead(motorDirPins[1]) == LOW)
    encoderCounts[1]++;
  else
    encoderCounts[1]--;
  interrupts();
}
void encoder2_ISR()
{
  if (digitalRead(motorDirPins[2]) == LOW)
    encoderCounts[2]++;
  else
    encoderCounts[2]--;
  //  Serial.println(encoderCounts[2]);
}

/////////////////////////////////////////////////////////////////             RUNNING MOTORS
void motor(int motorNumber, int pwm)
{
  if (pwm > 255) {
    pwm = 255;
  }
  if (pwm < -255) {
    pwm = -255;
  }

  if (pwm > 0)
  {
    //    if (mark_start >= 0) {
    //      digitalWrite(motorDirPins[motorNumber], FORWARD);
    //      analogWrite(motorPWMPins[motorNumber], pwm + 150);
    //      mark_start -= 1;
    //    }
    digitalWrite(motorDirPins[motorNumber], FORWARD);
    analogWrite(motorPWMPins[motorNumber], pwm);
  }

  if (pwm < 0)
  {
    //    if (mark_start >= 0) {
    //      digitalWrite(motorDirPins[motorNumber], BACKWARD);
    //      analogWrite(motorPWMPins[motorNumber], -pwm - 150);
    //      mark_start -= 1;
    //    }
    digitalWrite(motorDirPins[motorNumber], BACKWARD);
    analogWrite(motorPWMPins[motorNumber], -pwm);
  }

  if (pwm == 0)
  {
    digitalWrite(motorDirPins[motorNumber], BACKWARD);
    analogWrite(motorPWMPins[motorNumber], pwm);
  }

}

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

///////////////////////////////////////////////////////////////////      BUFFER FLUSH
void buffer_Flush(char *ptr)
{
  for (int i = 0; i < 64; i++)
  {
    ptr[i] = 0;
  }
}

/////////////////////////////////////////////////////////////////        GIVING COMMAND
void parseCommand()
{
  char command = rcv_buffer[0]; // our first byte tells us the command char is equivalent to byte
  switch (command)
  {
    ///////////////////////////////////////////////////////////////      ENCODER
    case 'E':
    case 'e':
      int encoderNum;
      sscanf(&rcv_buffer[1], " %d \r", &encoderNum);
      long counts;
      counts = encoderCounts[encoderNum];
      //itoa(encoderCounts[encoderNum],TXBuffer,10);   // serial.print can not handle printing a 64bit int so we turn it  into a string
      Serial.println(counts);
      break;

    ////////////////////////////////////////////////////////////////    MOTOR
    case 'M':
    case 'm':
      int  motorNumber;
      int  motorPWM;
      int  motorDirection;
      sscanf(&rcv_buffer[1], " %d %d \r", &motorNumber, &motorPWM);
      motor(motorNumber, motorPWM);
      break;

    ////////////////////////////////////////////////////////////////    ULTRASOUND
    case 'u':
    case 'U':
      int ultrasonicNumber;
      long duration, cm, start;
      //duration = -60;
      sscanf(&rcv_buffer[1], " %d \r", &ultrasonicNumber);
      digitalWrite(ultrasonicSensorTrigPins[ultrasonicNumber], LOW);
      //delayMicroseconds(5);
      digitalWrite(ultrasonicSensorTrigPins[ultrasonicNumber], HIGH);
      delayMicroseconds(10);
      digitalWrite(ultrasonicSensorTrigPins[ultrasonicNumber], LOW);
      start = micros();
      while (digitalRead(ultrasonicSensorEchoPins[ultrasonicNumber]) == LOW);
      start = micros();

      while (micros() - start <= 6000)
      {
        if (digitalRead(ultrasonicSensorEchoPins[ultrasonicNumber]) == LOW)
        {
          duration = micros() - start;
          break;
        }
        //duration = micros()-start;
      }
      //duration = pulseIn(ultrasonicSensorEchoPins[ultrasonicNumber], HIGH);
      cm = (duration / 2) / 29.1;
      //Serial.println(duration);
      //inches = (duration/2) /
      Serial.println(cm);
      break;

    ////////////////////////////////////  INFARED
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

    ////////////////////////////////////  ENTER RPM_GOAL
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
      print_setup = 1;
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
