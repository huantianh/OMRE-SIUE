#include <HardwareSerial.h>
#include <SimpleTimer.h>
#include <Wire.h>

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
double Kd[] = {0, 0, 0};

double ITerm[3]       = {0, 0, 0};
double lastInput[3]   = {0, 0, 0};
double error[3]       = {0, 0, 0};
double dInput[3]      = {0, 0, 0};
double setpoint[3]    = {0, 0, 0};
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

/***************************************************************
  MINIMU define and variables
 **************************************************************/

int IMUsetup = 0;

// Uncomment the following line to use a MinIMU-9 v5 or AltIMU-10 v5. Leave commented for older IMUs (up through v4).
#define IMU_V5

// Uncomment the below line to use this axis definition:
// X axis pointing forward
// Y axis pointing to the right
// and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1, 1, 1, -1, -1, -1, 1, 1, 1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition:
// X axis pointing forward
// Y axis pointing to the left
// and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

#include <Wire.h>

// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256

#define GRAVITY 10//this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board

#define M_X_MIN -1000
#define M_Y_MIN -1000
#define M_Z_MIN -1000
#define M_X_MAX +1000
#define M_Y_MAX +1000
#define M_Z_MAX +1000

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 1     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 0   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13

float G_Dt = 0.02;  // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer = 0; //general purpuse timer
long timer_old;
long timer24 = 0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6] = {0, 0, 0, 0, 0, 0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3] = {0, 0, 0}; //Store the acceleration in a vector
float Gyro_Vector[3] = {0, 0, 0}; //Store the gyros turn rate in a vector
float Omega_Vector[3] = {0, 0, 0}; //Corrected Gyro_Vector data
float Omega_P[3] = {0, 0, 0}; //Omega Proportional correction
float Omega_I[3] = {0, 0, 0}; //Omega Integrator
float Omega[3] = {0, 0, 0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};

unsigned int counter = 0;
byte gyro_sat = 0;

float DCM_Matrix[3][3] = {
  {
    1, 0, 0
  }
  , {
    0, 1, 0
  }
  , {
    0, 0, 1
  }
};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}}; //Gyros here


float Temporary_Matrix[3][3] = {
  {
    0, 0, 0
  }
  , {
    0, 0, 0
  }
  , {
    0, 0, 0
  }
};

unsigned char  Sensor_Data[8];
unsigned char  Sample_X;
unsigned char  Sample_Y;
unsigned char  Sample_Z;
unsigned char countx, county ;
signed int accelerationx[2] = {0, 0};
signed int accelerationy[2] = {0, 0};
signed int velocityx[2] = {0, 0};
signed int velocityy[2] = {0, 0};
signed int positionX[2];
signed int positionY[2];
signed int positionZ[2];
unsigned char direction1;
unsigned long sstatex, sstatey;



void calibrate(void);
void data_transfer(void);
void data_reintegration(void);
void movement_end_check(void);
void updatePos(void);

double changeTimeSeconds;
double prevTimes;

int velo_x[2] = {0, 0};
int velo_y[2] = {0, 0};
int pos_x[2]  = {0, 0};
int pos_y[2]  = {0, 0};


/**************************************************************
                                                                                 SET UP
 **************************************************************/

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

  /**********************************************************************************     MINIMU SETUP
  *************************************************************************************/

  pinMode (STATUS_LED, OUTPUT); // Status LED
  I2C_Init();
  Serial.println("Pololu MinIMU-9 + Arduino AHRS");
  digitalWrite(STATUS_LED, LOW);
  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();

  delay(20);

  for (int i = 0; i < 32; i++) // We take some readings...
  {
    Read_Gyro();
    Read_Accel();
    for (int y = 0; y < 6; y++) // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
  }

  for (int y = 0; y < 6; y++)
    AN_OFFSET[y] = AN_OFFSET[y] / 32;
  AN_OFFSET[5] -= GRAVITY * SENSOR_SIGN[5];

  //Serial.println("Offset:");
  for (int y = 0; y < 6; y++)
    Serial.println(AN_OFFSET[y]);

  delay(2000);
  digitalWrite(STATUS_LED, HIGH);

  timer = millis();
  delay(20);
  counter = 0;
  Serial.print("Finished Calibrating for IMU!");
}
/*******************************************************************************************************/


//////////////////////////////////////////////////////////////////    LOOP
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

  //  Serial.print(micros() * 0.000001);
  //  Serial.print("  ,  ");
  //  Serial.print(rpmValues[0]);
  //  Serial.print("  ,  ");
  //  Serial.print(rpmValues[1]);
  //  Serial.print("  ,  ");
  //  Serial.println(rpmValues[2]);

  /**************************************************************************      MINIMU  LOOP
   **************************************************************************/

  if ((millis() - timer) >= 20) // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer = millis();
    if (timer > timer_old)
    {
      G_Dt = (timer - timer_old) / 1000.0; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
      if (G_Dt > 0.2)
        G_Dt = 0; // ignore integration times over 200 ms
    }
    else
      G_Dt = 0;

    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer

    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
    {
      counter = 0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading
    }

    // Calculations...
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
    // ***
    if (IMUsetup == 1)
    {
      printdata();
    }
  }
  updatePos();
}

/*****************************************************

                       Function

 ****************************************************/

void Calibrate(void)
{
  unsigned int count1;
  count1 = 0;
  do {
    Read_Accel();
    sstatex = sstatex + Sample_X; // Accumulate Samples
    sstatey = sstatey + Sample_Y;
    count1++;
  } while (count1 != 0x0400); // 1024 times
  sstatex = sstatex >> 10; // division between 1024
  sstatey = sstatey >> 10;
}

void data_transfer(void)
{
  signed long positionXbkp;
  signed long positionYbkp;
  unsigned int delay;
  unsigned char posx_seg[4], posy_seg[4];
  if (positionX[1] >= 0) { //This line compares the sign of the X direction data
    direction1 = (direction1 | 0x10); //if its positive the most significant byte
    posx_seg[0] = positionX[1] & 0x000000FF; // is set to 1 else it is set to 8
    posx_seg[1] = (positionX[1] >> 8) & 0x000000FF; // the data is also managed in the
    // subsequent lines in order to
    posx_seg[2] = (positionX[1] >> 16) & 0x000000FF; // be sent. The 32 bit variable must be
    posx_seg[3] = (positionX[1] >> 24) & 0x000000FF; // split into 4 different 8 bit
    // variables in order to be sent via
    // the 8 bit SCI frame
  }

  else {
    direction1 = (direction1 | 0x80);
    positionXbkp = positionX[1] - 1;
    positionXbkp = positionXbkp ^ 0xFFFFFFFF;
    posx_seg[0] = positionXbkp & 0x000000FF;
    posx_seg[1] = (positionXbkp >> 8) & 0x000000FF;
    posx_seg[2] = (positionXbkp >> 16) & 0x000000FF;
    posx_seg[3] = (positionXbkp >> 24) & 0x000000FF;
  }

  if (positionY[1] >= 0) { // Same management than in the previous case
    direction1 = (direction1 | 0x08); // but with the Y data.
    posy_seg[0] = positionY[1] & 0x000000FF;
    posy_seg[1] = (positionY[1] >> 8) & 0x000000FF;
    posy_seg[2] = (positionY[1] >> 16) & 0x000000FF;
    posy_seg[3] = (positionY[1] >> 24) & 0x000000FF;
  }

  else {
    direction1 = (direction1 | 0x01);
    positionYbkp = positionY[1] - 1;
    positionYbkp = positionYbkp ^ 0xFFFFFFFF;
    posy_seg[0] = positionYbkp & 0x000000FF;
    posy_seg[1] = (positionYbkp >> 8) & 0x000000FF;
    posy_seg[2] = (positionYbkp >> 16) & 0x000000FF;
    posy_seg[3] = (positionYbkp >> 24) & 0x000000FF;
  }

  delay = 0x0100;

  Sensor_Data[0] = 0x03;
  Sensor_Data[1] = direction1;
  Sensor_Data[2] = posx_seg[3];
  Sensor_Data[3] = posy_seg[3];
  Sensor_Data[4] = 0x01;
  Sensor_Data[5] = 0x01;

  while (--delay);
  //  Serial.print(posx_seg[3]);
  //  Serial.print("    ,    ");
  //  Serial.print(posy_seg[3]);
  //  Serial.print("    ,    ");
  //  Serial.print(direction1);
  //  Serial.println();
}

void data_reintegration(void)
{
  if (direction1 >= 10)
  {
    positionX[1] = positionX[1] | 0xFFFFC000; // 18 "ones" inserted. Same size as the
  }
  //amount of shifts

  direction1 = direction1 & 0x01;

  if (direction1 == 1)
  {
    positionY[1] = positionY[1] | 0xFFFFC000;
  }
}

void movement_end_check(void)
{
  if (accelerationx[1] == 0) //we count the number of acceleration samples that equals cero
  {
    countx++;
  }
  else {
    countx = 0;
  }

  if (countx >= 25) //if this number exceeds 25, we can assume that velocity is cero
  {
    velocityx[1] = 0;
    velocityx[0] = 0;
  }

  if (accelerationy[1] == 0) //we do the same for the Y axis
  {
    county++;
  }
  else {
    county = 0;
  }

  if (county >= 25)
  {
    velocityy[1] = 0;
    velocityy[0] = 0;
  }
  //  Serial.println(countx);
}

////////////////////////////////////////////////////////////      Position by IMU
void updatePos(void)
{
  //Calibrate();
  //  unsigned char count2 ;
  //  count2 = 0;
  //
  //  do {
  //    Read_Accel();
  //    accelerationx[1] = accel_x + Sample_X; //filtering routine for noise attenuation
  //    accelerationy[1] = accel_y + Sample_Y; //64 samples are averaged. The resulting
  //    //average represents the acceleration of
  //    //an instant
  //    count2++;
  //
  //  } while (count2 != 0x40); // 64 sums of the acceleration sample

  accelerationx[1] = accel_x >> 6; // division by 64
  accelerationy[1] = accel_y >> 6;

  //  accelerationx[1] = accelerationx[1] >> 6; // division by 64
  //  accelerationy[1] = accelerationy[1] >> 6;
  //
  //  accelerationx[1] = accelerationx[1] - (int)sstatex; //eliminating zero reference
  //  //offset of the acceleration data
  //  accelerationy[1] = accelerationy[1] - (int)sstatey; // to obtain positive and negative
  //  //acceleration


  if ((accelerationx[1] <= 3) && (accelerationx[1] >= -3)) //Discrimination window applied
  {
    accelerationx[1] = 0; // to the X axis acceleration
  }
  //variable

  if ((accelerationy[1] <= 3) && (accelerationy[1] >= -3))
  {
    accelerationy[1] = 0;
  }

  //first X integration:
  velocityx[1] = velocityx[0] + accelerationx[0] + ((accelerationx[1] - accelerationx[0]) >> 1);
  //second X integration:
  positionX[1] = positionX[0] + velocityx[0] + ((velocityx[1] - velocityx[0]) >> 1);
  //first Y integration:
  velocityy[1] = velocityy[0] + accelerationy[0] + ((accelerationy[1] - accelerationy[0]) >> 1);
  //second Y integration:
  positionY[1] = positionY[0] + velocityy[0] + ((velocityy[1] - velocityy[0]) >> 1);

  accelerationx[0] = accelerationx[1]; //The current acceleration value must be sent
  //to the previous acceleration
  accelerationy[0] = accelerationy[1]; //variable in order to introduce the new
  //acceleration value.

  velocityx[0] = velocityx[1]; //Same done for the velocity variable
  velocityy[0] = velocityy[1];

//  positionX[1] = positionX[1] << 18; //The idea behind this shifting (multiplication)
//  //is a sensibility adjustment.
//  positionY[1] = positionY[1] << 18; //Some applications require adjustments to a
//  //particular situation
//  //i.e. mouse application
  
  Serial.print(positionX[1]);
  Serial.print("    ,    ");
  Serial.print(positionY[1]);
  Serial.print("    ,    ");
  Serial.print(direction1);
  Serial.println();

  data_transfer();

//  positionX[1] = positionX[1] >> 18; //once the variables are sent them must return to
//  positionY[1] = positionY[1] >> 18; //their original state
//
//  //  data_reintegration();
//  movement_end_check();

  positionX[0] = positionX[1]; //actual position data must be sent to the
  positionY[0] = positionY[1]; //previous position

  direction1 = 0;

}



////////////////////////////////////////////////////////////     Update RPM
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

/////////////////////////////////////////////////////////////        PID loop
void speed_pid()
{
  for (int i = 0; i < 3; i++)
  {
    if (setpoint[i])
    {
      //updateRPM();
      now[i] = micros();
      timeChange[i] = (now[i] - lastTime[i]);

      if (timeChange[i] >= SampleTime)
      {

        ///////////////////////////////////////////               Error Variables
        error[i] = setpoint[i] - rpmValues[i];
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

        ////////////////////////////////////////////               PID output
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
    if (setpoint[i] == 0)
    {
      motor(i, 0);
    }
  }
}

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

      setpoint[0] = rpm0;
      setpoint[1] = rpm1;
      setpoint[2] = rpm2;
      break;

    ////////////////////////////////////////////////////////////////////  PID Switch ON/OFF
    case 'p':
    case 'P':
      setpoint[0] = 0;
      setpoint[1] = 0;
      setpoint[2] = 0;
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
      IMUsetup = 0;
      for (int i = 0; i < 3; i++)
      {
        motor(i, 0);
        setpoint[i] = 0;
      }
      break;

    //////////////////////////////////////////////////////////////////     PRINT IMU DATA
    case 'a':
    case 'A':
      IMUsetup = 1;
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
