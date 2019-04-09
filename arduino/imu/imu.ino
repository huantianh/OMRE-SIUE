//IMU setup
int SENSOR_SIGN[9] = {1, 1, 1, -1, -1, -1, 1, 1, 1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
#include <Wire.h>
// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer
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

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

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

void setup()
{
  Serial.begin(115200);
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
}

void loop() //Main Loop
{
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

    printdata();
  }

}


////////////////////////////////////////////////Main Code
#include <HardwareSerial.h>
#include <SimpleTimer.h>

#define ENCODER_0INTERRUPT_PIN 5 // pin 18 that interrupts on both rising and falling of A and B channels of encoder
#define ENCODER_1INTERRUPT_PIN 4 // pin 19 https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/ to see the encoder pin number
#define ENCODER_2INTERRUPT_PIN 3 // pin 20

#define FORWARD 0
#define BACKWARD 1
#define INFRARED_SENSOR_0 A0
#define INFRARED_SENSOR_1 A1
#define INFRARED_SENSOR_2 A2
#define INFRARED_SENSOR_3 A3

volatile long encoderCounts[]              = { 0, 0, 0}; // variables accesed inside of an interrupt need to be volatile
bool motorDir[3]                           = {FORWARD, FORWARD, FORWARD};

const int motorPWMPins[3]                  = {8, 9, 10};
const int motorDirPins[3]                  = {29, 28, 27};
const int ultrasonicSensorTrigPins[]       = {30, 32, 34, 36, 38, 40};
const int ultrasonicSensorEchoPins[]       = {31, 33, 35, 37, 39, 41};
const int infraredSensorPins[]             = {0, 1, 2, 3};

double Kp = 1;
double Ki = 0.0007;

double sum[3]        = {0, 0, 0};
double error[3]      = {0, 0, 0};
double setpoint[3]   = {0, 0, 0};
double pwmValue[3]   = {0, 0, 0};
double rpms[3]       = {0, 0, 0};

unsigned long lastTime[3]   = {0, 0, 0};
unsigned long timeChange[3] = {0, 0, 0};

int rpmValues[3]         = {0, 0, 0};
double pastEncoderValues[3]  = {0, 0, 0};
unsigned long pastTimes[3] = {0, 0, 0};// millis() works for up to 50days! we'll need an unsigned long for it

char rcv_buffer[64];  // holds commands recieved
char TXBuffer[64];    // temp storage for large data sent

void motor(int, int, bool);

double changeInEncoders;
double changeInRevolutions;
double changeInTimeSeconds;
char pidSwitch = '0';


////////////////////////////////////////////////////////////////           SET UP
void setup() {
  Serial.begin(9600);
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
  pinMode(19, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);

  buffer_Flush(rcv_buffer);

  while (! Serial);

  attachInterrupt(ENCODER_0INTERRUPT_PIN, encoder0_ISR, CHANGE);
  attachInterrupt(ENCODER_1INTERRUPT_PIN, encoder1_ISR, CHANGE);
  attachInterrupt(ENCODER_2INTERRUPT_PIN, encoder2_ISR, CHANGE);

}
//////////////////////////////////////////////////////////////////    LOOP
void loop() {

  updateRPM();

  // determines if we have any serial commands and interpruts them
  receiveBytes();

  // proportional integral controller
  if (pidSwitch == '1')
  {
    pi();
  }
}

////////////////////////////////////////////////////////////     Update RPM
void updateRPM() {

  for ( int i = 0; i < 3; i++)
  {
    changeInEncoders = encoderCounts[i] - pastEncoderValues[i];
    changeInTimeSeconds = ((micros() - pastTimes[i]) * 0.000001); // *.001 to convert to seconds
    changeInRevolutions = changeInEncoders / 2248.6;

    rpmValues[i] = (changeInRevolutions / (changeInTimeSeconds)) * 60; // *60 to get Revolutions per MINUTE

    // update our values to be used next time around
    pastTimes[i] = micros();
    pastEncoderValues[i] = encoderCounts[i];


    Serial.print(micros() * 0.000001);
    Serial.print("  ,  ");
    Serial.print(abs(rpmValues[0]));
    Serial.print("  ,  ");
    Serial.print(abs(rpmValues[1]));
    Serial.print("  ,  ");
    Serial.println(abs(rpmValues[2]));

  }
}

/////////////////////////////////////////////////////////////           PI loop
void pi() {

  for (int i = 0; i < 3; i++)
  {
    if (setpoint[i] != 0)
    {
      //updateRPM();
      timeChange[i] = (micros() - lastTime[i]);
      lastTime[i] = micros();

      error[i] = setpoint[i] - rpmValues[i];
      sum[i] = (sum[i] + (error[i] * (double)timeChange[i]));

      pwmValue[i] = (Kp * error[i]) + (Ki * sum[i]);

      if (pwmValue[i] < 0) {
        motor(i, pwmValue[i] * -1, 0);
      }
      else {
        motor(i, pwmValue[i], 1);
      }

      Serial.print(rpmValues[0]);
      Serial.print("  ,  ");
      Serial.print(rpmValues[1]);
      Serial.print("  ,  ");
      Serial.println(rpmValues[2]);
    }
    else
    {
      error[i] = 0;
      sum[i]   = 0;
      motor(i, 0, 0);
    }
  }
}

///////////////////////////////////////////////////            COUNTING ENCODERS
void encoder0_ISR() // encoder0 interrupt service routine
{
  noInterrupts();
  if (motorDir[0])
  {
    encoderCounts[0]++;
  }
  else
  {
    encoderCounts[0]--;
  }
  interrupts();
}
void encoder1_ISR()
{
  noInterrupts();
  if (motorDir[1])
  {
    encoderCounts[1]++;
  }
  else
  {
    encoderCounts[1]--;
  }
  interrupts();
}
void encoder2_ISR()
{
  noInterrupts();
  if (motorDir[2])
  {
    encoderCounts[2]++;
  }
  else
  {
    encoderCounts[2]--;
  }
  interrupts();
}

//////////////////////////////////////////////////////             RUNNING MOTORS
void motor(int motorNumber, int pwm, bool dir)
{
  motorDir[motorNumber] = dir;
  digitalWrite(motorDirPins[motorNumber], dir);
  analogWrite(motorPWMPins[motorNumber], pwm);
}

///////////////////////////////////////////////////////             RECEIVE BYTE
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

////////////////////////////////////////////////////////////////      BUFFER FLUSH
void buffer_Flush(char *ptr)
{
  for (int i = 0; i < 64; i++)
  {
    ptr[i] = 0;
  }
}

//////////////////////////////////////////////////////////////        GIVING COMMAND
void parseCommand()
{
  char command = rcv_buffer[0]; // our first byte tells us the command char is equivalent to byte
  switch (command)
  {
    ///////////////////////////////////// ENCODER
    case 'E':
    case 'e':
      int encoderNum;
      sscanf(&rcv_buffer[1], " %d \r", &encoderNum);
      long counts;
      counts = encoderCounts[encoderNum];
      //itoa(encoderCounts[encoderNum],TXBuffer,10);   // serial.print can not handle printing a 64bit int so we turn it  into a string
      Serial.println(counts);
      break;

    ////////////////////////////////////  MOTOR
    case 'M':
    case 'm':
      int  motorNumber;
      int  motorPWM;
      int  motorDirection;
      sscanf(&rcv_buffer[1], " %d %d %d \r", &motorNumber, &motorPWM, &motorDirection);
      motor(motorNumber, motorPWM, motorDirection);
      break;

    ////////////////////////////////////  ULTRASOUND
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
      rpms[0] = (double)(rpm0);
      rpms[1] = (double)(rpm1);
      rpms[2] = (double)(rpm2);
      for (int i = 0; i < 3; i++)
      {
        // when the setpoint is in a 30 +/- range do not set the sum to 0, aka if major velocity change set your sum
        // to 0. if small then don't change it
        if (!(setpoint[i] + 30 >= rpms[i] && setpoint[i] - 30 <= rpms[i]))
        {
          //error[i] = 0;
          sum[i]   = 0;
        }
      }
      setpoint[0] = rpms[0];
      setpoint[1] = rpms[1];
      setpoint[2] = rpms[2];
      break;

    ////////////////////////////////////  PID Switch
    case 'p':
    case 'P':
      setpoint[0] = 0;
      setpoint[1] = 0;
      setpoint[2] = 0;
      motor(0, 0, 0);
      motor(1, 0, 0);
      motor(2, 0, 0);
      sscanf(&rcv_buffer[1], " %c \r", &pidSwitch);
      break;

    ////////////////////////////////////  RPM
    case 'r':
    case 'R':
      int rpmNum;
      sscanf(&rcv_buffer[1], " %d \r", &rpmNum);
      printDouble(rpmValues[rpmNum], 1000000000);
      break;

    ////////////////////////////////////  ENTER GAINS K
    case 'k':
    case 'K':
      char  pValue[20];
      char  iValue[20];
      sscanf(&rcv_buffer[1], " %s %s \r", &pValue, &iValue);
      char *ptr;
      Kp = strtod(pValue, &ptr);
      Ki = strtod(iValue, &ptr);
      break;

    ////////////////////////////////////  STOP
    case 's':
    case 'S':
      for (int i = 0; i < 3; i++)
      {
        motor(i, 0, 0);
      }

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
