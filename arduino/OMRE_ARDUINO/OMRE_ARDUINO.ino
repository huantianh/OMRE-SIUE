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

//double Kp = 1.9;
//double Ki = 0.09;
//double Kd = 0.2;

double Kp = 1;
double Ki = 0.1;
double Kd = 0;

double ITerm[3]       = {0, 0, 0};
double lastInput[3]   = {0, 0, 0};
double error[3]       = {0, 0, 0};
double dInput[3]      = {0, 0, 0};
double setpoint[3]    = {0, 0, 0};
double pwm_pid[3]     = {0, 0, 0};
double rpms[3]        = {0, 0, 0};

unsigned long now[3]        = {0, 0, 0};
unsigned long lastTime[3]   = {0, 0, 0};
unsigned long timeChange[3] = {0, 0, 0};
int SampleTime = 1000;

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
void loop()
{
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
void pi()
{
  for (int i = 0; i < 3; i++)
  {
    if (setpoint[i] != 0)
    {
      //updateRPM();
      now[i] = micros();
      timeChange[i] = (now[i] - lastTime[i]);

      if (timeChange[i] >= SampleTime)
      {

        ///////////////////////////////////////////        Error Variables
        error[i] = setpoint[i] - rpmValues[i];
        ITerm[i] += (Ki * error[i]);

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
        pwm_pid[i] = (Kp * error[i]) + ITerm[i] - (Kd * dInput[i]);

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

        Serial.print(rpmValues[0]);
        Serial.print("  ,  ");
        Serial.print(rpmValues[1]);
        Serial.print("  ,  ");
        Serial.println(rpmValues[2]);
      }
    }
    else
    {
      motor(i, 0);
    }
  }
}

////////////////////////////////////////////////////////////            COUNTING ENCODERS
void encoder0_ISR() // encoder0 interrupt service routine
{
  noInterrupts();
  if (motorDir[0] == 0)
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
  if (motorDir[1] == 0)
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
  if (motorDir[2] == 0)
  {
    encoderCounts[2]++;
  }
  else
  {
    encoderCounts[2]--;
  }
  interrupts();
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
    digitalWrite(motorDirPins[motorNumber], 0);
    analogWrite(motorPWMPins[motorNumber], pwm);
  }
  if (pwm < 0)
  {
    digitalWrite(motorDirPins[motorNumber], 1);
    analogWrite(motorPWMPins[motorNumber], pwm);
  }
  if (pwm == 0)
  {
    digitalWrite(motorDirPins[motorNumber], 1);
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

      rpms[0] = (double)(rpm0);
      rpms[1] = (double)(rpm1);
      rpms[2] = (double)(rpm2);

      setpoint[0] = rpms[0];
      setpoint[1] = rpms[1];
      setpoint[2] = rpms[2];
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
      Kp = strtod(pValue, &ptr);
      Ki = strtod(iValue, &ptr);
      break;

    ///////////////////////////////////////////////////////////////////  STOP
    case 's':
    case 'S':
      for (int i = 0; i < 3; i++)
      {
        motor(i, 0);
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
