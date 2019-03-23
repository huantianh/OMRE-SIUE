#include <PID_v1.h>


#define forward LOW
#define backward HIGH


volatile long encoder_counts[3] = {0,0,0};

int  motor_pwm;
int pwm;
int rpm_goal;

float encoder_changed_1;
float old_encoder_1;
float revolutions_changed_1;
int motor_rpm_1;
float encoder_changed_2;
float old_encoder_2;
float revolutions_changed_2;
int motor_rpm_2;
float encoder_changed_3;
float old_encoder_3;
float revolutions_changed_3;
int motor_rpm_3;

double total_time;
double time_changed;
unsigned long past_times;
long int pidTerm;
int error;
double errSum;
double Kp = 2;
double Ki = 0;
double Kd = 0;

char rcv_buffer[64];  // holds commands recieved
char TXBuffer[64];    // temp storage for large data sent

//PID libary
double Input, Output;
double Setpoint = rpm_goal;
PID myPID(&Input, &Output, &Setpoint, 1, 0, 0, DIRECT);

//motor 1
const int interruptPin_1 = 18;
const int pin_PWM_1 = 8;
const int pin_Dir_1 = 29;

//motor 2
const int interruptPin_2 = 19;
const int pin_PWM_2 = 9;
const int pin_Dir_2 = 28;

//motor 3
const int interruptPin_3 = 20;
const int pin_PWM_3 = 10;
const int pin_Dir_3 = 27;

void setup()
{
  Serial.begin(9600);

  //motor 1 pin setup
  pinMode(pin_PWM_1, OUTPUT);
  pinMode(pin_Dir_1, OUTPUT);
  pinMode(interruptPin_1, INPUT_PULLUP);
  //motor 2 pin setup
  pinMode(pin_PWM_2, OUTPUT);
  pinMode(pin_Dir_2, OUTPUT);
  pinMode(interruptPin_2, INPUT_PULLUP);
  //motor 3 pin setup
  pinMode(pin_PWM_3, OUTPUT);
  pinMode(pin_Dir_3, OUTPUT);
  pinMode(interruptPin_3, INPUT_PULLUP);

  //interrupt setup
  while (!Serial);
  attachInterrupt(digitalPinToInterrupt(interruptPin_1), count_encoder_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin_2), count_encoder_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin_3), count_encoder_3, CHANGE);

  //timer 1 setup
  TCCR1A = 0;
  TCCR1B = 0b111;

  //PID Libary
  Input = analogRead(pin_PWM_2);
  myPID.SetMode(AUTOMATIC);

}

void loop()
{
  RPM();
  pid();
  print_result();
  receiveBytes();
  //  //PID Libary
  //  Input = analogRead(pin_PWM_2);
  //  myPID.Compute();
  //  analogWrite(pin_PWM_2, Output);
}

//////////////////////////////////////////////Counting Encoder
void count_encoder_1()
{
  if (digitalRead(pin_Dir_1) == LOW)
    encoder_counts[1] ++;
  else
    encoder_counts[1] --;
  //  Serial.println(encoder_counts_1);
}
void count_encoder_2()
{
  if (digitalRead(pin_Dir_2) == HIGH)
    encoder_counts[2] ++;
  else
    encoder_counts[2] --;
  //  Serial.println(encoder_counts_2);
}
void count_encoder_3()
{
  if (digitalRead(pin_Dir_3) == LOW)
    encoder_counts[3] ++;
  else
    encoder_counts[3] --;
  //  Serial.println(encoder_counts_3);
}

////////////////////////////////////////////// Running Motor
void run_motor_1(int pwm)
{
  if (pwm >= 255)
  {
    pwm = 255;
  }
  if (pwm <= -255)
  {
    pwm = -255;
  }
  if (pwm > 0)
  {
    digitalWrite(pin_Dir_1, forward); // diffrent direction with motor3 beacuse 2 wheels need to run diffrent direction
    analogWrite(pin_PWM_1, pwm);
  }
  if (pwm < 0)
  {
    digitalWrite(pin_Dir_1, backward);
    analogWrite(pin_PWM_1, abs(pwm));
  }
  if (pwm == 0)
  {
    analogWrite(pin_PWM_1, 0);
  }
}

void run_motor_2(int pwm)
{
  if (pwm >= 255)
  {
    pwm = 255;
  }
  if (pwm <= -255)
  {
    pwm = -255;
  }
  if (pwm > 0)
  {
    digitalWrite(pin_Dir_2, backward); // diffrent direction with motor3 beacuse 2 wheels need to run diffrent direction
    analogWrite(pin_PWM_2, pwm);
  }
  if (pwm < 0)
  {
    digitalWrite(pin_Dir_2, forward);
    analogWrite(pin_PWM_2, abs(pwm));
  }
  if (pwm == 0)
  {
    analogWrite(pin_PWM_2, 0);
  }
}

void run_motor_3(int pwm)
{
  if (pwm >= 255)
  {
    pwm = 255;
  }
  if (pwm <= -255)
  {
    pwm = -255;
  }
  if (pwm > 0)
  {
    digitalWrite(pin_Dir_3, forward);
    analogWrite(pin_PWM_3, pwm);
  }
  if (pwm < 0)
  {
    digitalWrite(pin_Dir_3, backward);
    analogWrite(pin_PWM_3, abs(pwm));
  }
  if (pwm == 0)
  {
    analogWrite(pin_PWM_3, 0);
  }
}

/////////////////////////////////////// Converting Encoder to RPM
void RPM()
{
  if (command != 0)
  {
    encoder_changed_1 = encoder_counts_1 - old_encoder_1;
    encoder_changed_2 = encoder_counts_2 - old_encoder_2;
    encoder_changed_3 = encoder_counts_3 - old_encoder_3;

    time_changed = ((micros() - past_times) * 0.000001);
    total_time += time_changed;

    revolutions_changed_1 = encoder_changed_1 / 2248.86;
    revolutions_changed_2 = encoder_changed_2 / 2248.86;
    revolutions_changed_3 = encoder_changed_3 / 2248.86;

    motor_rpm_1 = (revolutions_changed_1 / (time_changed)) * 60; //Revolutions per Minute
    motor_rpm_2 = (revolutions_changed_2 / (time_changed)) * 60;
    motor_rpm_3 = (revolutions_changed_3 / (time_changed)) * 60;

    past_times = micros();

    old_encoder_1 = encoder_counts_1;
    old_encoder_2 = encoder_counts_2;
    old_encoder_3 = encoder_counts_3;

    Serial.print(total_time);
    Serial.print("  ,  ");
    Serial.print(motor_rpm_1);
    Serial.print("  ,  ");
    Serial.print(motor_rpm_2);
    Serial.print("  ,  ");
    Serial.println(motor_rpm_3);
  }
}

//////////////////////////////////////////////////    PID
void pid()
{
  double last_error;

  if (rpm_goal != 0)
  {
    if (rpm_goal >= 210)
    {
      rpm_goal = 210;
    }
    if (rpm_goal  <= -210)
    {
      rpm_goal = -210;
    }

    time_changed = ((micros() - past_times) * 0.000001);
    past_times = micros();

    error = abs(rpm_goal) - abs(motor_rpm_2);
    errSum += (error * time_changed);

    if (errSum >= 200)
    {
      errSum = 200;
    }
    if (errSum <= -200)
    {
      errSum = -200;
    }

    pidTerm = (Kp * error) + (Ki * errSum) + (Kd * ((error - last_error) / time_changed));
    if (pidTerm >= 255)
    {
      pidTerm = 255;
    }
    if (pidTerm <= -255)
    {
      pidTerm = -255;
    }

    run_motor_2(pidTerm);
    last_error = error;

    Serial.print(motor_rpm_2);
    Serial.print("  ,   ");
    Serial.println(error);
  }
}

///////////////////////////////////////////  Printing Results
void print_result()
{
  //  Serial.print("RPM:   ");  Serial.print(motor_rpm);
  //  Serial.println(motor_rpm);
  //  Serial.print("    ,     ");
  //  Serial.println(error);
  //  Serial.print("  ,  ");    Serial.print("RPM_Goal:   ");
  //  Serial.print("  ,  ");
  //  Serial.println(rpm_goal);

}

/////////////////////////////////////////// Giving Command
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

void buffer_Flush(char *ptr)
{
  for (int i = 0; i < 64; i++)
  {
    ptr[i] = 0;
  }
}

void parseCommand()
{
  char command = rcv_buffer[0]; // our first byte tells us the command char is equivalent to byte
  
  switch (command)
  {
    case 'E':
    case 'e':
      int encoderNum;
      sscanf(&rcv_buffer[1], " %d \r", &encoderNum);
      long counts;
      counts = encoder_counts[encoderNum];
      //itoa(encoderCounts[encoderNum],TXBuffer,10);   // serial.print can not handle printing a 64bit int so we turn it  into a string
      Serial.println(counts);
      break;

    case 'M':
    case 'm':
      int  motorNumber;
      int  motorPWM;
      int  motorDirection;

      sscanf(&rcv_buffer[1], " %d %d %d \r", &motorNumber, &motorPWM, &motorDirection);
      motor(motorNumber, motorPWM, motorDirection);
      break;

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
    case 'v':
    case 'V':

      int rpm0;
      int rpm1;
      int rpm2;
      sscanf(&rcv_buffer[1], "%d %d %d \r", &rpm0, &rpm1, &rpm2);
      //Serial.println(rpm0);
      //Serial.println(rpm1);
      //Serial.println(rpm2);

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

    case 'r':
    case 'R':
      int rpmNum;
      sscanf(&rcv_buffer[1], " %d \r", &rpmNum);
      printDouble(rpmValues[rpmNum], 1000000000);
      break;

    case 'k':
    case 'K':
      char  pValue[20];
      char  iValue[20];
      sscanf(&rcv_buffer[1], " %s %s \r", &pValue, &iValue);
      char *ptr;
      Kp = strtod(pValue, &ptr);
      Ki = strtod(iValue, &ptr);
      break;


      // default:
      //Serial.println("Error: Serial input incorrect");
  }
}
