#include <PID_v1.h>


#define forward LOW
#define backward HIGH

int command = 0;
char param = 'a';
volatile long encoder_counts_1 = 0;
volatile long encoder_counts_2 = 0;
volatile long encoder_counts_3 = 0;

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
  giving_command();
  pid();
  print_result();

  //  //PID Libary
  //  Input = analogRead(pin_PWM_2);
  //  myPID.Compute();
  //  analogWrite(pin_PWM_2, Output);
}

//////////////////////////////////////////////Counting Encoder
void count_encoder_1()
{
  if (digitalRead(pin_Dir_1) == LOW)
    encoder_counts_1 ++;
  else
    encoder_counts_1 --;
  //  Serial.println(encoder_counts_1);
}
void count_encoder_2()
{
  if (digitalRead(pin_Dir_2) == HIGH)
    encoder_counts_2 ++;
  else
    encoder_counts_2 --;
  //  Serial.println(encoder_counts_2);
}
void count_encoder_3()
{
  if (digitalRead(pin_Dir_3) == LOW)
    encoder_counts_3 ++;
  else
    encoder_counts_3 --;
    Serial.println(encoder_counts_3);
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
    
//    Serial.print(total_time);
//    Serial.print("  ,  ");
//    Serial.print(motor_rpm_1);
//    Serial.print("  ,  ");
//    Serial.print(motor_rpm_2);
//    Serial.print("  ,  ");
//    Serial.println(motor_rpm_3);
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
void giving_command()
{
  if (!Serial.available())    return 0;
  delay(10);
  param = Serial.read();
  if (!Serial.available())    return 0;
  command = Serial.parseInt();
  Serial.flush();
  switch (param)
  {
    case 'l':
      motor_pwm = command;
      run_motor_2(motor_pwm);
      //      Serial.print("motor2 pwm:");
      //      Serial.println(motor_pwm);
      break;

    case 'r':
      motor_pwm = command;
      run_motor_3(motor_pwm);
      //      Serial.print("motor3 pwm:");
      //      Serial.println(motor_pwm);
      break;

    case 'b':
      motor_pwm = command;
      run_motor_1(motor_pwm);
      //      Serial.print("motor1 pwm:");
      //      Serial.println(motor_pwm);
      break;

    case 's':
      motor_pwm = command;
      run_motor_1(motor_pwm);
      run_motor_2(motor_pwm);
      run_motor_3(motor_pwm);
      Serial.println("stop!!");
      break;

    case 'v':
      rpm_goal = command;
      Serial.print("RPM_Goal:  ");
      Serial.println(rpm_goal);
      break;

    default :
      {
        Serial.println("????????");
      }
  }
}
