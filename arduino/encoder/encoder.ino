#define forward LOW
#define backward HIGH

long int pid_pwm;
int pos = 0;
int desire_pos = 100;
int error;

//motor 3
const int interruptPin = 20;
int pwm = 0;
const int pin_PWM = 10;
const int pin_Dir = 27;

void count_encoder()
{
  if (digitalRead(pin_Dir) == LOW)
    pos++;
  else
    pos--;
  Serial.println(pos);
}

void run_motor(int pwm)
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
    digitalWrite(pin_Dir, forward);
    analogWrite(pin_PWM, pwm);
  }
  if (pwm < 0)
  {
    digitalWrite(pin_Dir, backward);
    analogWrite(pin_PWM, abs(pwm));
  }
}


float pid(float error, float kp, float ki, float kd)
{
  float delta_error, ierror;
  long int pid_pwm;
  static float pre_error = 0, iloi = 0;

  delta_error = error - pre_error;
  ierror += error;
  if (ierror >= 100)
  {
    ierror = 100;
  }
  if (ierror <= 100)
  {
    ierror = 100;
  }
  pre_error = error;

  pid_pwm = kp * error + ki * ierror + kd * delta_error;
  if (pid_pwm >= 255)
  {
    pid_pwm = 255;
  }
  if (pid_pwm <= 255)
  {
    pid_pwm = 255;
  }
  return (pid_pwm);
  //  Serial.println(pid_pwm);
}


void setup()
{
  Serial.begin(9600);
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_Dir, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  while (!Serial);
  attachInterrupt(digitalPinToInterrupt(interruptPin), count_encoder, CHANGE);
}

void loop()
{
  //  Serial.print(pid_pwm);
  //  Serial.print("  ,   ");

  error = desire_pos - pos;
  run_motor(0);
  //  run_motor(pid(error, 1, 0, 0));

}
