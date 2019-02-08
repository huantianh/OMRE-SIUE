#define forward LOW
#define backward HIGH

int command = 0;
char command_end = 'a';
int encoder_counts_2 = 0;
int encoder_counts_3 = 0;
char rcv_buffer[64];
int  motor_pwm = 0;
int pwm = 0;

//motor 2
const int interruptPin_2 = 19;
const int pin_PWM_2 = 9;
const int pin_Dir_2 = 28;

as
//motor 3
const int interruptPin_3 = 20;
const int pin_PWM_3 = 10;
const int pin_Dir_3 = 27;

void count_encoder_2()
{
  if (digitalRead(pin_Dir_2) == HIGH)
    encoder_counts_2 ++;
  else
    encoder_counts_2 --;
  //    Serial.println(encoder_counts_2);
}

void count_encoder_3()
{
  if (digitalRead(pin_Dir_3) == LOW)
    encoder_counts_3 ++;
  else
    encoder_counts_3 --;
  //    Serial.println(encoder_counts_3);
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


void run_distance(float distance_goal, int pwm_goal)
{
  encoder_counts_2 = 0;
  encoder_counts_3 = 0;
  int tick_goal = 1911 * distance_goal;
  float distance_travel = 0;
  while (abs(encoder_counts_2) < tick_goal && abs(encoder_counts_3) < tick_goal)
  {
    distance_travel = encoder_counts_2 / 1911;
    run_motor_2(pwm_goal);
    run_motor_3(pwm_goal);
    Serial.print("Distance traveled:  ");
    Serial.print(distance_travel);
    Serial.print("  ,  ");
    Serial.print("encoder2:   ");
    Serial.print(encoder_counts_2);
    Serial.print("  ,  ");
    Serial.print("encoder3:   ");
    Serial.println(encoder_counts_3);
  }
  run_motor_2(0);
  run_motor_3(0);

  //  while (abs(encoder_counts_2) < tick_goal)
  //  {
  //    run_motor_2(pwm_goal);
  //    Serial.println(encoder_counts_2);
  //  }
  //  run_motor_2(0);
  //
  //  while (abs(encoder_counts_3) < tick_goal)
  //  {
  //    run_motor_3(pwm_goal);
  //    Serial.println(encoder_counts_3);
  //  }
  //  run_motor_3(0);
}



void communication()
{
  if (Serial.available() > 5)
  {
    command = Serial.parseInt();
    command_end = Serial.read();
    switch (command_end)
    {
      case 'l':
        if (command >= 0 && command < 1000)
        {
          motor_pwm = command;
          run_motor_2(motor_pwm);
          run_motor_2(motor_pwm);
          Serial.print("motor2 pwm:");
          Serial.println(motor_pwm);
          break;
        }
      case 'r':
        if (command >= 0 && command < 1000)
        {
          motor_pwm = command;
          run_motor_3(motor_pwm);
          run_motor_3(motor_pwm);
          Serial.print("motor3 pwm:");
          Serial.println(motor_pwm);
          break;
        }
      case 's':
        {
          if (command >= 0 && command < 1000)
          {
            motor_pwm = 0;
            run_motor_2(motor_pwm);
            run_motor_3(motor_pwm);
            Serial.println("stop!!");
            //Serial.println(motor_pwm);
            break;
          }
        }
    }

  }
}

void setup()
{
  Serial.begin(9600);
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
  attachInterrupt(digitalPinToInterrupt(interruptPin_2), count_encoder_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin_3), count_encoder_3, CHANGE);
}

void loop()
{
  communication();

//  run_distance(0.5, 150);
//  delay(500);
//  run_distance(0.5, -150);
//  delay(500);
//  run_distance(2, 50);
//  delay(500);

  //  Serial.print("motor2 pwm:  ");
  //  Serial.print(encoder_counts_2);
  //  Serial.print("  ,   ");
  //  Serial.print("motor3 pwm:  ");
  //  Serial.println(encoder_counts_3);

}
