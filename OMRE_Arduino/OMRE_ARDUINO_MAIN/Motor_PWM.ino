/*****************************************          RUNNING MOTORS          ***************************************/
void motor(int motorNumber, float pwm)
{
  if (pwm > 255)
  {
    pwm = 255;
  }

  if (pwm < -255)
  {
    pwm = -255;
  }

  if (pwm > 0)
  {
    digitalWrite(motorDirPins[motorNumber], FORWARD);
    analogWrite(motorPWMPins[motorNumber], pwm);
  }

  if (pwm < 0)
  {
    digitalWrite(motorDirPins[motorNumber], BACKWARD);
    analogWrite(motorPWMPins[motorNumber], -pwm);
  }

  if (pwm == 0)
  {
    //    digitalWrite(motorDirPins[motorNumber], BACKWARD);
    analogWrite(motorPWMPins[motorNumber], pwm);
  }
}
