/****************************************            COUNTING ENCODERS          *********************************/
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

void ultrasound_read()
{
  for (int i = 0; i < 6; i++)
  {
    pinMode(ultrasonicSensorTrigPins[i], OUTPUT);
    digitalWrite(ultrasonicSensorTrigPins[i], LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonicSensorTrigPins[i], HIGH);
    delayMicroseconds(5);
    digitalWrite(ultrasonicSensorTrigPins[i], LOW);
  }

  for (int i = 0; i < 6; i++)
  {
    pinMode(ultrasonicSensorEchoPins[i], INPUT);
    duration = pulseIn(ultrasonicSensorEchoPins[i], HIGH);
  }


             //  start = micros();
             //  while (digitalRead(ultrasonicSensorEchoPins[ultrasonicNumber]) == LOW);
             //  start = micros();
             //
             //  while (micros() - start <= 6000)
             //  {
             //    if (digitalRead(ultrasonicSensorEchoPins[ultrasonicNumber]) == LOW)
             //    {
             //      duration = micros() - start;
             //      break;
             //    }
             //    //duration = micros()-start;
             //  }
             //  //duration = pulseIn(ultrasonicSensorEchoPins[ultrasonicNumber], HIGH);
             //  cm = (duration / 2) / 29.1;
             //  //Serial.println(duration);
             //  //inches = (duration/2) /
             //  Serial.println(cm);

}
