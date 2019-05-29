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

/*****************************************             UltraSonic                 ***************************************************************/
void UsSensor()
{
  for (int i = 0; i < 6; i++)
  {
    digitalWrite(ultrasonicSensorTrigPins[i], LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonicSensorTrigPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasonicSensorTrigPins[i], LOW);

    duration_US[i] = pulseIn(ultrasonicSensorEchoPins[i], HIGH, 5000); //6500 micro seconds gives about 100 cm max
    cm_US[i] = (duration_US[i] * 0.034) / 2;
  }
}

/*****************************************             Infrared                 ***************************************************************/
void irSensor()
{
  cm_IR[0] = IR_Sensor0.getDistance();  // this returns the distance to the object you're measuring
  cm_IR[1] = IR_Sensor1.getDistance();
  cm_IR[2] = IR_Sensor2.getDistance();
  cm_IR[3] = IR_Sensor3.getDistance();
}
