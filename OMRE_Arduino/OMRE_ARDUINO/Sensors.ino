/****************************************            COUNTING ENCODERS          *********************************/
void encoder0_ISR() // encoder0 interrupt service routine
{
  //  noInterrupts();
  if (digitalRead(motorDirPins[0]) == HIGH)
    encoderCounts[0]++;
  else
    encoderCounts[0]--;
  //  interrupts();
  //  Serial.println(encoderCounts[0]);
}
void encoder1_ISR()
{
  noInterrupts();
  if (digitalRead(motorDirPins[1]) == HIGH)
    encoderCounts[1]++;
  else
    encoderCounts[1]--;
  interrupts();
}
void encoder2_ISR()
{
  noInterrupts();
  if (digitalRead(motorDirPins[2]) == HIGH)
    encoderCounts[2]++;
  else
    encoderCounts[2]--;
  interrupts();
  //  Serial.println(encoderCounts[2]);
}

/*****************************************             UltraSonic               ***************************************************************/
void UsSensor()
{
  for (int i = 0; i < 6; i++)
  {
    digitalWrite(ultrasonicSensorTrigPins[i], LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonicSensorTrigPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasonicSensorTrigPins[i], LOW);

    duration_US[i] = pulseIn(ultrasonicSensorEchoPins[i], HIGH, 6500); //2500 = 30cm //6500 micro seconds gives about 100 cm max
    m_US[i] = ((duration_US[i] * 0.034) / 2) * 0.01;
  }
}

/*****************************************             Infrared                 ***************************************************************/
void irSensor()
{
  m_IR[0] = (IR_Sensor0.getDistance()) * 0.01; // this returns the distance to the object you're measuring
  m_IR[1] = (IR_Sensor1.getDistance()) * 0.01;
  m_IR[2] = (IR_Sensor2.getDistance()) * 0.01;
  m_IR[3] = (IR_Sensor3.getDistance()) * 0.01;
}

/*****************************************             Motor Current            ***************************************************************/
void m_current()
{
  for (int i = 0; i < 3; i++)
  {
    if (pwm_dir[i]   == '0')
    {
      for (int m = 0; m < avgSamples; m++)
      {
        m_cur[i] += analogRead(motorCurrentPins[i]) * 0.01;
      }
      m_cur[i] = m_cur[i] / avgSamples;
    }
    if (pwm_dir[i]   == '1')
    {
      for (int m = 0; m < avgSamples; m++)
      {
        m_cur[i] += -analogRead(motorCurrentPins[i]) * 0.01;
      }
      m_cur[i] = m_cur[i] / avgSamples;
    }
  }
}
/*****************************************             Motor Voltage            ***************************************************************/
void m_voltage()
{
  for (int i = 0; i < avgSamples; i++)
  {
    sensorValue += analogRead(motorVolPins);
    delay(2);
  }
  sensorValue = sensorValue / avgSamples;
  float voltage = sensorValue * 4.88;
  float current = (voltage - Vref) * sensitivity;
  float sense = 0.018 / (voltage - Vref);
  Serial.print(voltage);
  Serial.print(" , ");
  Serial.print(sense);
  Serial.print(" , ");
  Serial.println(current);
  delay(200);
  sensorValue = 0;

  //  vol_sen = analogRead(motorVolPins);
  //  //  m_vol   = map(vol_sen,0,1023, 0,2500) + 0;
  //  //  m_vol = (vol_sen - 512)* 0.073170;
  //  m_vol = ((vol_sen + 0.5)* 5.0) / 1024.0;
  //  Serial.print(vol_sen);
  //  Serial.print(" , ");
  //  Serial.println(m_vol);
  //  delay(500);


}
