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

/*****************************************             IMU                     ***************************************************************/
void setupIMU()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}
