void printdata(void)
{
  if (PRINT_ULTRASOUND == '1')
  {
    Serial.print(m_US[0]);
    Serial.print("  ,  ");
    Serial.print(m_US[1]);
    Serial.print("  ,  ");
    Serial.print(m_US[2]);
    Serial.print("  ,  ");
    Serial.print(m_US[3]);
    Serial.print("  ,  ");
    Serial.print(m_US[4]);
    Serial.print("  ,  ");
    Serial.println(m_US[5]);
  }

  if (PRINT_IR == '1')
  {
    Serial.print(m_IR[0]);
    Serial.print("  ,  ");
    Serial.print(m_IR[1]);
    Serial.print("  ,  ");
    Serial.print(m_IR[2]);
    Serial.print("  ,  ");
    Serial.println(m_IR[3]);
  }

  if (PRINT_RPM == '1')
  {
    Serial.print(micros() * 0.000001);
    Serial.print("  ,  ");
    Serial.print(rpmValues[0]);
    Serial.print("  ,  ");
    Serial.print(rpmValues[1]);
    Serial.print("  ,  ");
    Serial.println(rpmValues[2]);
  }

  if (PRINT_IMU == '1')
  {
    Serial.print(accel_x);
    Serial.print("  ,  ");
    Serial.print(accel_y);
    Serial.print("  ,  ");
    Serial.print(accel_z);
    Serial.print("  ,  ");
    Serial.print(accel_filter_x);
    Serial.print("  ,  ");
    Serial.print(accel_filter_y);
    Serial.print("  ,  ");
    Serial.print(accel_filter_z);
    Serial.print("  ,  ");
    Serial.print(vx);
    Serial.print("  ,  ");
    Serial.print(vy);
    Serial.print("  ,  ");
    Serial.print(vz);
    Serial.print("  ,  ");
    Serial.print(pos_x);
    Serial.print("  ,  ");
    Serial.print(pos_y);
    Serial.print("  ,  ");
    Serial.println(pos_z);
    //    delay(50);
  }
}
