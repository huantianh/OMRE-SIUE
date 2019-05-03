void printdata(void)
{
  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  //  if ((a.acceleration.x <= 0.05) && (a.acceleration.x >= -0.05)) //Discrimination window applied
  //  {
  //    a.acceleration.x = 0; // to the X axis acceleration
  //  }
  //  //variable
  //
  //  if ((a.acceleration.y <= 0.05) && (a.acceleration.y >= -0.05))
  //  {
  //    a.acceleration.y = 0;
  //  }

#if PRINT_IMU == 1
  Serial.print(a.acceleration.x);
  Serial.print("   ,   ");
  Serial.print(a.acceleration.y);
  Serial.print("   ,   ");
  Serial.print(a.acceleration.z);
  Serial.print("   ,   ");
  Serial.print(g.gyro.x);
  Serial.print("   ,   ");
  Serial.print(g.gyro.y);
  Serial.print("   ,   ");
  Serial.print(g.gyro.z);
  Serial.println();
  delay(200);
#endif

#if PRINT_VELO_IMU == 1
  Serial.print(accel_filter_x);
  Serial.print("   ,   ");
  Serial.print(accel_filter_y);
  Serial.print("   ,   ");
  Serial.print(accel_z);
  Serial.print("   ,   ");
  Serial.print(vx);
  Serial.print("   ,   ");
  Serial.print(vy);
  Serial.print("   ,   ");
  Serial.print(vz);
  Serial.print("   ,   ");
  Serial.print(v);
  Serial.print("   ,   ");
  Serial.print(pos_x);
  Serial.println();
  delay(10);
#endif

#if PRINT_RPM == 1
  Serial.print(micros() * 0.000001);
  Serial.print("  ,  ");
  Serial.print(rpmValues[0]);
  Serial.print("  ,  ");
  Serial.print(rpmValues[1]);
  Serial.print("  ,  ");
  Serial.println(rpmValues[2]);
#endif

}
