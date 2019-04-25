
void printdataIMU (void)
{
  Serial.print(accelerationx[1]);
  Serial.print("   ,   ");
  Serial.print(accelerationy[1]);
  Serial.print("   ,   ");
  Serial.println(accel_z);
}
