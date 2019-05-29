void printdata(void)
{
#if PRINT_ULTRASOUND == 1
  Serial.print(cm[0]);
  Serial.print("  ,  ");
  Serial.print(cm[1]);
  Serial.print("  ,  ");
  Serial.print(cm[2]);
  Serial.print("  ,  ");
  Serial.print(cm[3]);
  Serial.print("  ,  ");
  Serial.print(cm[4]);
  Serial.print("  ,  ");
  Serial.println(cm[5]);
#endif

#if PRINT_IR == 1
  Serial.print(cm_IR[0]);
  Serial.print("  ,  ");
  Serial.print(cm_IR[1]);
  Serial.print("  ,  ");
  Serial.print(cm_IR[2]);
  Serial.print("  ,  ");
  Serial.println(cm_IR[3]);
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
