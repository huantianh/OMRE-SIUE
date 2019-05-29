/*************************************************    Update RPM    ***********************************************/
void updateRPM()
{
  for ( int i = 0; i < 3; i++)
  {
    changeInEncoders[i] = encoderCounts[i] - pastEncoderValues[i];
    changeInTimeSeconds[i] = ((micros() - pastTimes[i]) * 0.000001); // *.001 to convert to seconds
    changeInRevolutions[i] = changeInEncoders[i] / 2248.6;

    rpmValues[i] = (changeInRevolutions[i] / (changeInTimeSeconds[i])) * 60; // *60 to get Revolutions per MINUTE

    // update our values to be used next time around
    pastTimes[i] = micros();
    pastEncoderValues[i] = encoderCounts[i];
    //    Serial.print(micros() * 0.000001);
    //    Serial.print("  ,  ");
    //    Serial.print(rpmValues[0]);
    //    Serial.print("  ,  ");
    //    Serial.print(rpmValues[1]);
    //    Serial.print("  ,  ");
    //    Serial.println(rpmValues[2]);

  }
}
/***************************************************************************************************************************************/
