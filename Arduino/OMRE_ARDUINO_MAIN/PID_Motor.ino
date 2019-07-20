/*********************************************************   PID for Motor    ****************************************************************/
void speed_pid()
{
  for (int i = 0; i < 3; i++)
  {
    if (rpm_setpoint[i])
    {
      //updateRPM();
      now[i] = micros();
      timeChange[i] = (now[i] - lastTime[i]);

      if (timeChange[i] >= SampleTime)
      {

        ///////////////////////////////////////////        Error Variables
        error[i] = rpm_setpoint[i] - rpmValues[i];
        ITerm[i] += (Ki[i] * error[i]);

        if (ITerm[i] > 255)
        {
          ITerm[i] = 255;
        }
        if (ITerm[i] < -255)
        {
          ITerm[i] = -255;
        }

        dInput[i] = (rpmValues[i] - lastInput[i]);

        ////////////////////////////////////////////       PID output
        pwm_pid[i] = (Kp[i] * error[i]) + ITerm[i] - (Kd[i] * dInput[i]);

        if (pwm_pid[i] > 255)
        {
          pwm_pid[i] = 255;
        }
        if (pwm_pid[i] < -255)
        {
          pwm_pid[i] = -255;
        }

        ///////////////////////////////////////////        Run Motor
        motor(i, pwm_pid[i]);

        //////////////////////////////////////////         Remember Variables for next time
        lastInput[i] = rpmValues[i];
        lastTime[i] = now[i];
        
        delay(2);
      }
    }
    if (rpm_setpoint[i] == 0)
    {
      motor(i, 0);
    }
  }
}
/***************************************************************************************************************************************/
