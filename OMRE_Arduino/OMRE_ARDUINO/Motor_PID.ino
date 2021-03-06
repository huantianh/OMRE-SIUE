/*************************************       PID for Speed        ****************************************************************/
unsigned long now[3]          = {0, 0, 0};
unsigned long lastTime[3]     = {0, 0, 0};
unsigned long timeChange[3]   = {0, 0, 0};
double ITerm[3]               = {0, 0, 0};
double lastInput[3]           = {0, 0, 0};
double error[3]               = {0, 0, 0};
double dInput[3]              = {0, 0, 0};
double pwm_pid[3]             = {0, 0, 0};
int    rpms[3]                = {0, 0, 0};
double SampleTime             = 1000;

double SampleTimeInSec = ((double)SampleTime)/1000;

double kp = 1;
double ki = 0.15 * SampleTimeInSec;
double kd = 0 / SampleTimeInSec;

//double kp = 0.3;
//double ki = 0.02; 
//double kd = 0;

double Kp[] = {kp, kp, kp};
double Ki[] = {ki, ki, ki};
double Kd[] = {kd, kd, kd};

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
