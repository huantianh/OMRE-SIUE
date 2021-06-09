/*************************************       PID for Speed        ****************************************************************/
unsigned long now1[3]          = {0, 0, 0};
unsigned long lastTime1[3]     = {0, 0, 0};
unsigned long timeChange1[3]   = {0, 0, 0};
double ITerm1[3]               = {0, 0, 0};
double lastInput1[3]           = {0, 0, 0};
double error1[3]               = {0, 0, 0};
double dInput1[3]              = {0, 0, 0};
double pwm_pid1[3]             = {0, 0, 0};
double vol_pid1[3]             = {0, 0, 0};
double SampleTime1             = 1000;

double SampleTimeInSec1 = ((double)SampleTime1) / 1000;

double kp1 = 2;
double ki1 = 2 * SampleTimeInSec1;
double kd1 = 0 / SampleTimeInSec1;

//double kp = 0.3;
//double ki = 0.02;
//double kd = 0;

double Kp1[] = {kp1, kp1, kp1};
double Ki1[] = {ki1, ki1, ki1};
double Kd1[] = {kd1, kd1, kd1};

void cur_pid()
{
  for (int i = 0; i < 3; i++)
  {
    if (cur_setpoint[i])
    {
      //updateRPM();
      now1[i] = micros();
      timeChange1[i] = (now1[i] - lastTime1[i]);

      if (timeChange1[i] >= SampleTime1)
      {

        ///////////////////////////////////////////        Error Variables
        error1[i] = cur_setpoint[i] - m_cur[i];
        ITerm1[i] += (Ki1[i] * error1[i]);

        if (ITerm1[i] > 12)
        {
          ITerm1[i] = 12;
        }
        if (ITerm1[i] < -12)
        {
          ITerm1[i] = -12;
        }

        dInput1[i] = (m_cur[i] - lastInput1[i]);

        ////////////////////////////////////////////       PID output

        vol_pid1[i] = (Kp1[i] * error1[i]) + ITerm1[i] - (Kd1[i] * dInput1[i]);
        pwm_pid1[i] = 18.2194 * vol_pid1[i] - 28.2786;

        //        Serial.println(pwm_pid1[0]);

        //        pwm_pid1[i] = (Kp1[i] * error1[i]) + ITerm1[i] - (Kd1[i] * dInput1[i]);

        if (pwm_pid1[i] > 255)
        {
          pwm_pid1[i] = 255;
        }
        if (pwm_pid1[i] < -255)
        {
          pwm_pid1[i] = -255;
        }

        ///////////////////////////////////////////        Run Motor
        motor(i, pwm_pid1[i]);

        //////////////////////////////////////////         Remember Variables for next time
        lastInput1[i] = m_cur[i];
        lastTime1[i] = now1[i];

        delay(2);
      }
    }
    if (cur_setpoint[i] == 0)
    {
      motor(i, 0);
    }
  }
}
/***************************************************************************************************************************************/
