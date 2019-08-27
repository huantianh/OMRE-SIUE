/*************************************************    Update RPM    ***********************************************/
double pastEncoderValues[3]   = {0, 0, 0};
unsigned long pastTimes[3]    = {0, 0, 0};// millis() works for up to 50days! we'll need an unsigned long for it
double changeInEncoders[3]    = {0, 0, 0};
double changeInRevolutions[3] = {0, 0, 0};
double changeInTimeSeconds[3] = {0, 0, 0};

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
  }
}
/***************************************************************************************************************************************/

/******************************************  Update Pos_IMU ************************************************/
float gyro_x;
float last_accel_x;
float last_accel_y;
float last_accel_z;

int countx   = 0;
int county   = 0;
int countz   = 0;
int countv   = 0;

double alpha = 0.1;
double prev_accel;

unsigned long present       = 0;
unsigned long last          = 0;

float filter_accel;
float dt = 0.5;

void updatePos()
{
  // alpha = 0:1;
  accel_filter_x = 0.06 *  accel_x + (1 - 0.06) * last_accel_x;
  accel_filter_y = 0.06 *  accel_y + (1 - 0.06) * last_accel_y;
  accel_filter_z = 0.002 *  accel_z + (1 - 0.002) * last_accel_z;

  if (( accel_filter_x  <= 0.03) && ( accel_filter_x >= -0.03)) //Discrimination window applied
  {
    accel_filter_x = 0; // to the X axis acceleration
  }

  if (( accel_filter_y  <= 0.03) && ( accel_filter_y >= -0.03)) //Discrimination window applied
  {
    accel_filter_y = 0; // to the Y axis acceleration
  }

  if (( accel_filter_z  <= 0.03) && ( accel_filter_z >= -0.03)) //Discrimination window applied
  {
    accel_filter_z = 0; // to the Z axis acceleration
  }

  present = micros() * 0.000001;
  dt = (present - last);

  vx += accel_filter_x * dt;
  vy += accel_filter_y * dt;
  vz += accel_filter_z * dt;

  v = sqrt(vx * vx + vy * vy);

  pos_x += vx * dt;
  pos_y += vy * dt;
  pos_z += vz * dt;

  pos = sqrt(pos_x * pos_x + pos_y * pos_y);


  ///////////////////// Checking velocity_end for X
  if (accel_filter_x  == 0) //we count the number of acceleration samples that equals zero
  {
    countx++;
  }
  else
  {
    countx = 0;
  }

  if (countx >= 20) //if this number exceeds 25, we can assume that velocity is zero
  {
    vx = 0;
  }

  ///////////////////// Checking velocity_end for Y
  if (accel_filter_y  == 0) //we count the number of acceleration samples that equals zero
  {
    county++;
  }
  else
  {
    county = 0;
  }

  if (county >= 20) //if this number exceeds 25, we can assume that velocity is zero
  {
    vy = 0;
  }

  ////////////////////// Checking velocity_end for Z
  if (accel_filter_z  == 0) //we count the number of acceleration samples that equals zero
  {
    countz++;
  }
  else
  {
    countz = 0;
  }

  if (countz >= 20) //if this number exceeds 25, we can assume that velocity is zero
  {
    vz = 0;
  }

  ///////////////////// Checking velocity_end for V
  if ((accel_filter_x  == 0) && (accel_filter_y == 0)) //we count the number of acceleration samples that equals zero
  {
    countv++;
  }
  else
  {
    countv = 0;
  }

  if (countv >= 20) //if this number exceeds 25, we can assume that velocity is zero
  {
    v = 0;
    pos = 0;
    vx = 0;
    vy = 0;
    pos_x = 0;
    pos_y = 0;
  }

  /////////////////////// Remember for nexttime
  last = present;
  accel_x = last_accel_x;
  accel_y = last_accel_y;
  accel_z = last_accel_z;
}
