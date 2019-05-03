/******************************************  Update Pos_IMU ************************************************/
float accel_x;
float accel_y;
float accel_z;
float accel_filter_x;
float accel_filter_y;
float accel_filter_z;

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

float vx;
float vy;
float vz;
float v;

float pos_x;
float pos_y;
float pos_z;
float pos;

unsigned long present       = 0;
unsigned long last          = 0;

float filter_accel;
float dt = 0.5;

void updatePos()
{
  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  // alpha = 0:1;
  accel_x = a.acceleration.x;
  accel_filter_x = alpha *  accel_x + (1 - alpha) * last_accel_x;

  accel_y = a.acceleration.y;
  accel_filter_y = 0.07 *  accel_y + (1 - 0.07) * last_accel_y;

  accel_z = a.acceleration.z;
  accel_filter_z = alpha *  accel_z + (1 - alpha) * last_accel_z;

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
  vy += accel_filter_x * dt;
  vz += accel_filter_x * dt;

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

  if (countx >= 25) //if this number exceeds 25, we can assume that velocity is zero
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

  if (county >= 25) //if this number exceeds 25, we can assume that velocity is zero
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

  if (countz >= 25) //if this number exceeds 25, we can assume that velocity is zero
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

  if (countv >= 25) //if this number exceeds 25, we can assume that velocity is zero
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

}

void comp_filter(float new_accel, float new_gyro)
{
  float filterTerm0;
  float filterTerm1;
  float filterTerm2;
  float timeConstant;

  timeConstant = 0.5; // default 1.0

  filterTerm0 = (new_accel - prev_accel) * timeConstant * timeConstant;
  filterTerm2 += filterTerm0 * dt;
  filterTerm1 = filterTerm2 + ((new_accel - prev_accel) * 2 * timeConstant) + new_gyro;
  filter_accel = (filterTerm1 * dt) + prev_accel;

  filter_accel = prev_accel;
}
/***************************************************************************************************************************************/
