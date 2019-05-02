/******************************************  Update Pos_IMU ************************************************/
float accel_x;
float accel_filter_x;
float gyro_x;
float last_accel_x;
int countx   = 0;

double alpha = 0.1;
double prev_accel;

double vx;
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

  if (( accel_filter_x  <= 0.05) && ( accel_filter_x >= -0.05)) //Discrimination window applied
  {
    accel_filter_x = 0; // to the X axis acceleration
  }

  present = micros() * 0.000001;
  dt = (present - last);

  vx += accel_filter_x * dt;

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
    countx = 0;
  }

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
