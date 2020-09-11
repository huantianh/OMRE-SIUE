/*******************************              GIVING COMMAND                *******************************************/
void parseCommand()
{
  char command = rcv_buffer[0]; // our first byte tells us the command char is equivalent to byte
  switch (command)
  {
    ////////////////////////////////////////////////////////////////             ENCODER
    case 'E':
    case 'e':
      int encoderNum;
      sscanf(&rcv_buffer[1], " %d \r", &encoderNum);
      float counts;
      counts = encoderCounts[encoderNum];
      Serial.println(encoderCounts[encoderNum]);
      break;

    ////////////////////////////////////////////////////////////////             MOTOR PWM
    case 'M':
    case 'm':
      int  motorNumber;
      int  motorPWM;
      sscanf(&rcv_buffer[1], " %d %d \r", &motorNumber, &motorPWM);
      pidSwitch                             = '0';
      motor(motorNumber, motorPWM);

      PRINT_ULTRASOUND                      = '0';
      PRINT_IR                              = '0';
      PRINT_RPM                             = '1';
      PRINT_MCUR                            = '0';
      break;

    ////////////////////////////////////////////////////////////////             MOTOR CURRENT
    case 'c':
    case 'C':
      int curNum;
      sscanf(&rcv_buffer[1], " %d \r", &curNum);
      MCURSwitch                            = '1';

      PRINT_RPM                             = '0';
      PRINT_ULTRASOUND                      = '0';
      PRINT_IR                              = '0';
      PRINT_MCUR                            = '1';
      Serial.println(m_cur[curNum]);
      break;

    ////////////////////////////////////////////////////////////////             ULTRASOUND
    case 'u':
    case 'U':
      int ultrasonicNumber;
      sscanf(&rcv_buffer[1], " %d \r", &ultrasonicNumber);
      ultrasonicSwitch = '1';

      PRINT_ULTRASOUND                      = '1';
      PRINT_IR                              = '0';
      PRINT_RPM                             = '0';
      PRINT_MCUR                            = '0';

      Serial.println(m_US[ultrasonicNumber]);
      break;

    ////////////////////////////////////////////////////////////////              INFARED
    case 'i':
    case 'D':

      int infraredNumber;
      sscanf(&rcv_buffer[1], " %d \r", &infraredNumber);
      IRSwitch = '1';

      PRINT_ULTRASOUND                      = '0';
      PRINT_IR                              = '1';
      PRINT_RPM                             = '0';
      PRINT_MCUR                            = '0';
      Serial.println (m_IR[infraredNumber]);
      break;

    ////////////////////////////////////////////////////////////////              MOTOR RPM
    case 'v':
    case 'V':

      int rpm0;
      int rpm1;
      int rpm2;
      sscanf(&rcv_buffer[1], "%d %d %d \r", &rpm0, &rpm1, &rpm2);
      pidSwitch                             = '1';
      MCURSwitch                            = '1';

      PRINT_ULTRASOUND                      = '0';
      PRINT_IR                              = '0';
      PRINT_RPM                             = '1';
      PRINT_MCUR                            = '1';

      rpm[0] = rpm0;
      rpm[1] = rpm1;
      rpm[2] = rpm2;

      for (int i = 0; i < 3; i++)
      {
        if (rpm[i] > 0)
        {
          pwm_dir[i]   = '0';
        }
        if (rpm[i] < 0)
        {
          pwm_dir[i]   = '1';;
        }
      }

      rpm_setpoint[0] = rpm0;
      rpm_setpoint[1] = rpm1;
      rpm_setpoint[2] = rpm2;
      break;

    ////////////////////////////////////////////////////////////////              Check  RPM
    case 'r':
    case 'R':
      int rpmNum;
      sscanf(&rcv_buffer[1], " %d \r", &rpmNum);
      PRINT_RPM                             = '1';
      Serial.println(rpmValues[rpmNum]);
      //      Serial.print("  ,  ");
      //      Serial.print(rpmValues[1]);
      //      Serial.print("  ,  ");
      //      Serial.println(rpmValues[2]);

      break;

    ////////////////////////////////////////////////////////////////            STOP
    case 's':
    case 'S':

      pidSwitch                             = '0';
      ultrasonicSwitch                      = '0';
      IRSwitch                              = '0';
      MCURSwitch                            = '0';
      printSwitch                           = '0';

      for (int i = 0; i < 3; i++)
      {
        motor(i, 0);
        rpm_setpoint[i] = 0;
      }
      break;

    ////////////////////////////////////////////////////////////////             PRINT DATA
    case 'p':
    case 'P':

      printSwitch                           = '1';
      break;

    ////////////////////////////////////////////////////////////////             Move Robot Forward
    case 'f':
    case 'F':

      pidSwitch                             = '1';
      PRINT_ULTRASOUND                      = '0';
      PRINT_IR                              = '0';
      PRINT_RPM                             = '1';
      PRINT_MCUR                            = '0';
      rpm_setpoint[0] = 0;
      rpm_setpoint[1] = 100;
      rpm_setpoint[2] = -100;
      break;

    /////////////////////////////////////////////////////////////////            Move Robot Backward
    case 'b':
    case 'B':

      pidSwitch                             = '1';
      PRINT_ULTRASOUND                      = '0';
      PRINT_IR                              = '0';
      PRINT_RPM                             = '1';
      rpm_setpoint[0] = 0;
      rpm_setpoint[1] = -100;
      rpm_setpoint[2] = 100;
      break;
  }
}

//supporting function to print doubles precicely
void printDouble( double val, unsigned int precision) {
  // prints val with number of decimal places determine by precision
  // NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
  // example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

  Serial.print (int(val));  //prints the int part
  Serial.print("."); // print the decimal point
  unsigned int frac;
  if (val >= 0)
    frac = (val - int(val)) * precision;
  else
    frac = (int(val) - val ) * precision;
  Serial.println(frac, DEC) ;
}
