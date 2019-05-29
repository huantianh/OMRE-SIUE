/*******************************              GIVING COMMAND                *******************************************/
void parseCommand()
{
  char command = rcv_buffer[0]; // our first byte tells us the command char is equivalent to byte
  switch (command)
  {
    ///////////////////////////////////////////////////////////////             ENCODER
    case 'E':
    case 'e':
      int encoderNum;
      sscanf(&rcv_buffer[1], " %d \r", &encoderNum);
      float counts;
      counts = encoderCounts[encoderNum];
      Serial.println(encoderCounts[encoderNum]);
      break;

    ////////////////////////////////////////////////////////////////             MOTOR
    case 'M':
    case 'm':
      int  motorNumber;
      int  motorPWM;
      int  motorDirection;
      sscanf(&rcv_buffer[1], " %d %d \r", &motorNumber, &motorPWM);
      motor(motorNumber, motorPWM);
      break;

    ////////////////////////////////////////////////////////////////             ULTRASOUND
    case 'u':
    case 'U':
      int ultrasonicNumber;
      sscanf(&rcv_buffer[1], " %d \r", &ultrasonicNumber);
      ultrasonicSwitch = '1';
      Serial.println(cm_US[ultrasonicNumber]);
      break;

    ////////////////////////////////////////////////////////////////              INFARED
    case 'i':
    case 'I':

      int infraredNumber;
      sscanf(&rcv_buffer[1], " %d \r", &infraredNumber);
      IRSwitch = '1';
      Serial.println (cm_IR[infraredNumber]);
      break;

    ////////////////////////////////////////////////////////////////              ENTER RPM_GOAL
    case 'v':
    case 'V':

      int rpm0;
      int rpm1;
      int rpm2;
      sscanf(&rcv_buffer[1], "%d %d %d \r", &rpm0, &rpm1, &rpm2);

      rpm_setpoint[0] = rpm0;
      rpm_setpoint[1] = rpm1;
      rpm_setpoint[2] = rpm2;
      break;

    ////////////////////////////////////////////////////////////////              PID Switch ON/OFF
    case 'p':
    case 'P':
      rpm_setpoint[0] = 0;
      rpm_setpoint[1] = 0;
      rpm_setpoint[2] = 0;
      motor(0, 0);
      motor(1, 0);
      motor(2, 0);
      sscanf(&rcv_buffer[1], " %c \r", &pidSwitch);
      break;

    ////////////////////////////////////////////////////////////////              Check  RPM
    case 'r':
    case 'R':
      int rpmNum;
      sscanf(&rcv_buffer[1], " %d \r", &rpmNum);
      printDouble(rpmValues[rpmNum], 1000000000);
      break;

    //    ///////////////////////////////////////////////////////////////               ENTER GAINS K
    //    case 'k':
    //    case 'K':
    //      char  pValue[20];
    //      char  iValue[20];
    //      sscanf(&rcv_buffer[1], " %s %s \r", &pValue, &iValue);
    //      char *ptr;
    //      for (int i = 0; i < 3; i++)
    //      {
    //        Kp[i] = strtod(pValue, &ptr);
    //        Ki[i] = strtod(iValue, &ptr);
    //      }
    //      break;

    ////////////////////////////////////////////////////////////////              STOP
    case 's':
    case 'S':
      printSwitch = 0;
      for (int i = 0; i < 3; i++)
      {
        motor(i, 0);
        rpm_setpoint[i] = 0;
      }
      break;

    ////////////////////////////////////////////////////////////////             PRINT DATA
    case 'a':
    case 'A':
      sscanf(&rcv_buffer[1], " %c \r", &printSwitch);
      break;

    ////////////////////////////////////////////////////////////////             Move Robot Forward
    case 'f':
    case 'F':

      rpm_setpoint[0] = 0;
      rpm_setpoint[1] = -50;
      rpm_setpoint[2] = 50;
      break;

    /////////////////////////////////////////////////////////////////            Move Robot Backward
    case 'b':
    case 'B':

      rpm_setpoint[0] = 0;
      rpm_setpoint[1] = 50;
      rpm_setpoint[2] = -50;
      break;
      //    default:
      //      Serial.println("Error: Serial input incorrect");
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
