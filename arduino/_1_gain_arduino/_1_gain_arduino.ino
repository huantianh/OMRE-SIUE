
#include <HardwareSerial.h>
#include <SimpleTimer.h>



#define ENCODER_0INTERRUPT_PIN 5// pin  18that interrupts on both rising and falling of A and B channels of encoder
#define ENCODER_1INTERRUPT_PIN 4 // pin 19 https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/ to see the encoder pin number
#define ENCODER_2INTERRUPT_PIN 3 // pin 20

#define FORWARD 0
#define BACKWARD 1
#define INFRARED_SENSOR_0 A0
#define INFRARED_SENSOR_1 A1
#define INFRARED_SENSOR_2 A2
#define INFRARED_SENSOR_3 A3
#define VELOCITY_TIME 15         //Every # miliseconds we update our rpm of wheels



volatile long encoderCounts[] = {
  0,0,0}; // variables accesed inside of an interrupt need to be volatile
bool motorDir[3] = {
  FORWARD,FORWARD,FORWARD};

const int motorPWMPins[3]            = {
  8,9,10};
const int motorDirPins[3]            = {
  29,28,27};
const int ultrasonicSensorTrigPins[] = {
  30,32,34,36,38,40};
const int ultrasonicSensorEchoPins[] = {
  31,33,35,37,39,41};
const int infraredSensorPins[] = {
  0,1,2,3};

double Kp = 0.5; 
double Ki = 0.007;
double Kd = 0;

double sum[3]        = {0,0,0};
double error[3]      = {0,0,0};
double setpoint[3]   = {0,0,0};
double deri[3]   = {0,0,0};
double prevError[3]   = {0,0,0};
double pwmValue[3]   = {0,0,0};
double rpms[3]  = {0,0,0};

unsigned long lastTime[3]   = {
  0,0,0};
unsigned long timeChange[3] = {
  0,0,0};

double velocityValues[3]   = {
  0,0,0};
float rpmValues[3]        = {
  0,0,0};
long pastEncoderValues[3]  = {
  0,0,0};
unsigned long pastTimes[3] = {
  0,0,0};// millis() works for up to 50days! we'll need an unsigned long for it


char rcv_buffer[64];  // holds commands recieved
char TXBuffer[64];    // temp storage for large data sent 
void motor(int,int,bool);
SimpleTimer rpmUpdateTimer;
void updateRPM();
double changeInEncoders;
double changeInRevolutions;
double changeInTimeSeconds;
char pidSwitch = '1';



void setup() {

  Serial.begin(115200);
  for(int i =0;i<6;i++)
  {
    pinMode(ultrasonicSensorTrigPins[i], OUTPUT);
    pinMode(ultrasonicSensorEchoPins[i], INPUT);
  }

  rpmUpdateTimer.setInterval(VELOCITY_TIME,updateRPM);
  //INFRARED SENSORS
  pinMode(INFRARED_SENSOR_0,INPUT);
  pinMode(INFRARED_SENSOR_1,INPUT);
  pinMode(INFRARED_SENSOR_2,INPUT);
  pinMode(INFRARED_SENSOR_3,INPUT);
  // Motor
  for(int i =0; i<3;i++)
  {
    pinMode(motorPWMPins[i], OUTPUT);
    pinMode(motorDirPins[i], OUTPUT); //LOW=CCW HIGH=CW
  }

  pinMode(18,INPUT_PULLUP);
  pinMode(19,INPUT_PULLUP);
  pinMode(20,INPUT_PULLUP);


  buffer_Flush(rcv_buffer);

  while (! Serial);

  attachInterrupt(ENCODER_0INTERRUPT_PIN,encoder0_ISR,CHANGE);
  attachInterrupt(ENCODER_1INTERRUPT_PIN,encoder1_ISR,CHANGE);
  attachInterrupt(ENCODER_2INTERRUPT_PIN,encoder2_ISR,CHANGE);


}

void loop() {
  // this is our timer called every millisecond defined by VELOCITY_TIME (at top) to update our rpm 
  rpmUpdateTimer.run();

  // determines if we have any serial commands and interpruts them
  receiveBytes();

  // proportional integral controller

  if(pidSwitch == '1')
  {
    pi();
  }

}

void updateRPM() {

  for( int i=0; i<3;i++)
  {
    // linear velocity velocityValues[i] = (((encoderCounts[i]-pastEncoderValues[i])*0.08567979964)/((millis()-pastTimes[i])*.001));
    changeInEncoders = encoderCounts[i] - pastEncoderValues[i];
    changeInTimeSeconds = ((millis()-pastTimes[i])*.001);// *.001 to convert to seconds
    changeInRevolutions = changeInEncoders/2249;

    rpmValues[i] = (changeInRevolutions/(changeInTimeSeconds))*60; // *60 to get Revolutions per MINUTE
    //if(changeInEncoders >0)
    //Serial.println(changeInEncoders);

    // update our values to be used next time around
    pastTimes[i]= millis();
    pastEncoderValues[i]=encoderCounts[i];
    //printDouble(rpmValues[0],90000000);

  }
}

//                                         PI loop
void pi() 
{
  //////////////////////// motor 0 
   if(setpoint[0] != 0)
   {
      //updateRPM();
      timeChange[0] = (millis() - lastTime[0]);
      lastTime[0] = millis();
      //integral
      error[0] = setpoint[0] -rpmValues[0];
      sum[0] = (sum[0] +(error[0]*(double)timeChange[0]));
      //derivative
      deri[0] = error[0] - prevError[0];
      prevError[0] = error[0];        
      pwmValue[0] = (Kp * error[0]) + (Ki * sum[0]) + (Kd * deri[0]); 
            
      
      //condition for wide up integral
      if(error[0] = 0)
      { 
        sum[0] = 0;
      }
      if(error[0] = 30)
      {
        sum[0] = 0;
      }
   
      if(pwmValue[0] < 0)
      {
        motor(0,pwmValue[0]*-1,0);
      } 
      else
      {
        motor(0,pwmValue[0],1);
      }
      
      delay(15); 
      
        Serial.print(millis()*0.001);
        Serial.print("  ,  ");
        //Serial.print("rpm0 ");
        Serial.print(rpmValues[0]);
        Serial.print("  ,  ");
        //Serial.print("rpm1 ");
        Serial.print(rpmValues[1]);
        Serial.print("  ,  ");
        //Serial.print("rpm2 ");
        Serial.println(rpmValues[2]); 
   }
   else
   {
      error[0] = 0;
      sum[0]   = 0;
      deri[0] = 0;
      prevError[0] = 0;
      motor(0,0,0);
   }
   
////////////////////////////// motor 1 
   if(setpoint[1] != 0)
   {         
      timeChange[1] = (millis() - lastTime[1]);
      lastTime[1] = millis();
      error[1] = setpoint[1] -rpmValues[1];
      sum[1] = (sum[1] +(error[1]*(double)timeChange[1]));
      pwmValue[1] = (Kp * error[1]) + (Ki * sum[1]);
      
      if(pwmValue[1] < 0)
      {
        motor(1,pwmValue[1]*-1,0);
      } 
      else
      {
        motor(1,pwmValue[1],1);
      }
       
       delay(15); 
       
        Serial.print(millis()*0.001);
        Serial.print("  ,  ");
        //Serial.print("rpm0 ");
        Serial.print(rpmValues[0]);
        Serial.print("  ,  ");
        //Serial.print("rpm1 ");
        Serial.print(rpmValues[1]);
        Serial.print("  ,  ");
        //Serial.print("rpm2 ");
        Serial.println(rpmValues[2]); 
   }
   else
   {
        error[1] = 0;
        sum[1]   = 0;
        motor(1,0,0);
   }
   
////////////////////////////// motor 2    
  if(setpoint[2] != 0)
  {  
      timeChange[2] = (millis() - lastTime[2]);
      lastTime[2] = millis();
      error[2] = setpoint[2] -rpmValues[2];
      sum[2] = (sum[2] +(error[2]*(double)timeChange[2]));
      pwmValue[2] = (Kp * error[2]) + (Ki * sum[2]);
      
      if(pwmValue[2] < 0)
      {
        motor(2,pwmValue[2]*-1,0);
      } 
      else
      {
        motor(2,pwmValue[2],1);
      }
       
       delay(15); 
       
        Serial.print(millis()*0.001);
        Serial.print("  ,  ");
        //Serial.print("rpm0 ");
        Serial.print(rpmValues[0]);
        Serial.print("  ,  ");
        //Serial.print("rpm1 ");
        Serial.print(rpmValues[1]);
        Serial.print("  ,  ");
        //Serial.print("rpm2 ");
        Serial.println(rpmValues[2]);   
  }
  else
  {
       error[2] = 0;
       sum[2]   = 0;
       motor(2,0,0);
  }   
}


void encoder0_ISR() // encoder0 interrupt service routine 
{
  noInterrupts();
  if(motorDir[0])
  {
    encoderCounts[0]++;

  }
  else
  {
    encoderCounts[0]--;
  }
  interrupts();
}
void encoder1_ISR()
{
  noInterrupts();
  if(motorDir[1])
  {
    encoderCounts[1]++;
  }
  else
  {
    encoderCounts[1]--;
  }
  interrupts();
}
void encoder2_ISR()
{
  noInterrupts();
  if(motorDir[2])
  {
    encoderCounts[2]++;
  }
  else
  {
    encoderCounts[2]--;
  }
  interrupts();
}

void motor(int motorNumber, int pwm, bool dir)
{
  //dir = !dir;                              // This is to ensure positive RPM is CCW
  motorDir[motorNumber] = dir;

  digitalWrite(motorDirPins[motorNumber],dir);
  // could input check here for less than 255
  analogWrite(motorPWMPins[motorNumber], pwm);
  //  Serial.print("Motor: ");
  //  Serial.print(motorNumber);
  //  Serial.print(" Speed: ");
  //  Serial.print(pwm);
  //  Serial.print(" Direction: ");
  //  Serial.println(dir);

}

void receiveBytes()
{
  static byte index = 0;
  char terminator = '\r'; // what tells us our command is done
  while(Serial.available() > 0)
  {
    rcv_buffer[index] = Serial.read(); // read in our serial commands
    if(rcv_buffer[index] == terminator) // main loop for processing our command
    {
      index = 0;
      parseCommand();
      buffer_Flush(rcv_buffer);
    }
    else
    {
      index++;
      if(index >= 64)
      {
        Serial.println("buffer overflow");
        index = 0;
        buffer_Flush(rcv_buffer);
      }
    }
  }

}

void buffer_Flush(char *ptr)
{
  for(int i = 0; i < 64; i++)
  {
    ptr[i] = 0;
  }
}


void parseCommand()
{
  char command = rcv_buffer[0]; // our first byte tells us the command char is equivalent to byte

  //uint16_t value = analogRead(INFRARED_SENSOR_0);
  //  double distance = get_IR(value);
  switch(command)
  {
  case 'E':
  case 'e':
    int encoderNum;
    sscanf(&rcv_buffer[1], " %d \r",&encoderNum);
    long counts;
    counts = encoderCounts[encoderNum];
    //itoa(encoderCounts[encoderNum],TXBuffer,10);   // serial.print can not handle printing a 64bit int so we turn it  into a string
    Serial.println(counts);
    break;

  case 'M':
  case 'm':
    int  motorNumber;
    int  motorPWM;
    int motorDirection;

    sscanf(&rcv_buffer[1], " %d %d %d \r",&motorNumber, &motorPWM, &motorDirection);
    motor(motorNumber,motorPWM,motorDirection);
    break;
 
  case 'u':
  case 'U':
    int ultrasonicNumber;
    long duration,cm,start;
    //duration = -60; 
    sscanf(&rcv_buffer[1], " %d \r",&ultrasonicNumber);
    digitalWrite(ultrasonicSensorTrigPins[ultrasonicNumber], LOW);
    //delayMicroseconds(5);
    digitalWrite(ultrasonicSensorTrigPins[ultrasonicNumber], HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasonicSensorTrigPins[ultrasonicNumber], LOW);
    start = micros();
    
    while(digitalRead(ultrasonicSensorEchoPins[ultrasonicNumber]) == LOW);
    start = micros();
    
    while(micros()-start <= 6000)
    {
      if(digitalRead(ultrasonicSensorEchoPins[ultrasonicNumber]) == LOW)
      {
        duration = micros()-start;
        break;
      }
      //duration = micros()-start;
    }
    
    
    //duration = pulseIn(ultrasonicSensorEchoPins[ultrasonicNumber], HIGH);
    cm = (duration/2) / 29.1;
    //Serial.println(duration);
    //inches = (duration/2) /  
    Serial.println(cm);
    break;

  case 'i':
  case 'I':
    uint16_t value;
    int infraredNumber;
    double distance;

    sscanf(&rcv_buffer[1], " %d \r",&infraredNumber);
    value = analogRead(infraredNumber);
    distance = get_IR(value);
    Serial.println (distance);
    break;
  case 'v':
  case 'V':

    int rpm0;
    int rpm1;
    int rpm2;
    sscanf(&rcv_buffer[1], "%d %d %d \r",&rpm0,&rpm1,&rpm2);
    //Serial.println(rpm0);
    //Serial.println(rpm1);
    //Serial.println(rpm2);

    rpms[0] = (double)(rpm0/10);
    rpms[1] = (double)(rpm1/10);
    rpms[2] = (double)(rpm2/10);

   for(int i = 0;i<3;i++)
   {
     // when the setpoint is in a 30 +/- range do not set the sum to 0, aka if major velocity change set your sum
     // to 0. if small then don't change it
     if(!(setpoint[i]+ 20 >= rpms[i] && setpoint[i]-20 <= rpms[i]))
     {
            //error[i] = 0;
            sum[i]   = 0;
     }
   }
    
    setpoint[0] = rpms[0];
    setpoint[1] = rpms[1];
    setpoint[2] = rpms[2];
    break;
    
  case 'p':
  case 'P':
   
    setpoint[0] = 0;
    setpoint[1] = 0;
    setpoint[2] = 0;
    motor(0,0,0);
    motor(1,0,0);
    motor(2,0,0);
    sscanf(&rcv_buffer[1], " %c \r",&pidSwitch);
    break;
    
  case 'r':
  case 'R':
    int rpmNum;
    sscanf(&rcv_buffer[1], " %d \r",&rpmNum);
    printDouble(rpmValues[rpmNum],1000000000);
   break;
  
  case 'k':
  case 'K':
     char  pValue[20];
     char  iValue[20];
     char  dValue[20];
     sscanf(&rcv_buffer[1], " %s %s %s \r",&pValue,&iValue,&dValue);
     char *ptr;
     Kp = strtod(pValue,&ptr);
     Ki = strtod(iValue,&ptr);
     Kd = strtod(dValue,&ptr);
     break;

  }
}

double get_IR(uint16_t value){
  if (value < 16)  value = 16;
  //return 4800.0 / (value - 1120.0);
  return 4800.0 / (value - 20.0);

}


//supporting function to print doubles precicely 
void printDouble( double val, unsigned int precision){
  // prints val with number of decimal places determine by precision
  // NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
  // example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

  Serial.print (int(val));  //prints the int part
  Serial.print("."); // print the decimal point
  unsigned int frac;
  if(val >= 0)
    frac = (val - int(val)) * precision;
  else
    frac = (int(val)- val ) * precision;
  Serial.println(frac,DEC) ;
} 



