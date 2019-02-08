#define ENCODER_0INTERRUPT_PIN 18 // pin that interrupts on both rising and falling of A and B channels of encoder
#define ENCODER_1INTERRUPT_PIN 19
#define ENCODER_2INTERRUPT_PIN 20

#define FORWARD 0
#define BACKWARDS 1
#define INFRARED_SENSOR_0 A0
#define INFRARED_SENSOR_1 A1
#define INFRARED_SENSOR_2 A2
#define INFRARED_SENSOR_3 A3
volatile long encoderCounts[] = {0,0,0};
bool motorDir[3] = {FORWARD,FORWARD,FORWARD};

const int motorPWMPins[3]            = {8,10,9};
const int motorDirPins[3]            = {29,28,27};
const int ultrasonicSensorTrigPins[] = {30,32,34,36,38,40};
const int ultrasonicSensorEchoPins[] = {31,33,35,37,39,41};
const int infraredSensorPins[] = {0,1,2,3};
//const int infaredSensorPins
char rcv_buffer[64];
void motor(int,int,bool);


void setup() {
  
  Serial.begin(115200);
  for(int i =0;i<6;i++)
  {
   pinMode(ultrasonicSensorTrigPins[i], OUTPUT);
   pinMode(ultrasonicSensorEchoPins[i], INPUT);
  }

  //INFRARED SENSORS
  pinMode(INFRARED_SENSOR_0,INPUT);
  pinMode(INFRARED_SENSOR_1,INPUT);
  pinMode(INFRARED_SENSOR_2,INPUT);
  pinMode(INFRARED_SENSOR_3,INPUT);
  //Left Motor
  pinMode(motorPWMPins[0], OUTPUT);
  pinMode(motorDirPins[0], OUTPUT); //LOW=CCW HIGH=CW

  //Right Motor
  pinMode(motorPWMPins[1], OUTPUT);
  pinMode(motorDirPins[1], OUTPUT);
 
  //Rear Motor
  pinMode(motorPWMPins[2], OUTPUT);
  pinMode(motorDirPins[2], OUTPUT);
  buffer_Flush(rcv_buffer);
  while (! Serial);
 attachInterrupt(18,encoder0_ISR,CHANGE);
 attachInterrupt(19,encoder1_ISR,CHANGE);
 attachInterrupt(20,encoder2_ISR,CHANGE);
//  
}

void loop() {
    
    receiveBytes();
  
}

void encoder0_ISR()
{
  noInterrupts();
  if(!motorDir[0])
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
  if(!motorDir[1])
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
  if(!motorDir[2])
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

    motorDir[motorNumber] = dir;
  
  digitalWrite(motorDirPins[motorNumber],dir);
  // could input check here for less than 255
  analogWrite(motorPWMPins[motorNumber], pwm);
  Serial.print("Motor: ");
  Serial.print(motorNumber);
  Serial.print(" Speed: ");
  Serial.print(pwm);
  Serial.print(" Direction: ");
  Serial.println(dir);
  
}

void receiveBytes()
{
  static byte index = 0;
  char terminator = '\r';
  while(Serial.available() > 0)
  {
    rcv_buffer[index] = Serial.read();
    if(rcv_buffer[index] == terminator)
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
    char command = rcv_buffer[0];

    //uint16_t value = analogRead(INFRARED_SENSOR_0);
        //  double distance = get_IR(value);
    
    switch(command)
    {
      case 'E':
      case 'e':
        Serial.print("Encoder 0: ");
        Serial.println(encoderCounts[0]);
        Serial.print("Encoder 1: ");
        Serial.println(encoderCounts[1]);
        Serial.print("Encoder 2: ");
        Serial.println(encoderCounts[2]);
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
        long duration,cm,inches;
        sscanf(&rcv_buffer[1], " %d \r",&ultrasonicNumber);
   
        digitalWrite(ultrasonicSensorTrigPins[ultrasonicNumber], LOW);
        delayMicroseconds(5);
        digitalWrite(ultrasonicSensorTrigPins[ultrasonicNumber], HIGH);
        delayMicroseconds(10);
        digitalWrite(ultrasonicSensorTrigPins[ultrasonicNumber], LOW);
        
        duration = pulseIn(ultrasonicSensorEchoPins[ultrasonicNumber], HIGH);
        cm = (duration/2) / 29.1;
        inches = (duration/2) / 74; 
  
        Serial.print(inches);
        Serial.print("in, ");
        Serial.print(cm);
        Serial.print("cm");
        Serial.println();
        break;

        case 'i':
        case 'I':
          uint16_t value;
          int infraredNumber;
          double distance;
        
          sscanf(&rcv_buffer[1], " %d \r",&ultrasonicNumber);
          value = analogRead(ultrasonicNumber);
          distance = get_IR(value);
          
          Serial.print (distance);
          Serial.println (" cm");
          Serial.println ();
          break;
        
      default:
      Serial.println("Error: Serial input incorrect");
        
      
    }
}

double get_IR(uint16_t value){
  if (value < 16)  value = 16;
  //return 4800.0 / (value - 1120.0);
  return 4800.0 / (value - 20.0);
    
}
