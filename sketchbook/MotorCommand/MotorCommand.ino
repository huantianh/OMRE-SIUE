//Test motors on robot

//Rear Motor Right [2] 
const int speed_2 = 8;   //PWM pin
const int dir_2 = 29;      //direction pin
const int cur_2 = A8;      //current pin


//Rear Motor Left [3] 
const int speed_3 = 9;    //PWM pin
const int dir_3 = 28;      //direction pin
const int cur_3 = A9;      //current pin


//Front Motor [1] ; 
const int speed_1 = 10 ;    //PWM pin
const int dir_1 = 27;      //direction pin
const int cur_1 = A10;      //current pin  


void setup() {
  
  Serial.begin(9600);

  //Left Motor
  pinMode(speed_2, OUTPUT);
  pinMode(dir_2, OUTPUT); //LOW=CCW HIGH=CW

  //Right Motor
  pinMode(speed_3, OUTPUT);
  pinMode(dir_3, OUTPUT);
 
  //Rear Motor
  pinMode(speed_1, OUTPUT);
  pinMode(dir_1, OUTPUT);
  
}

void loop() {
 
    if (Serial.available() > 0) {
       
      
      //Test motor 1
      analogWrite(speed_1, 100);
      digitalWrite(dir_1, HIGH);
      delay(3000);
      analogWrite(speed_1, 0);
      digitalWrite(dir_1, LOW);
      delay(1000);
      analogWrite(speed_1, 255);
      delay(3000);
      analogWrite(speed_1, 0);
      
      //Test motor 2
      analogWrite(speed_2, 100);
      digitalWrite(dir_2, HIGH);
      delay(3000);
      analogWrite(speed_2, 0);
      digitalWrite(dir_2, LOW);
      delay(1000);
      analogWrite(speed_2, 255);
      delay(3000);
      analogWrite(speed_2, 0);
      
      //Test motor 3
      analogWrite(speed_3, 100);
      digitalWrite(dir_3, HIGH);
      delay(3000);
      analogWrite(speed_3, 0);
      digitalWrite(dir_3, LOW);
      delay(1000);
      analogWrite(speed_3, 255);
      delay(3000);
      analogWrite(speed_3, 0);
  
  }

}

