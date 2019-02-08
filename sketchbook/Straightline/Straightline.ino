//Test motors on robot

//Rear Motor Right [2] 
const int speed_2 = 10;   //PWM pin
const int dir_2 = 27;      //direction pin
const int cur_2 = A10;      //current pin


//Rear Motor Left [3] 
const int speed_3 = 11;    //PWM pin
const int dir_3 = 26;      //direction pin
const int cur_3 = A11;      //current pin


//Front Motor [1] ; 
const int speed_1 = 9 ;    //PWM pin
const int dir_1 = 28;      //direction pin
const int cur_1 = A9;      //current pin  

void setup() {

  //Left Motor
  pinMode(speed_2, OUTPUT);
  pinMode(dir_2, OUTPUT);

  //Right Motor
  pinMode(speed_3, OUTPUT);
  pinMode(dir_3, OUTPUT);
 
  //Rear Motor
  pinMode(speed_1, OUTPUT);
  pinMode(dir_1, OUTPUT);

}

void loop() {

//Test motor 1
analogWrite(speed_1, 55);
//analogWrite(speed_1, 50);
digitalWrite(dir_1, HIGH);
delay(20);
analogWrite(speed_1, 15);
//digitalWrite(dir_1, LOW);
delay(3000);
//analogWrite(speed_1, 255);
//delay(3000);
//analogWrite(speed_1, 0);

//Test motor 2
//analogWrite(speed_2, 100);
//digitalWrite(dir_2, HIGH);
//delay(3000);
//analogWrite(speed_2, 0);
//digitalWrite(dir_2, LOW);
//delay(1000);
//analogWrite(speed_2, 255);
//delay(3000);
//analogWrite(speed_2, 0);
//
////Test motor 3
//analogWrite(speed_3, 100);
//digitalWrite(dir_3, HIGH);
//delay(3000);
//analogWrite(speed_3, 0);
//digitalWrite(dir_3, LOW);
//delay(1000);
//analogWrite(speed_3, 255);
//delay(3000);
//analogWrite(speed_3, 0);

}
