const int trigPin1 = 42;
const int echoPin1 = 43;

const int trigPin2 = 44;
const int echoPin2 = 45;

const int trigPin3 = 30;
const int echoPin3 = 31;

const int trigPin4 = 32;
const int echoPin4 = 33;

const int trigPin5 = 34;
const int echoPin5 = 35;

const int trigPin6 = 36;
const int echoPin6 = 37;

const int trigPin7 = 38;
const int echoPin7 = 39;

const int trigPin8 = 40;
const int echoPin8 = 41;

long duration1, distance1;
long duration2, distance2;
long duration3, distance3;
long duration4, distance4;
long duration5, distance5;
long duration6, distance6;
long duration7, distance7;
long duration8, distance8;

void setup() {

  Serial.begin(9600);

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);

  pinMode(trigPin5, OUTPUT);
  pinMode(echoPin5, INPUT);
  
  pinMode(trigPin6, OUTPUT);
  pinMode(echoPin6, INPUT);

  pinMode(trigPin7, OUTPUT);
  pinMode(echoPin7, INPUT);
  
  pinMode(trigPin8, OUTPUT);
  pinMode(echoPin8, INPUT);
  
}

void loop(){
  

digitalWrite(trigPin1, LOW);
delayMicroseconds(2);
digitalWrite(trigPin1, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin1, LOW);
duration1 = pulseIn(echoPin1, HIGH);
distance1 = duration1*0.034/2;
Serial.print("distance1: ");
Serial.println(distance1);

digitalWrite(trigPin2, LOW);
delayMicroseconds(2);
digitalWrite(trigPin2, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin2, LOW);
duration2 = pulseIn(echoPin2, HIGH);
distance2 = duration2*0.034/2;
Serial.print("distance2: ");
Serial.println(distance2);

digitalWrite(trigPin3, LOW);
delayMicroseconds(2);
digitalWrite(trigPin3, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin3, LOW);
duration3 = pulseIn(echoPin3, HIGH);
distance3 = duration3*0.034/2;
Serial.print("distance3: ");
Serial.println(distance3);

digitalWrite(trigPin4, LOW);
delayMicroseconds(2);
digitalWrite(trigPin4, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin4, LOW);
duration4 = pulseIn(echoPin4, HIGH);
distance4 = duration4*0.034/2;
Serial.print("distance4: ");
Serial.println(distance4);

digitalWrite(trigPin5, LOW);
delayMicroseconds(2);
digitalWrite(trigPin5, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin5, LOW);
duration5 = pulseIn(echoPin5, HIGH);
distance5 = duration5*0.034/2;
Serial.print("distance5: ");
Serial.println(distance5);

digitalWrite(trigPin6, LOW);
delayMicroseconds(2);
digitalWrite(trigPin6, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin6, LOW);
duration6 = pulseIn(echoPin6, HIGH);
distance6 = duration6*0.034/2;
Serial.print("distance6: ");
Serial.println(distance6);

digitalWrite(trigPin7, LOW);
delayMicroseconds(2);
digitalWrite(trigPin7, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin7, LOW);
duration7 = pulseIn(echoPin7, HIGH);
distance7 = duration7*0.034/2;
Serial.print("distance7: ");
Serial.println(distance7);

digitalWrite(trigPin8, LOW);
delayMicroseconds(2);
digitalWrite(trigPin8, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin8, LOW);
duration8 = pulseIn(echoPin8, HIGH);
distance8 = duration8*0.034/2;
Serial.print("distance8: ");
Serial.println(distance8);

delay(3000);
}


