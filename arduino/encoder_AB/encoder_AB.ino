#define forward LOW
#define backward HIGH

int pos = 0;

const int interruptPin = 2;
const int pin_PWM = 10;
const int pin_Dir = 27;


void count_encoder()
{
  if (digitalRead(5) == HIGH)
    pos++;
  else
    pos--;
   Serial.println(pos);
}

void run_motor(int pwm)
{
  if (pwm >= 255)
  {
    pwm = 255;
  }
  if (pwm <= -255)
  {
    pwm = -255;
  }
  
  if (pwm > 0)
  {
    digitalWrite(pin_Dir, forward);
    analogWrite(pin_PWM, pwm);
  }
  if (pwm < 0)
  {
    digitalWrite(pin_Dir, backward);
    analogWrite(pin_PWM, abs(pwm));
  }
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_Dir, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), count_encoder, FALLING);
}

void loop() {

  run_motor(0);
 
}
