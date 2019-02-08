#include <SharpIR.h>

#define ir1 A0
#define ir2 A1
#define ir3 A2
#define ir4 A3
#define ir5 A4
#define ir6 A5
#define model 1080
// ir: the pin where your sensor is attached
// model: an int that determines your sensor:  1080 for GP2Y0A21Y
//                                            20150 for GP2Y0A02Y
//                                            (working distance range according to the datasheets)

SharpIR SharpIR1(ir1, model);
SharpIR SharpIR2(ir2, model);
SharpIR SharpIR3(ir3, model);
SharpIR SharpIR4(ir4, model);
SharpIR SharpIR5(ir5, model);
SharpIR SharpIR6(ir6, model);

void setup() {
  
  Serial.begin(9600);
}

void loop() {
  delay(2000);   

  unsigned long pepe1=millis();  // takes the time before the loop on the library begins

  int dis1=SharpIR1.distance();
  int dis2=SharpIR2.distance();
  int dis3=SharpIR3.distance();
  int dis4=SharpIR4.distance();
  int dis5=SharpIR5.distance();
  int dis6=SharpIR6.distance();



  Serial.print("Mean distance: ");
  Serial.println(dis1);
  Serial.print("Mean distance: ");
  Serial.println(dis2);
  Serial.print("Mean distance: ");
  Serial.println(dis3);
  Serial.print("Mean distance: ");
  Serial.println(dis4);
  Serial.print("Mean distance: ");
  Serial.println(dis5);
  Serial.print("Mean distance: ");
  Serial.println(dis6);
  
  unsigned long pepe2=millis()-pepe1;  // the following gives you the time taken to get the measurement
  Serial.print("Time taken (ms): ");
  Serial.println(pepe2);  

}
