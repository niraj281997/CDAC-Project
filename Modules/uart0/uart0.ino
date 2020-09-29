#include <SoftwareSerial.h>

int i, x;
void setup()
 {
  Serial.begin(115200);
  Serial2.begin(115200,SERIAL_8N1,3,1);
 }
 void loop()
 {
  while (Serial2.available()>0)
  {
  x=Serial2.read();
   // Serial1.println("X=");Serial1.print(x);
Serial.println(x);
//    if(x==45){
//      while(1)
//      {
//
//             if(i==0)
//             {
//               Serial.println("X=");Serial.print(x);
//             digitalWrite(2, HIGH);
//                 i++;
//              }
//             if(i==1)
//              {
//              digitalWrite(2, HIGH);
//              delay(500);
//              digitalWrite(2, LOW);
//               }
//         }
//    }
  }
 }
