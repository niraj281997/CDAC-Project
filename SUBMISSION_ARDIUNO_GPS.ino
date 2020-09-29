#include <AWS_IOT.h>
#include <WiFi.h>
#include <ThingerESP32.h>  
#include "types.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <SoftwareSerial.h>
TinyGPSPlus gps;
int x;
static int A=0;
AWS_IOT hornbill;

HardwareSerial SerialGPS(1);
struct GpsDataState_t {
  double originLat = 0;
  double originLon = 0;
  double originAlt = 0;
  double distMax = 0;
  double dist = 0;
  double altMax = -999999;
  double altMin = 999999;
  double spdMax = 0;  
  double prevDist = 0;
};
GpsDataState_t gpsState = {};

//GpsDataState_t gpsState = {};
#define TASK_SERIAL_RATE 100
uint32_t nextSerialTaskTs = 0;

#define USERNAME "NIRAJ"
#define DEVICE_ID "ESP32"
#define DEVICE_CREDENTIAL "ESP__32"

#define SSID "vulcan's wifi"
#define SSID_PASSWORD "vulcan@1997"
// Thing details foe AWS
char HOST_ADDRESS[]="a2blxd0ewsut8v-ats.iot.us-east-1.amazonaws.com";
char CLIENT_ID[]= "ESP_thing";
char TOPIC_NAME[]= "$aws/things/ESP_thing/shadow/update";

ThingerESP32 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);
bool val = LOW;

String inString = "";    // string to hold input

//Connection status
int status = WL_IDLE_STATUS;
// Payload array to store thing shadow JSON document
char payload[512];
// Counter for iteration
int counter = 0;
void setup() {
  WiFi.disconnect(true);
  pinMode(2, OUTPUT);
   // Serial.begin(115200);
    Serial.begin(115200,SERIAL_8N1,3,1);
    SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
  thing.add_wifi(SSID,  SSID_PASSWORD);
while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Wifi network: ");
    Serial.println(SSID);
    status = WiFi.begin(SSID,  SSID_PASSWORD);
    delay(5000);
  }
  Serial.println("Connected to Wifi!");
  if(hornbill.connect(HOST_ADDRESS,CLIENT_ID)== 0) {
    Serial.println("Connected to AWS, bru");
    delay(1000);
  }
  else {
    Serial.println("AWS connection failed, Check the HOST Address");
    while(1);
  }
  // digital pin control example (i.e. turning on/off a light, a relay, configuring a parameter, etc)
  thing["led"] << digitalPin(2);

thing["location"] >> [](pson& out){
      out["lat"] = gps.location.lat();
      out["lon"] = gps.location.lng();
};
  // more details at http://docs.thinger.io/arduino/
}//nfn



void loop() {


 
thing.handle();


  while (Serial.available()>0)
  {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
 x=inString.toInt();
   // Serial1.println("X=");Serial1.print(x);
Serial.println(x);
    if(x==1){
       
     //digitalWrite(2, HIGH);
      while(1)
{
 
digitalWrite(2, HIGH);
val=1;
val=Serial2.read();
if(val){
 
  while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }

    if (gps.satellites.value() > 4)
    {
        gpsState.dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), gpsState.originLat, gpsState.originLon);

        if (gpsState.dist > gpsState.distMax && abs(gpsState.prevDist - gpsState.dist) < 50) {
            gpsState.distMax = gpsState.dist;
        }

        gpsState.prevDist = gpsState.dist;

        if (gps.altitude.meters() > gpsState.altMax) {
            gpsState.altMax = gps.altitude.meters();
        }

        if (gps.speed.mps() > gpsState.spdMax) {
            gpsState.spdMax = gps.speed.mps();
        }

        if (gps.altitude.meters() < gpsState.altMin) {
            gpsState.altMin = gps.altitude.meters();
        }
    }

    if (nextSerialTaskTs < millis()) {
        delay(2000);
        if(A==12)
        {
          float aa=gps.location.lat();
          float bb=gps.location.lng();                                                                                                                                                                        
         //sprintf(payload,"{\"It's an Emergency please reach to the below Latitude and Longitude\"\"\t\t\t%lf \n\t\t\t%lf\"}",(gps.location.lat(), 6),(gps.location.lng(), 6));
         sprintf(payload,"{\"It's an Emergency please reach to the below Latitude and Longitude\"\"\t\t\t%lf \n\t\t\t%lf\"}",aa,bb);
           if(hornbill.publish(TOPIC_NAME,payload) == 0)
                  {
                  Serial.println("Message published successfully");
                  }
             else {
                     Serial.println("Message was not published");
                   }
A++;
        }
        Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
        Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
        Serial.print("ALT=");  Serial.println(gps.altitude.meters());
        Serial.print("Sats=");  Serial.println(gps.satellites.value());
        Serial.print("DST: ");
        Serial.println(gpsState.dist,1);
          A++;
        nextSerialTaskTs = millis() + TASK_SERIAL_RATE;
    }
 
}
 
  }
}else{

  Serial.println("Data not available !!!!");
}

}


 
 
   
}


