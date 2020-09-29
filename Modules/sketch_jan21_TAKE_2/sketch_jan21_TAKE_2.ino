#include <ThingerESP32.h>  
#include "types.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <SoftwareSerial.h>
TinyGPSPlus gps;
int x;
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

ThingerESP32 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);
bool val = LOW;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    Serial2.begin(115200,SERIAL_8N1,16,17);

  thing.add_wifi(SSID, SSID_PASSWORD);

  // digital pin control example (i.e. turning on/off a light, a relay, configuring a parameter, etc)
  thing["led"] << digitalPin(LED_BUILTIN);

thing["location"] >> [](pson& out){
      out["lat"] = gps.location.lat();
      out["lon"] = gps.location.lng();
};
  // more details at http://docs.thinger.io/arduino/
}//nfn

void loop() {



  while (Serial2.available()>0)
  {
  x=Serial2.read();
   // Serial1.println("X=");Serial1.print(x);
Serial.println(x);
    if(x){
       SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
     //digitalWrite(2, HIGH);
      while(1)
{
  thing.handle();
   digitalWrite(2, HIGH);
val=1;
//val=Serial2.read();
if(val){
 
  while (SerialGPS.available() > 0) 
      {
        gps.encode(SerialGPS.read());
    }

    if (gps.satellites.value() > 4) {
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
        Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
        Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
        Serial.print("ALT=");  Serial.println(gps.altitude.meters());
        Serial.print("Sats=");  Serial.println(gps.satellites.value());
        Serial.print("DST: ");
        Serial.println(gpsState.dist,1);

        nextSerialTaskTs = millis() + TASK_SERIAL_RATE;
    }

}
 
  }
}

}


 
 
   
}
