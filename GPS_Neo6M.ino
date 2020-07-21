#include "SoftwareSerial.h"
#include "TinyGPS++.h"

TinyGPSPlus gps;

SoftwareSerial ss(6, 7);

void setup() {
    Serial.begin(9600);
    ss.begin(9600);
}

void loop() {
    while(ss.available() > 0) {
        if(gps.encode(ss.read())) {
            if(gps.location.isUpdated()) {
                Serial.println(String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6));
            }
            delay(100);
        }
    }
}
