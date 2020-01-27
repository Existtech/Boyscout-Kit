/* This sketch was modified by Existential Technologies for the Boyscout Kit. 
Using the Particle Boron LTE and the Adafruit Festher GPS, positional 
data can be sent to the Particle Cloud. */

#include "Particle.h"
#include "TinyGPS++.h" //make sure to include this library

SYSTEM_THREAD(ENABLED);

const unsigned long PUBLISH_PERIOD = 5000; // This is how frequent the lat/long coordinates are published 1 second = 1000
const unsigned long SERIAL_PERIOD = 5000;
const unsigned long MAX_GPS_AGE_MS = 10000; // GPS location must be newer than this to be considered valid

TinyGPSPlus gps; // The TinyGPS++ object

unsigned long lastSerial = 0;
unsigned long lastPublish = 0;
unsigned long startFix = 0;
bool validFix = false;

void setup() {// This script opens the serial ports and launches the GPS
    Serial.begin(9600);
    Serial1.begin(9600); // The feather GPS is connected to Serial1 and D6
    pinMode(D6, OUTPUT);
    digitalWrite(D6, LOW); // Settings D6 LOW powers up the GPS module
    startFix = millis();
    validFix = true;
}

void loop(){
    while (Serial1.available() > 0) {
        if (gps.encode(Serial1.read())) {
            gpsVariable();
        }
    }
}

void gpsVariable(){
    if (millis() - lastSerial >= SERIAL_PERIOD) {
        lastSerial = millis();
        char buf[120]; //this is buffer that stores the lat/long coordinates
        if (gps.location.isValid() && gps.location.age() < MAX_GPS_AGE_MS) {
            snprintf(buf, sizeof(buf), "{ \ name\:\"Particle\", \lat\:\%f\, \lon\:\%f\ }", gps.location.lat(), gps.location.lng());
            Serial.println(buf); //this is for debugging
            if (validFix) {
                validFix = false;
                unsigned long elapsed = millis() - startFix;
                Serial.printlnf("%lu milliseconds to get GPS fix", elapsed);
            }
        }
        if (Particle.connected()) {
            if (millis() - lastPublish >= PUBLISH_PERIOD) {
                lastPublish = millis();
                Particle.publish("gps", buf, PRIVATE);
            }
        }
    }

}
