#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define GPS_RX_PIN 3
#define GPS_TX_PIN 2
#define LED_PIN 13

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

#define TARGET_LAT 37.7749
#define TARGET_LON -122.4194
#define THRESHOLD 100

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      float lat = gps.location.lat();
      float lon = gps.location.lng();
      float distance = gps.distanceBetween(lat, lon, TARGET_LAT, TARGET_LON);

      if (distance <= THRESHOLD) {
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
      Serial.print(lat, 6);
      Serial.print(lon, 6);
      Serial.print(distance);
      Serial.println();
    }
  }
}
