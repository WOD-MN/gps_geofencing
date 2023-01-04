#include <TinyGPS++.h>
#include <LoRa.h>

// create a TinyGPS++ object
TinyGPSPlus gps;

void setup() {
  // initialize serial communication
  Serial.begin(9600);

  // initialize the LoRa module
  LoRa.begin(433E6);
}

void loop() {
  // check for incoming GPS data
  if (Serial.available()) {
    // read and decode the data
    gps.encode(Serial.read());

    // check if a new fix is available
    if (gps.location.isValid()) {
      // get the current latitude and longitude
      double lat = gps.location.lat();
      double lng = gps.location.lng();

      // calculate the distance to the geofence location
      double distance = haversine(lat, lng, geofenceLat, geofenceLng);

      // check if the distance is within the desired range
      if (distance < 100) {
        // inside the geofence
        // transmit GPS data over LoRa network
        LoRa.beginPacket();
        LoRa.print("Entered geofence: ");
        LoRa.print(lat, 6);
        LoRa.print(", ");
        LoRa.print(lng, 6);
        LoRa.endPacket();
      } else {
        // outside the geofence
        // transmit GPS data over LoRa network
        LoRa.beginPacket();
        LoRa.print("Left geofence: ");
        LoRa.print(lat, 6);
        LoRa.print(", ");
        LoRa.print(lng, 6);
        LoRa.endPacket();
      }
    }
  }
}

double haversine(double lat1, double long1, double lat2, double long2) {
  // convert to radians
  lat1 = lat1 * PI / 180.0;
  long1 = long1 * PI / 180.0;
  lat2 = lat2 * PI / 180.0;
  long2 = long2 * PI / 180.0;

  double dlong = long2 - long1;
  double dlat = lat2 - lat1;

  double ans = pow(sin(dlat/2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlong/2), 2);
  ans = 2 * asin(sqrt(ans));

  // earth's radius in miles
  double R = 3956;
  ans = ans * R;

  return ans;
}
