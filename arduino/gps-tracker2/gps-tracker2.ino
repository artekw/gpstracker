/***************************************************
  Adafruit MQTT Library FONA Example

  Designed specifically to work with the Adafruit FONA
  ----> http://www.adafruit.com/products/1946
  ----> http://www.adafruit.com/products/1963
  ----> http://www.adafruit.com/products/2468
  ----> http://www.adafruit.com/products/2542

  These cellular modules use TTL Serial to communicate, 2 pins are
  required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

 /*
  * TODO:
  * reconetc GPRS
  */

#include "config.h"
#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"

/*************************** FONA Pins ***********************************/

// GSm pins
#define GSM_RX  2
#define GSM_TX  3
#define GSM_RST 4
SoftwareSerial fonaSS = SoftwareSerial(GSM_TX, GSM_RX);

Adafruit_FONA fona = Adafruit_FONA(GSM_RST);

float gps_data[3];
char Lat[15];
char Lon[15];
char Speed[15];

/************ Global State (you don't need to change this!) ******************/

// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

// FONAconnect is a helper function that sets up the FONA and connects to
// the GPRS network. See the fonahelper.cpp tab above for the source!
boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);

boolean GPS();

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish latitude = Adafruit_MQTT_Publish(&mqtt, "feeds/geo/latitude");
Adafruit_MQTT_Publish longitude = Adafruit_MQTT_Publish(&mqtt, "feeds/geo/longitude");
Adafruit_MQTT_Publish speed_kph = Adafruit_MQTT_Publish(&mqtt, "feeds/geo/speed");

/*************************** Sketch Code ************************************/

// How many transmission failures in a row we're willing to be ok with before reset
uint8_t txfailures = 0;
#define MAXTXFAILURES 3

void setup() {
  while (!Serial);

  // Watchdog is optional!
  //Watchdog.enable(8000);
  
  Serial.begin(115200);

  Serial.println(F("GPS Tracker"));

  Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  Watchdog.reset();
  
  // Initialise the FONA module
  while (! FONAconnect(F(GSM_APN), F(GSM_USERNAME), F(GSM_PASSWORD))) {
    Serial.println("Retrying FONA");
  }
  while (! GPS()) {
    Serial.println("Retrying FONA");
  }
  Serial.println(F("Connected to Cellular!"));

  Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  Watchdog.reset();
}

void loop() {
  prepareData();
  // Make sure to reset watchdog every loop iteration!
  Watchdog.reset();

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  Watchdog.reset();
  // Now we can publish stuff!
  
  
  Serial.print(F("\nSending "));
  Serial.print("...");
  if (! latitude.publish(Lon) or 
      ! longitude.publish(Lat) or
      ! speed_kph.publish(Speed)) {
    Serial.println(F("Failed"));
    txfailures++;
  } else {
    Serial.println(F("OK!"));
    txfailures = 0;
  }

  Watchdog.reset();  

  // ping the server to keep the mqtt connection alive, only needed if we're not publishing
  //if(! mqtt.ping()) {
  //  Serial.println(F("MQTT Ping failed."));
  //}

}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}

void prepareData() {
  void GPS_Data(float *pdata);
  GPS_Data(&gps_data[0]);
  dtostrf(gps_data[1], 6, 6, Lon);
  dtostrf(gps_data[0], 6, 6, Lat);
  dtostrf(gps_data[2], 3, 3, Speed);
}
