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
   TODO:
   reconnect GPRS
*/

#include "config.h"
#include <TimeLib.h>
#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"
#include <ArduinoJson.h>
//#include <MemoryFree.h>

/*************************** FONA Pins ***********************************/

// GSm pins
#define GSM_RX  2
#define GSM_TX  3
#define GSM_RST 4 // pin number 16 in sim808 throug diode to ground https://cdn-shop.adafruit.com/datasheets/SIM808_Hardware+Design_V1.00.pdf max 4.3 V
SoftwareSerial fonaSS = SoftwareSerial(GSM_TX, GSM_RX);

Adafruit_FONA fona = Adafruit_FONA(GSM_RST);

StaticJsonBuffer<122> jsonBuffer;

// global - a tak nie powinno się robić
JsonObject& root = jsonBuffer.createObject();


char geodata[122];

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
Adafruit_MQTT_Publish feed = Adafruit_MQTT_Publish(&mqtt, TOPIC);

/*************************** Sketch Code ************************************/

// How many transmission failures in a row we're willing to be ok with before reset
byte txfailures = 0;
#define MAXTXFAILURES 3

void setup() {
  
  while (!Serial);

  // Watchdog is optional!
  Watchdog.enable(8000);

  Serial.begin(115200);

  Serial.println(F("GPS Tracker"));

  Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  Watchdog.reset();

  // Initialise the FONA module
  while (! FONAconnect(F(GSM_APN), F(GSM_USERNAME), F(GSM_PASSWORD))) {
    Serial.println(F("Retrying FONA GSM"));
  }
  while (! GPS()) {
    Serial.println(F("Retrying FONA GPS"));
    Watchdog.reset();
  }
  Serial.println(F("Connected to Cellular!"));

  Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  Watchdog.reset();
  
}


void loop() {
//  Serial.println(freeMemory());
  prepareData();
  // Make sure to reset watchdog every loop iteration!
  Watchdog.reset();

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  Watchdog.reset();
  // Now we can publish stuff!

  if (! feed.publish(geodata)) {
    
    Serial.println(F("Sent Failed"));
    txfailures++;
  } else {
    Serial.println(F("Sent OK!"));
    txfailures = 0;
  }

  Watchdog.reset();
  delay(DELAY*1000);  // wait a few seconds to stabilize connection
  Watchdog.reset();
  
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  byte ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print(F("Connecting to MQTT... "));

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(F("Retrying MQTT connection in 10 seconds..."));
    mqtt.disconnect();
    Watchdog.disable();
    delay(10000);  // wait 10 seconds
    Watchdog.enable(8000);
    Watchdog.reset();
  }
  Serial.println(F("MQTT Connected!"));
  Watchdog.reset();
}


void prepareData() {
  
  int ctime[6];
  float gps_data[5];
  void GPS_Data(float *fdata, int *idata);
  GPS_Data(gps_data, ctime);
  setTime(ctime[0], ctime[1], ctime[2], ctime[3], ctime[4], ctime[5]);
  time_t uxdate = now();
  int vbat;
  //if (! fona.getBattPercent(&vbat)) vbat = 0;
  if (! fona.getBattVoltage(&vbat)) vbat = 0;
  //}
  // {"tst":1476474031,"acc":1000,"_type":"location","alt":140,"lon":-90.48259734672334,"vac":10,"p":100.160530090332,"lat":38.75410327670748,"batt":100,"tid":"JT"}
  // Owntracks API: 
  // http://owntracks.org/booklet/tech/json/

    root.set("_type","location");
    root.set("acc",3);
    root.set("batt",vbat);
    root.set("cog",int(gps_data[4]));
    root.set("lat",(gps_data[0]),7);
    root.set("lon",(gps_data[1]),7);
    root.set("tid",TID);
    root.set("vel",(int(gps_data[2]) <= 2 ? 0 : int(gps_data[2])));
    root.set("tst",uxdate);
    root.printTo(geodata, 122);


}
