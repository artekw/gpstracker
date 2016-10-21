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
//#include "Time.h"
#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"
#include <ArduinoJson.h>
//#include <stdlib.h>

/*************************** FONA Pins ***********************************/

// GSm pins
#define GSM_RX  2
#define GSM_TX  3
#define GSM_RST 4 // pin number 16 in sim808 throug diode to ground https://cdn-shop.adafruit.com/datasheets/SIM808_Hardware+Design_V1.00.pdf max 4.3 V
SoftwareSerial fonaSS = SoftwareSerial(GSM_TX, GSM_RX);
//SoftwareSerial *FONASERIAL = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(GSM_RST);
//Adafruit_FONA *FONA = &fona;

//int ctime[6];
//char Lat[10];
//char Lon[10];
//char Alt[10];
//uint16_t Course;
//uint16_t Speed;
//char Date[14];


//byte offset = 0;

StaticJsonBuffer<125> jsonBuffer;

// global - a tak nie powinno siÄ™ robiÄ‡
JsonObject& root = jsonBuffer.createObject();
//JsonObject *ROOT = &root;

char geodata[125];

/************ Global State (you don't need to change this!) ******************/

// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT);
//Adafruit_MQTT_FONA *MQTT(&mqtt);
// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

// FONAconnect is a helper function that sets up the FONA and connects to
// the GPRS network. See the fonahelper.cpp tab above for the source!
boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);

boolean GPS();

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
Adafruit_MQTT_Publish feed = Adafruit_MQTT_Publish(&mqtt, TOPIC);
//Adafruit_MQTT_Publish *FEED = &feed;
/*************************** Sketch Code ************************************/

// How many transmission failures in a row we're willing to be ok with before reset
byte txfailures = 0;
#define MAXTXFAILURES 3

void setup() {
  
  while (!Serial);

  // Watchdog is optional!
  //Watchdog.enable(10000);

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


  //Serial.print(F("\nSending "));
  //Serial.print(F("..."));
  
  if (! feed.publish(geodata)) {
    
    Serial.println(F("Sent Failed"));
    txfailures++;
  } else {
    //Serial.println(F("OK!"));
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

  //Serial.print(F("Connecting to MQTT... "));

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    //Serial.println(mqtt.connectErrorString(ret));
    Serial.println(F("Retrying MQTT connection in 10 seconds..."));
    mqtt.disconnect();
    delay(10000);  // wait 8 seconds
  }
  Serial.println(F("MQTT Connected!"));
  //if (! Serial.println(geodata)) {
  //  Serial.println(F("Failed"));
  //  } else {
  //  Serial.println(F("data OK!"));
    
 // }
}

void prepareData() {
  
  int ctime[6];
  float gps_data[5];
  void GPS_Data(float *fdata, int *idata);
  GPS_Data(gps_data, ctime);
  //dtostrf(gps_data[0], 6, 6, &Lat[0]);
  //dtostrf(gps_data[1], 6, 6, &Lon[0]);
  //dtostrf(gps_data[2], 6, 2, Speed);
  //Speed = gps_data[2];
  //dtostrf(gps_data[3], 6, 2, Alt);
  //dtostrf(gps_data[4], 6, 2, Course);
  //Course = gps_data[4];
  // prepare data to unixtime
  setTime(ctime[0], ctime[1], ctime[2], ctime[3], ctime[4], ctime[5]);
  // timezone
  //adjustTime(offset * SECS_PER_HOUR);
   // unixtime to string
  //snprintf(&Date[0], 14, "%lu", now());
  time_t uxdate = now();
  //Serial.println(test);

  //(int(gps_data[2]) >= 2 ? 0 : int(gps_data[2]));
  
  //int _speed = int(gps_data[2]);
  //if (_speed < SPEED_TR) _speed = 0;
  int vbat;
  //if (! fona.getBattPercent(&vbat)) vbat = 0;
  if (! fona.getBattVoltage(&vbat)) vbat = 0;
  //fona.getBattVoltage
        //} else {
         // Serial.print(F("VPct = ")); Serial.print(vbat); Serial.println(F("%"));
  //}
  // {"tst":1476474031,"acc":1000,"_type":"location","alt":140,"lon":-90.48259734672334,"vac":10,"p":100.160530090332,"lat":38.75410327670748,"batt":100,"tid":"JT"}
  // Owntracks API: 
  // http://owntracks.org/booklet/tech/json/
  //root["_type"] = "location";
 root.set("_type","location");
  //root["acc"] = 10;
 root.set("acc",3);
  //root["alt"] = atof(Alt);
 // root.set("alt",atof(Alt));
  //root["batt"] = 100;
 root.set("batt",vbat);
  //root["cog"] = atof(Course);
 root.set("cog",int(gps_data[4]));
  //root["lat"] = Lat;
 root.set("lat",(gps_data[0]),7);
  //root["lon"] = Lon;
 root.set("lon",(gps_data[1]),7);
  //root["tid"] = TID;
 root.set("tid",TID);
  //root["vel"] = _speed;
 root.set("vel",(int(gps_data[2]) <= 2 ? 0 : int(gps_data[2])));
  //root["tst"] = Date;
  //ROOT->set("tst",atol(Date));
 root.set("tst",uxdate);
 root.printTo(geodata, 125);




}
