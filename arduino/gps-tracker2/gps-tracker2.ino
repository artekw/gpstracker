
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

//#include <avr/pgmspace.h>

//#include <Adafruit_SleepyDog.h>
#include "config.h"
#include <TimeLib.h>


#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"
//#include "SoftReset.h"
#include "ArduinoJson.h"

//#include <MemoryFree.h>

//#include <avr/power.h>
//#include <avr/wdt.h>


/*************************** FONA Pins ***********************************/

// GSm pins
#define GSM_RX  2
#define GSM_TX  3
#define GSM_RST 4 // pin number 16 in sim808 throug diode to ground https://cdn-shop.adafruit.com/datasheets/SIM808_Hardware+Design_V1.00.pdf max 4.3 V
SoftwareSerial fonaSS = SoftwareSerial(GSM_TX, GSM_RX);
//SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(GSM_RST);
//Adafruit_FONA fona = Adafruit_FONA();
//delay(200);
//StaticJsonBuffer<122> jsonBuffer;

// global - a tak nie powinno się robić
//JsonObject& root = jsonBuffer.createObject();


uint8_t readytosend = 0;
uint16_t reporter = 0;
char geodata[125];

//char *geodatapoint = geodata;
/************ Global State (you don't need to change this!) ******************/
//const char MQTT_SERVER[] PROGMEM = AIO_SERVER;
//const char MQTT_USERNAME[] PROGMEM = AIO_USERNAME;
//const char MQTT_PASSWORD[] PROGMEM = AIO_PASSWORD;
//const char MQTT_TOPIC[] PROGMEM = TOPIC; 
//const char PHONE_GSM_APN[] PROGMEM = GSM_APN;  
//#define MQTT_CONN_KEEPALIVE 1800
//#define MQTT_FONA_QUERYDELAY 50
 
//#define MQTT_DEBUG
// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_PASSWORD);
//Adafruit_MQTT_FONA mqtt(&fona,AIO_SERVER,AIO_SERVERPORT);
// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

// FONAconnect is a helper function that sets up the FONA and connects to
// the GPRS network. See the fonahelper.cpp tab above for the source!
boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);

boolean GPS();

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
Adafruit_MQTT_Publish feed = Adafruit_MQTT_Publish(&mqtt,TOPIC);
// Adjust as necessary, in seconds.  Default to 5 minutes.


/*************************** Sketch Code ************************************/

// How many transmission failures in a row we're willing to be ok with before reset
byte txfailures = 0;
#define MAXTXFAILURES 3
//void MQTT_connect();

void setup() {
  //while (!fonaSS);

  // Watchdog is optional!
  //Watchdog.enable(8000);

  Serial.begin(115200);

  
  Serial.println(F("GPS Tracker starting ..."));

  //Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  //Watchdog.reset();
 
  // Initialise the FONA module
  while (! FONAconnect(F(GSM_APN), F(GSM_USERNAME), F(GSM_PASSWORD))) {

    Serial.println(F("Retrying FONA GSM - GPRS"));
    delay(10);
  }
     while (! GPS()) {
    Serial.println(F("Retrying FONA GPS"));
    delay(5000);
    //Watchdog.reset();
  }
  Serial.println(F("Connected to Cellular!"));
  delay(10);

  //Watchdog.reset();
  delay(10000);  // wait a few seconds to stabilize connection
  //Watchdog.reset();
  
}


void loop() {
    //delay(100); 
  //fona.flush();
    delay(100);   
  //Serial.println(freeMemory());
  
  prepareData();

    delay(100);

    while( fona.GPRSstate() != 1 ){
    //TCPavailable fona.getNetworkStatus()  fona.TCPconnected()
    
       
   
       Serial.println(F("No network ! traing to bring up ..."));
       delay(10);
    
    
       //void sendATcommand(char* ATcommand,unsigned int *timeout);
       //sendATcommand("AT+CGATT=0",1000);
       fonaSS.println("AT+CGATT=0");
       delay(1000);  // wait a few seconds to stabilize connection
       fonaSS.println("AT+CGATT=1");
       //sendATcommand("AT+CGATT=1",1000);
       delay(5000); 
     
       Serial.println(F("Enabling GPRS again !"));
       delay(10);

      
      while (! GPS()) {
      Serial.println(F("Restarting FONA GPS - not fixed"));
      delay(5000);
      }
      
    Serial.println(F("Retrying FONA GPRS/GPS after crash !"));
    delay(5000);
    Serial.println(F("Connected to Cellular again !"));
    delay(10);
   }
   Serial.println(F("Network avaliable !"));
   delay(10);
  // Make sure to reset watchdog every loop iteration!
  ////Watchdog.reset();

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  
if (readytosend != 0) {
       
   
    
    MQTT_connect();
    delay(50);

    //Watchdog.reset();
    // Now we can publish stuff!
  
    if (! feed.publish(geodata)) {
    
       Serial.println(F("Sent Failed"));
       delay(10);
       txfailures++;
    } else {
       Serial.println(F("Sent OK!"));
       delay(10);
       //delete[] geodata;
       //Serial.println(freeMemory());
       //delay(500);
       //feed.publish(geodata);
      txfailures = 0;
    readytosend = 0;
    //delete[] geodata;
    

   }
} else {

  Serial.println(F("No movement of 10 meters waiting ..."));
  delay(10);
  readytosend = 0;
  
  //delay(50);
    mqtt.ping(1);
  delay(100);    
      /*if (mqtt.ping()){
      delay(1000);
      Serial.println(F("MQTT server avaliable"));
      }else{
      delay(500);
      Serial.println(F("No MQTT server !!!"));  
      }
      */
}
  //Watchdog.reset();
  delay(DELAY*1000);  // wait a few seconds to stabilize connection
  //Watchdog.reset();

}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  
  byte ret;

  // Stop if already connected.
   if (mqtt.connected()) {
    
    return;
   }

  Serial.println(F("Connecting to MQTT... "));
      delay(10);
      while ((ret = mqtt.connect()) != 0 ) { // connect will return 0 for connected
      delay(100);
      //Serial.println(mqtt.connectErrorString(ret));
      mqtt.disconnect();
     
      Serial.println(F("Retrying MQTT connection in 2 seconds..."));
      delay(10);
      //mqtt.connect();
      delay(2000);  // wait 5 seconds

     }
   
  Serial.println(F("MQTT Connected!"));
  delay(10);
  //Watchdog.reset();
  
}


void prepareData() {

  float lastmovement_W = 0;
  float lastmovement_L = 0;
  int vbat;
  int ctime[6];
  float gps_data[5];
  if (! fona.getBattVoltage(&vbat)) vbat = 0;
  delay(100);
  void GPS_Data(float *fdata, int *idata);
  GPS_Data(gps_data, ctime);
  delay(100);
  setTime(ctime[0], ctime[1], ctime[2], ctime[3], ctime[4], ctime[5]);
  delay(100);
  time_t uxdate = now();
  delay(100);
   //if (! fona.getBattPercent(&vbat)) vbat = 0;

  //}
  // {"tst":1476474031,"acc":1000,"_type":"location","alt":140,"lon":-90.48259734672334,"vac":10,"p":100.160530090332,"lat":38.75410327670748,"batt":100,"tid":"JT"}
  // Owntracks API: 
  // http://owntracks.org/booklet/tech/json/

//        char loc[50];
        
//        sprintf(loc, "{\"longitude\":%s,\"latitude\":%s}", Lon, Lat);
   //      dtostrf(longitude, 6, 6, Lon);
   //  dtostrf(latitude, 6, 6, Lat);
   //dtostrf(hdop, 1, 1, HDOP);
    //root["_type"] = "location";
    //double_with_n_digits(
  StaticJsonBuffer<140> jsonBuffer;
  delay(100);
  JsonObject& root = jsonBuffer.createObject();
  delay(100);
    root.set("_type","location");
    root.set("acc",3);
    root.set("batt",vbat);
    root.set("cog",int(gps_data[4]));
    delay(2);
    root.set("lat",(gps_data[0]),7);
    delay(2);
    root.set("lon",(gps_data[1]),7);
    root.set("tid",TID);
    root.set("vel",(int(gps_data[2]) <= SPEED_TR ? 0 : int(gps_data[2])));
    delay(10);
    root.set("tst",uxdate);
    delay(2);
    root.printTo(geodata, 125);
    delay(100);
    //delete[] &jsonBuffer,&uxdate,&vbat,root,&ctime;
    //sprintf((gps_data[0]),%f, movement_W);
  if ((((gps_data[0]) - lastmovement_W ) > float(0.0010000)) or (((gps_data[1]) - lastmovement_L ) > float(0.0010000)) or reporter == HEARTBIT) {
  reporter=0;
  readytosend = 1;
/*
byte stat = fona.GPSstatus();
  
  if (stat < 0 or stat == 0  or stat == 1) {
    
    Serial.println(F("Not fixed!"));

   if (fona.getNetworkStatus() == 1) {
      
      float lat,lon;
      boolean gsmloc_success = fona.getGSMLoc(&lat, &lon);

    if (gsmloc_success) {
      Serial.print("GSMLoc lat:");
      Serial.println(lat, 6);
      Serial.print("GSMLoc long:");
      Serial.println(lon, 6);
    }
  }


   delay(500);
  }
*/


  /*
  
  Serial.println();
  Serial.print(F("Latitude OLD: "));
  Serial.print(lastmovement_W,7);
  Serial.println();
  Serial.print(F("Longitude OLD: "));
  Serial.print(lastmovement_L,7);
  Serial.println();
  */

  lastmovement_W = (gps_data[0]);
  delay(10);
  lastmovement_L = (gps_data[1]);
  delay(10);
  //delete[] gps_data;
  /*Serial.println();
  Serial.println(F("Latitude: "));
  Serial.print(lastmovement_W,7);
  Serial.println();
  Serial.println(F("Longitude: "));
  Serial.print(lastmovement_L,7);
  Serial.println();
  */
  } else {
     readytosend = 0;

     // reporter every 15 min - position reported even not moved
     reporter++;
     
     
  }
  Serial.println();
  Serial.print(F("Latitude OLD: "));
  delay(10);
  Serial.print(lastmovement_W,7);
  Serial.println();
  Serial.print(F("Longitude OLD: "));
  delay(10);
  Serial.print(lastmovement_L,7);
  Serial.println();
  Serial.print(F("Times checking: "));
  delay(10);
  Serial.print(HEARTBIT);
  Serial.print(F(" before reporting, heartbit now waiting: "));
  delay(10);
  Serial.println(reporter);
  Serial.println();
  //Serial.print(F("GPS status: "));
  //Serial.println(fona.GPSstatus());
  //void sendATcommand(char *ATcommand, unsigned int *timeout);
  //sendATcommand("AT+CGPSSTATUS?",500);
   
}



