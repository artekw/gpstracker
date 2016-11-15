
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


#include "config.h"
#include <TimeLib.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"
#include "ArduinoJson.h"
#include <SoftwareSerial.h>
#define FONA_DEFAULT_TIMEOUT_MS 750
#define MQTT_CONN_KEEPALIVE 1800
#define MQTT_FONA_INTERAVAILDELAY 200
#define MQTT_FONA_QUERYDELAY 750
/*************************** FONA Pins ***********************************/

// GSm pins
#define GSM_RX  2
#define GSM_TX  3
#define GSM_RST 4 // pin number 16 in sim808 throug diode to ground https://cdn-shop.adafruit.com/datasheets/SIM808_Hardware+Design_V1.00.pdf max 4.3 V
SoftwareSerial fonaSS = SoftwareSerial(GSM_TX, GSM_RX);
//oftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(GSM_RST);


int readytosend = 0;
uint16_t reporter = 0;
uint32_t logCounter = 0; 
char geodata[125];

//char *geodatapoint = geodata;
/************ Global State (you don't need to change this!) ******************/
//const char MQTT_SERVER[] PROGMEM = AIO_SERVER;
//const char MQTT_USERNAME[] PROGMEM = AIO_USERNAME;
//const char MQTT_PASSWORD[] PROGMEM = AIO_PASSWORD;
//const char MQTT_TOPIC[] PROGMEM = TOPIC; 
//const char PHONE_GSM_APN[] PROGMEM = GSM_APN;  

 
//#define MQTT_DEBUG
// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_PASSWORD);
//Adafruit_MQTT_FONA mqtt(&fona,AIO_SERVER,AIO_SERVERPORT);

// FONAconnect is a helper function that sets up the FONA and connects to
// the GPRS network. See the fonahelper.cpp tab above for the source!
boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);

boolean GPS();
//reset
void(* resetFunc) (void) = 0;
/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
Adafruit_MQTT_Publish feed = Adafruit_MQTT_Publish(&mqtt,TOPIC);
// Adjust as necessary, in seconds.  Default to 5 minutes.


/*************************** Sketch Code ************************************/


void setup() {
  
  Serial.begin(115200);
  while (!Serial);
  delay(20);
  fonaSS.begin(4800);
  while (!fonaSS);
  delay(20);
  fona.begin(fonaSS);
  
  Serial.println(F("GPS Tracker starting ..."));
  
  delay(20000);  // wait a few seconds to stabilize connection

  while ( !fonaSS);
  // Initialise the FONA module
  while (! FONAconnect(F(GSM_APN), F(GSM_USERNAME), F(GSM_PASSWORD))) {
    
    Serial.println(F("Retrying FONA GPRS ..."));
    delay(20000);
    while ( !fonaSS);
}

    delay(10000); 
    while ( !fonaSS);
    byte trying =0;
    while (! GPS() or trying == 10) {
    Serial.println(F("Retrying FONA GPS ..."));
    //Serial.println(trying);
    delay(15000);
    while ( !fonaSS);
    trying++;
   }
  if (trying == 10)resetFunc();
  Serial.println(F("Connected to Cellular!"));


  
  delay(10000);  // wait a few seconds to stabilize connection
  
  
}


void loop() {
    while ( !fonaSS);
    delay(50); 
    fonaSS.flush();
    delay(50); 
    Serial.flush();
    delay(50);   


//-------------main function not generate when no GPS, GPRS, GSM

do{
   clearbuffer();
   while (!fonaSS);
   fona.deleteSMS(1);

   Serial.println(F("Network avaliable GPS fixed GPRS connected, processing ..."));
   delay(50);
   clearbuffer();

   prepareData();
   delay(100);
    
  
if (readytosend == 1) {
      
    clearbuffer();
    MQTT_connect();
    delay(50);

    
    // Now we can publish stuff!
    clearbuffer();
    byte check =0;
  do {
    while (!fonaSS);
    if (! feed.publish(geodata)) {
       Serial.println(F("Sent Failed"));
       delay(1000);
          check++;

       } else {
       Serial.println(F("Sent OK!"));
       delay(50);
       
       //Serial.println(freeMemory());
       check = 3;
       readytosend = 0;

    //delete[] geodata;
    }
  } while (check < 3);



} else {

  Serial.println(F("No movement of 10 meters waiting ..."));
  delay(50);
  readytosend = 0;
  
      delay(50);
      while ( !fonaSS);
      mqtt.ping(3);
      delay(100); 
          while (!fonaSS);
          if (!fona.TCPconnected()){
              return;
           }   
      /*if (mqtt.ping()){
      delay(1000);
      Serial.println(F("MQTT server avaliable"));
      }else{
      delay(500);
      Serial.println(F("No MQTT server !!!"));  
      }
      */
}
  
  clearbuffer();


  
  delay(DELAY*1000);  // wait a few seconds to stabilize connection

    while ( !fonaSS);
    } while(fonaSS && fona.GPRSstate() == 1  && fona.GPSstatus() > 1  );

   Serial.println(F("No GPS fixed or GPRS connected, waiting for restart ..."));
   delay(1000);
    while ( !fonaSS);
    if( fona.GPRSstate() != 1 or fona.getRSSI() < 6){
           Serial.println(F("No network !!! traing to bring up ..."));
           delay(50);
          //asm volatile ( "jmp 0"); 
          //albo tak ale reset przed setup void(* resetFunc) (void) = 0;  wywolanie resetFunc();
          fonaSS.end();
          resetFunc();
      }           
 
      while (!fonaSS);
      if (fona.GPSstatus() <= 1){
        Serial.println(F("Restarting FONA GPS - not fixed ..."));
        delay(50);
         byte retry=0;
         while (! GPS() or retry == 10) {
           Serial.println(F("Trying up FONA GPS ..."));
           delay(10000);
           while (!fonaSS);
           retry++;
        }

        delay(5000);
        while (!fonaSS);
        if (fona.GPSstatus() <= 1) {
          fonaSS.end();
          resetFunc();//asm volatile ( "jmp 0"); 
        }
        
      }
   
  //OCR0A = 0xAF;
  //TIMSK0 |= _BV(OCIE0A);
}


void clearbuffer(){
   // Read all available serial input to flush pending data.
    uint16_t timeoutloop = 0;
    while (!fonaSS);
    fonaSS.peek();
    while (timeoutloop++ < 60) {
        while(fonaSS.available()) {
            fonaSS.read();
            timeoutloop = 0;  // If char was received reset the timer
        }
        delay(1);
    }
}
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.



void MQTT_connect() {
  while ( !fonaSS);
  byte ret;

  // Stop if already connected.
   if (mqtt.connected()) {
    
    return;
   }

  Serial.println(F("Connecting to MQTT ... "));
      delay(50);
      while (!fonaSS);
      while ((ret = mqtt.connect()) != 0 ) { // connect will return 0 for connected
      delay(50);
      //Serial.println(mqtt.connectErrorString(ret));
      
      mqtt.disconnect();
     
      Serial.println(F("Retrying MQTT connection in 2 seconds ..."));
      delay(50);
      //mqtt.connect();
      delay(2000);  // wait 5 seconds
      while (!fonaSS);
     }
  // Configure timer0 compare interrupt to run and decrease the log counter every millisecond.

  Serial.println(F("MQTT Connected!"));
  delay(50);

  
}


void prepareData() {

  static float lastmovement_W = 0;
  static float lastmovement_L = 0;
  int vbat;
  int ctime[6];
  float gps_data[5];
  while (!fonaSS);
  if (! fona.getBattVoltage(&vbat)) vbat = 0;
  delay(50);
    void GPS_Data(float *fdata, int *idata);
  //GPS_Data(gps_data, ctime);
  while (!fonaSS);
  GPS_Data(&gps_data[0], &ctime[0]);
  delay(50);
  setTime(ctime[0], ctime[1], ctime[2], ctime[3], ctime[4], ctime[5]);
  //setTime(&ctime[0], &ctime[1], &ctime[2], &ctime[3], &ctime[4], &ctime[5]);
  delay(20);
  time_t uxdate = now();
  delay(20);
   //if (! fona.getBattPercent(&vbat)) vbat = 0;

  //}
  // {"tst":1476474031,"acc":1000,"_type":"location","alt":140,"lon":-90.48259734672334,"vac":10,"p":100.160530090332,"lat":38.75410327670748,"batt":100,"tid":"JT"}
  // Owntracks API: 
  // http://owntracks.org/booklet/tech/json/


  StaticJsonBuffer<140> jsonBuffer;
  //memset(jsonBuffer, 0, sizeof(jsonBuffer));
  //delay(50);
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
    delay(50);
    root.set("tst",uxdate);
    delay(2);
    memset(geodata, 0, sizeof(geodata));
    delay(50);
    root.printTo(geodata, 125);
    delay(50);
    //delete[] &jsonBuffer,&uxdate,&vbat,root,&ctime;
    //sprintf((gps_data[0]),%f, movement_W);
  if ((((gps_data[0]) - lastmovement_W ) > float(0.0010000)) or (((gps_data[1]) - lastmovement_L ) > float(0.0010000)) or reporter == HEARTBIT) {
  reporter=0;
  readytosend = 1;

//byte stat = fona.GPSstatus();
  
  //if (stat < 0 or stat == 0  or stat == 1) {
    
    //Serial.println(F("Not fixed!"));

   //if (fona.getNetworkStatus() == 1) {
      
      //float lat,lon;
      //fona.getGSMLoc(&lat, &lon);

   
      //Serial.print("GSMLoc lat:");
      //Serial.println(lat, 7);
      //Serial.print("GSMLoc long:");
      //Serial.println(lon, 7);

  //}


  delay(100);
  lastmovement_W = (gps_data[0]);
  delay(50);
  lastmovement_L = (gps_data[1]);
  delay(50);
  
  } else {
      
      readytosend = 0;
      
     // reporter every xxx min - position reported even not moved
     ++reporter;
     
  }       

  Serial.println();
  Serial.print(F("   Latitude OLD: "));
  delay(50);
  Serial.print(lastmovement_W,7);
  Serial.println();
  Serial.print(F("   Longitude OLD: "));
  delay(50);
  Serial.print(lastmovement_L,7);
  Serial.println();
  Serial.println();
  Serial.print(F("   Times checking: "));
  delay(50);
  Serial.print(HEARTBIT);
  Serial.print(F(" before reporting heartbit, waiting: "));
  delay(50);
  Serial.println(reporter);
  Serial.println();
  Serial.println();
  

}



