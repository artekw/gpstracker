
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

/*************************** FONA Params ***********************************/

//#undef FONA_DEFAULT_TIMEOUT_MS
//#define FONA_DEFAULT_TIMEOUT_MS 600
//#undef MQTT_CONN_KEEPALIVE
//#define MQTT_CONN_KEEPALIVE 1800
//#undef MQTT_FONA_INTERAVAILDELAY
//#define MQTT_FONA_INTERAVAILDELAY 200
//#undef MQTT_FONA_QUERYDELAY
//#define MQTT_FONA_QUERYDELAY 600

/*************************** FONA Pins ***********************************/

#define GSM_RX  2
#define GSM_TX  3
#define GSM_RST 4 // pin number 16 in sim808 throug diode to ground https://cdn-shop.adafruit.com/datasheets/SIM808_Hardware+Design_V1.00.pdf max 4.3 V
SoftwareSerial fonaSS = SoftwareSerial(GSM_TX, GSM_RX);
//oftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(GSM_RST);


int readytosend = 0;
uint16_t reporter = 0;
char geodata[125];



/************ Global State (you don't need to change this!) ******************/
//const __FlashStringHelper *MQTT_TOPIC;
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


/*************************** Sketch Code ************************************/


void setup() {
  
  Serial.begin(115200);
  while (!Serial);
  fonaSS.begin(9600);
  while (!fonaSS);
  //delay(20);
  //fonaSS.println("ATZ");
  fona.begin(fonaSS);
  delay(100);
  //while (!fonaSS);
  //fonaSS.println("AT&F1");
  //delay(500);
  //fonaSS.println("AT&W0");
  //delay(500);
  //clearbuffer();
#ifdef DEBUG
  Serial.println(F("GPS Tracker starting ..."));
#endif
#ifndef DEBUG
  Serial.println(F("Debug is off"));
#endif

  delay(10000);  // wait a few seconds to stabilize connection

  while ( !fonaSS);
  // Initialise the FONA module
  while (! FONAconnect(F(GSM_APN), F(GSM_USERNAME), F(GSM_PASSWORD))) {
#ifdef DEBUG   
    Serial.println(F("Retrying FONA GPRS ..."));
#endif
    delay(15000);
    while ( !fonaSS);
}

    delay(10000); 
    while ( !fonaSS);
    byte trying =0;
    while (! GPS() ) {
#ifdef DEBUG
    Serial.println(F("Retrying FONA GPS ..."));
#endif
    delay(15000);
    while ( !fonaSS);
    trying++;
    if (trying == 20)resetFunc();
   }
#ifdef DEBUG
  Serial.println(F("Connected to Cellular!"));
#endif

  
  delay(10000);  // wait a few seconds to stabilize connection
  
  
}


void loop() {
    while ( !fonaSS);
    fonaSS.flush();
    delay(50); 
    Serial.flush();
    delay(50);   


//-------------main function not generate when no GPS, GPRS, GSM

do{
   //clearbuffer();
   delay(DELAY*1000);  // wait a few seconds to stabilize connection
   while (!fonaSS);
   fona.deleteSMS(1);
   delay(500);
#ifdef DEBUG
   Serial.println(F("Network avaliable GPS fixed GPRS connected, processing ..."));
   delay(50);
#endif
   //clearbuffer();
   prepareData();
       
  
if (readytosend == 1) {
      
    //clearbuffer();
    MQTT_connect();
    //delay(50);

    
    // Now we can publish stuff!
    //clearbuffer();
    int check =3;
  do {
    while (!fonaSS);
    //clearbuffer();
    if (! feed.publish(geodata)) {
#ifdef DEBUG
       Serial.println(F("Sent Failed"));
       delay(50);
#endif
          check--;
          //clearbuffer();
          MQTT_connect();
         if (check == 0) return;
         delay(1000);
       } else {
#ifdef DEBUG
       Serial.println(F("Sent OK!"));
       delay(50);
#endif       
       //Serial.println(freeMemory());
       check = 0;
       readytosend = 0;
       if (check == 0) return;
       delay(1000);
    }
  } while (check != 0);



} else {
#ifdef DEBUG
  Serial.print(F("No movement of ")); 
  Serial.print(MOVEMENT);
  Serial.println(F(" meters waiting ..."));
  delay(50);
#endif
  readytosend = 0;
  
      
      while ( !fonaSS);
      mqtt.ping(3);
      delay(50); 
          while (!fonaSS);
          if (!fona.TCPconnected()){
              return;
           }   
}
  
  //clearbuffer();
  

  
  

    while ( !fonaSS);
    } while(fonaSS && fona.GPRSstate() == 1  && fona.GPSstatus() > 1  );
#ifdef DEBUG
   Serial.println(F("No GPS fixed or GPRS connected, waiting for restart ..."));
#endif
   delay(1000);
    while ( !fonaSS);
    if( fona.GPRSstate() != 1 ){
#ifdef DEBUG
           Serial.println(F("No network !!! traing to bring up ..."));
           delay(50);
#endif
          //asm volatile ( "jmp 0"); 
          //albo tak ale reset przed setup void(* resetFunc) (void) = 0;  wywolanie resetFunc();
          fonaSS.end();
          resetFunc(); //reset
      }           
 
      while (!fonaSS);
      if (fona.GPSstatus() <= 1){
#ifdef DEBUG
        Serial.println(F("Restarting FONA GPS - not fixed ..."));
         delay(50);
#endif         
         byte retry=0;
         while (! GPS() ) {
#ifdef DEBUG
           Serial.println(F("Trying up FONA GPS ..."));
#endif
           delay(10000);
           while (!fonaSS);
           retry++;
            if (retry == 10) {
             fonaSS.end();
             resetFunc();//asm volatile ( "jmp 0"); //reset
            }
         }
      }
   

}


/*
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
*/
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.



void MQTT_connect() {
  while ( !fonaSS);
  byte ret;

  // Stop if already connected.
   if (mqtt.connected()) {
    
    return;
   }
#ifdef DEBUG
  Serial.println(F("Connecting to MQTT ... "));
      delay(50);
#endif
      while (!fonaSS);
      byte test=0;
      while ((ret = mqtt.connect()) != 0 ) { // connect will return 0 for connected
      /*
      switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
      }
      */
      delay(50);
      //Serial.println(mqtt.connectErrorString(ret));
      
      mqtt.disconnect();
#ifdef DEBUG     
      Serial.println(F("Retrying MQTT connection in 5 seconds ..."));
#endif
      delay(5000);  // wait 2 seconds
      while (!fonaSS);
      test ++;
            if (test == 20) {
             fonaSS.end();
             resetFunc();//asm volatile ( "jmp 0"); 
            }
      
     }
#ifdef DEBUG
  Serial.println(F("MQTT Connected!"));
  delay(50);
#endif  
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
    while (!fonaSS);
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
  float converter[2];
  converter[0]=gps_data[0];
  converter[1]=gps_data[1];
   
  float movement = 0;
  movement = distanceCoordinates(converter[0],converter[1],lastmovement_W,lastmovement_L);
  if ((int)movement > 1000 or (int)movement < 0  ) movement = MOVEMENT + 1.0; //1000 means more than 200 km/h, it is added before first initialosation of last loc.
  StaticJsonBuffer<125> jsonBuffer;
  //memset(jsonBuffer, 0, sizeof(jsonBuffer));
  //delay(50);
  JsonObject& root = jsonBuffer.createObject();
  delay(100);
  
    
    root.set("_type","location");
    root.set("acc",int(movement));
    root.set("batt",vbat);
    root.set("cog",int(gps_data[4]));
    delay(2);
    root.set("lat",converter[0],7);
    delay(2);
    root.set("lon",converter[1],7);

    //root.set("lat",gps_data[0],7);
    //delay(2);
    //root.set("lon",gps_data[1],7);
    
    root.set("tid",TID);
    root.set("vel",(int(gps_data[2]) <= SPEED_TR ? 0 : int(gps_data[2])));
    delay(50);
    root.set("tst",uxdate);
    delay(2);
    memset(geodata, 0, sizeof(geodata));
    delay(50);
    root.printTo(geodata, 125);
    delay(50);

        
  //if ((((gps_data[0]) - lastmovement_W ) > float(0.0010000)) or (((gps_data[1]) - lastmovement_L ) > float(0.0010000)) or reporter == HEARTBIT) {
  // if (((converter[0] - lastmovement_W ) > float(0.0010000)) or ((converter[1] - lastmovement_L ) > float(0.0010000)) or reporter == HEARTBIT) {
 
 if ( movement > MOVEMENT or reporter == HEARTBIT){
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
  lastmovement_W = converter[0];
  lastmovement_L = converter[1];
  


/*
lastmovement_W = gps_data[0];
delay(50);
lastmovement_L = gps_data[1];
delay(50);
*/
  
  delete[] converter,movement,root;
  
  } else {
      
      readytosend = 0;
      
     // reporter every xxx min - position reported even not moved
    reporter++;
      //simulator for test only
      //float adding = random(0,2) * 0.0010000;
      //lastmovement_W = converter[0] + adding;
      //lastmovement_L = converter[1] + adding;

     
  }       
#ifdef DEBUG
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
  Serial.print(F("   Seconds waiting: "));
  delay(50);
  Serial.print((HEARTBIT * (DELAY +5)));
  Serial.print(F(" seconds before heartbit: "));
  delay(50);
  Serial.println( ((HEARTBIT * (DELAY +5)) - (reporter *(DELAY +5))));
  Serial.println();
#endif
  delete[] converter,movement,root;

}

// Calculate distance between two points
//distanceCoordinates(latitude, longitude, initialLatitude, initialLongitude);
float distanceCoordinates(float flat1, float flon1, float flat2, float flon2) {

  // Variables
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  // Calculations
  diflat  = radians(flat2-flat1);
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2 = cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;
  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
  
  dist_calc*=6371000.0; //Converting to meters
  
  return  dist_calc;
}
  
  

