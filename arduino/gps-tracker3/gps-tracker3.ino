
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
//#include "ArduinoJson.h"
#include "ArduinoJson_32u4A7.h"
#include <SoftwareSerial.h>
#include "LowPower_32u4A7.h"
/*************************** FONA Params ***********************************/

//#undef FONA_DEFAULT_TIMEOUT_MS
//#define FONA_DEFAULT_TIMEOUT_MS 600
//#undef MQTT_CONN_KEEPALIVE
//#define MQTT_CONN_KEEPALIVE 1800
//#undef MQTT_FONA_INTERAVAILDELAY
//#define MQTT_FONA_INTERAVAILDELAY 200
//#undef MQTT_FONA_QUERYDELAY
//#define MQTT_FONA_QUERYDELAY 600

/***************************LED*******************************************/

//byte ledState = 0;             // ledState used to set the LED
//long previousMillis = 0;        // will store last time LED was updated
 

/*************************** FONA Pins ***********************************/

#define GSM_RX  2
#define GSM_TX  3
#define GSM_RST 4 // pin number 16 in sim808 throug diode to ground https://cdn-shop.adafruit.com/datasheets/SIM808_Hardware+Design_V1.00.pdf max 4.3 V
#define GSM_POWER 5
#define GSM_CHARGE 6
SoftwareSerial fonaSS = SoftwareSerial(GSM_TX, GSM_RX);
//oftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(GSM_RST);


byte readytosend = 0;
byte reporter = 0;
static char geodata[126];
volatile int vbat=4200;
static float lastmovement_W = 0;
static float lastmovement_L = 0;

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


#define _HWB_H_13()  (PORTB |=  (1<<PB5))
#define _HWB_L_13()  (PORTB &= ~(1<<PB5))
#define _HWB_H_5()  (PORTD |=  (1<<PD5))
#define _HWB_L_5()  (PORTD &= ~(1<<PD5))

void setup() {
  
  DDRB |= (1<<PB5); // pinMode(13,OUTPUT); //  DDRB |= (1<<PB5); //pinMode(LED_BUILTIN, OUTPUT);  //PB5
  DDRD |= (1<<PD5); //pinMode(GSM_POWER, OUTPUT);  /PD5
  DDRD &= ~(1<<PD6);//pinMode(GSM_CHARGE, INPUT);

  Serial.begin(57600);
  delay(100);
  _HWB_L_5();//PORTD &= ~(1<<PD5);//digitalWrite(GSM_POWER,LOW);
  delay(3000);
  //digitalWrite(GSM_POWER,HIGH);
  //delay(100);
  //digitalWrite(GSM_POWER,LOW);
//charge check


  while (!Serial);
  fonaSS.begin(4800);
  delay(50);
  while (!fonaSS);
  //delay(20);
  //fonaSS.println("ATZ");
  fona.begin(fonaSS);
  delay(100);

  charge();
  
  //fona.enableRTC(1);
  //while (!fonaSS);
  //fonaSS.println("AT&F1");
  //delay(500);
  //fonaSS.println("AT+ECHARGE=1");
  //delay(1000);
  //fonaSS.println("AT&W");
  //delay(500);
  //fonaSS.println("AT&W");
  //delay(500);
  //clearbuffer();
//start 
  for (byte i=0; i<10;i++ ) {
  _HWB_H_13();//digitalWrite(LED_BUILTIN, 1);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  _HWB_L_13();//digitalWrite(LED_BUILTIN, 0);    // turn the LED off by making the voltage LOW
  delay(80); 
  }
//#ifdef DEBUG
  Serial.println(F("GPS Tracker starting ..."));
  delay(50);
//#endif
#ifndef DEBUG
  Serial.println(F("Debug is off"));
  delay(50);
#endif


  sleep(2);  // wait a few seconds to stabilize connection
  delay(1000);
  while ( !fonaSS);
  // Initialise the FONA module
  while (! FONAconnect(F(GSM_APN), F(GSM_USERNAME), F(GSM_PASSWORD))) {
#ifdef DEBUG   
    Serial.println(F("Retrying FONA GPRS ..."));
    delay(50);
#endif

    sleep(2);
    delay(1000);
    while ( !fonaSS);
}

    sleep(2); 
    delay(1000);
    while ( !fonaSS);
    byte trying =0;
    while (! GPS() ) {
#ifdef DEBUG
    Serial.println(F("Retrying FONA GPS ..."));
    delay(50);
#endif

    sleep(2);
    delay(1000);
    while ( !fonaSS);
    trying++;
    if (trying == 20)resetFunc();
   }
#ifdef DEBUG
  Serial.println(F("Connected to Cellular!"));
  delay(50);
#endif

  sleep(2);  // wait a few seconds to stabilize connection
  delay(1000);
  
}


void loop() {
    while ( !fonaSS);
    fonaSS.flush();
    delay(50); 
    Serial.flush();
    delay(50);  

   if (! fona.getBattVoltage(&vbat)) {
   
    vbat = 0;
   //} else {
   // blink();
  } 
//charge needed    
if ( vbat <= 3400) {
    _HWB_H_13();//digitalWrite(LED_BUILTIN, 1);
#ifdef DEBUG
    Serial.println(F("LOW VOLTAGE ! charge it !!!"));
    delay(50);
#endif 
    _HWB_H_5();
    sleep(500); //66 min
    _HWB_L_5();
    delay(3000);
  } else { 

   _HWB_L_13();//digitalWrite(LED_BUILTIN, 0);
   
  }
//charge check
charge();



//-------------main function not generate when no GPS, GPRS, GSM

do{
   //clearbuffer();
   //delay(DELAY*1000);  // wait a few seconds to stabilize connection


   sleep(DELAY);
   
   delay(1000);
   while (!fonaSS);
   fona.deleteSMS(1);
   delay(1000);
#ifdef DEBUG
   Serial.println(F("Network avaliable GPS fixed GPRS connected, processing ..."));
   delay(50);
#endif
   //clearbuffer();
   prepareData();
   //delay(1000);    
  
if (readytosend == 1) {
      
    //clearbuffer();
    MQTT_connect();
    //delay(100);

    
    // Now we can publish stuff!
    //clearbuffer();
    byte check =3;
  do {
    while (!fonaSS);
    //clearbuffer();
    if (! feed.publish(geodata)) {
        
#ifdef DEBUG
       Serial.println(F("Sent Failed"));
       delay(50);
#endif
        delay(50);
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
      
      delay(100); 
          while (!fonaSS);
          if (!fona.TCPconnected()){
             delay(50); 
              return;
           }   
}
  
  //clearbuffer();
  

  
  

    while ( !fonaSS);
    } while(fona.GPRSstate() == 1  && fona.GPSstatus() > 1  );
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

           sleep(1);
           delay(2000);
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
    int timeoutloop = 0;
    
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
      //delay(100);
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
#ifdef DEBUG
      Serial.println(mqtt.connectErrorString(ret));
//      delay(50);
#endif
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



  
  int ctime[6];
  float gps_data[5];
  
  while (!fonaSS);
   if (! fona.getBattVoltage(&vbat)) {
   
    vbat = 0;
   //} else {
   // blink();
  } 
  //fona.getBattVoltage(&vbat);
  
  
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
  yield();
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
  //movement = distance_in_m(converter[0],converter[1],lastmovement_W,lastmovement_L);
  if ((int)movement > 1000 or (int)movement < 0  ) movement = MOVEMENT + 1.0; //1000 means more than 200 km/h, it is added before first initialosation of last loc.
  
  StaticJsonBuffer<126> jsonBuffer;
  
  //memset(jsonBuffer, 0, sizeof(jsonBuffer));
  //delay(50);
  JsonObject& root = jsonBuffer.createObject();
  //yield();
  delay(100);
  
    
    root.set("_type","location");
    root.set("acc",int(movement));
    
    root.set("batt",vbat);
    root.set("cog",int(gps_data[4]));
    
    root.set("lat",converter[0],7);
    
    root.set("lon",converter[1],7);
    
    //root.set("lat",gps_data[0],7);
    //delay(2);
    //root.set("lon",gps_data[1],7);
    
    root.set("tid",TID);
    root.set("vel",(int(gps_data[2]) <= SPEED_TR ? 0 : int(gps_data[2])));
    
    root.set("tst",uxdate);
    
    memset(geodata, 0, sizeof(geodata));
    
    delay(50);
    root.printTo(geodata, 126);
    
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
  
  delete[] converter,movement,&root;
  
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
  Serial.print((HEARTBIT * ((DELAY *8) +5)));
  Serial.print(F(" seconds before heartbit: "));
  delay(50);
  Serial.println( ((HEARTBIT * ((DELAY *8) +5)) - (reporter *((DELAY *8) +5))));
  Serial.println();
#endif
  delete[] converter,movement,&root;

}

// Calculate distance between two points
//http://forum.arduino.cc/index.php?topic=27541.0
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
  yield();
  return  dist_calc;
  
}

//BLINK

void sleep(uint16_t count){

        while  (count-- >0){        
                
          LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
          yield();
          delay(100);
        }
          
}

void charge(){
  while ((digitalRead(GSM_CHARGE))) {
    
    _HWB_H_13();//digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
    //digitalWrite(LED_BUILTIN, LOW);
    //delay(2000);
    #ifdef DEBUG
    Serial.println(F("Charging"));
    delay(50);
    #endif
    fona.enableGPRS(false);
    delay(500);
    fona.enableGPS(false);
    delay(500);
    _HWB_H_5();
    sleep(1000); //133 min
    _HWB_L_5();
    delay(3000);
  } 
    fona.enableGPRS(true);
    delay(500);
    fona.enableGPS(true);
    delay(500);
    _HWB_L_13();//digitalWrite(LED_BUILTIN, 0); 
}



