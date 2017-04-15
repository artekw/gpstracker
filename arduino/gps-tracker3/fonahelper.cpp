#include "config.h"
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>


/*
//prog mem helper - change buffer size for topic if needed 
const char stringBuffer[30];
const char* getString(const char* str[30]) {
strcpy_P(stringBuffer, (const char*)str);
delay(50);
return stringBuffer;
}
*/


extern Adafruit_FONA fona;
extern SoftwareSerial fonaSS;

//extern SoftwareSerial *fonaSerial;


// flags
byte gps_enabled = 0;


boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password) {
 

    while (!fonaSS);
    fonaSS.begin(9800);
    delay(50);

 
  if (! fona.begin(fonaSS)) {           // can also try fona.begin(Serial1) 
    delay(50);
#ifdef DEBUG
    Serial.println(F("Couldn't find FONA"));
    delay(50);
#endif
    return false;
  } else {

  // to test   
  }
  
  while (!fonaSS);
  

  byte net =0;
  while (fona.getNetworkStatus() != 1) {
   delay(500);
#ifdef DEBUG
   Serial.println(F("Waiting for network ..."));
#endif
   delay(15000);
   while ( !fonaSS);
   net++;
   if (net == 20) {
   fonaSS.end();
   asm volatile ( "jmp 0");
   }
  }


  //delay(20000);  // wait a few seconds to stabilize connection
  
  fona.setGPRSNetworkSettings(apn, username, password);
  //Serial.println(F("Disabling GPRS"));
  while (!fonaSS);
  fona.enableGPRS(false);
  delay(500);  // wait a few seconds to stabilize connection
#ifdef DEBUG
  Serial.println(F("Enabling GPRS"));
  delay(50);
#endif
  while (!fonaSS);
  if (!fona.enableGPRS(true)) {
#ifdef DEBUG
    Serial.println(F("Failed to turn GPRS on")); 
    delay(50); 
#endif
    return false;
  }
  while (!fonaSS);
  fona.enableRTC(1);
  
  return true;
}

boolean GPS() {
  
  if (gps_enabled == 0 ) {
#ifdef DEBUG
    Serial.println(F("Enabling GPS"));
    delay(50);
#endif
    while ( !fonaSS);
    fona.enableGPS(false);
    delay(500);
    while ( !fonaSS);
    if(!fona.enableGPS(true)) {
#ifdef DEBUG
      Serial.println(F("Failed to turn GPS on"));
      delay(50);
#endif  
      return false;
    } else {
      gps_enabled = 1;
      delay(10000);
    }
  }
  byte stat;
  while ( !fonaSS);
  stat = fona.GPSstatus();
  delay(500);
  if (stat < 0 or stat == 0  or stat == 1) {
#ifdef DEBUG    
    Serial.println(F("Not fixed!"));
    delay(50);
#endif
    return false;

   }
   if (stat >= 2) {
#ifdef DEBUG
    Serial.println(F("GPS Fixed"));
    delay(50);
#endif
    return true;
   } else {
    return false;
   }

return false;
}


void GPS_Data(float *fdata, int *idata) {
  
  char utime[14];
  //int YY;
  //int MM;
  //int DD;
  //int hh;
  //int mm;
  //int ss;
  float latitude, longitude, speed_kph,cog;
  //, speed_kph, heading, altitude;
  //boolean gps_success = fona.getGPS(&fdata[0], &fdata[1], &fdata[3], &fdata[2], &fdata[4], &utime[0]);
  while (!fonaSS);
  boolean gps_success = fona.getGPS(&latitude,&longitude, &fdata[3], &speed_kph, &cog, &utime[0]);
  delay(50);
  //sscanf(utime,"%04d%02d%02d%02d%02d%02d",&YY,&MM,&DD,&hh,&mm,&ss);
  sscanf(utime,"%04d%02d%02d%02d%02d%02d",&idata[5],&idata[4],&idata[3],&idata[0],&idata[1],&idata[2]);
  delay(50);
  //idata[0] = hh;
  //idata[1] = mm;
  //idata[2] = ss;
  //idata[3] = DD;
  //idata[4] = MM;
  //idata[5] = YY;

 
    
  if (&gps_success) {

      fdata[0]=latitude;
      fdata[1]=longitude;
      fdata[2]=speed_kph;
      fdata[4]=cog;
       //sendATcommand("AT+CGATT=1",1000);
    delay(50);

    return &fdata;
    
  }
}


