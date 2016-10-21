#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"


#define halt(s) { Serial.println(F( s )); while(1);  }

extern Adafruit_FONA fona;
extern SoftwareSerial fonaSS;

char utime[14];
int YY;
byte DATEOTHER[5];
//int DD;
//int hh;
//int mm;
//int ss;

// flags
byte gps_enabled = 0;


boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password) {
  Watchdog.reset();

  //Serial.println(F("Initializing FONA....(May take 3 seconds)"));
  
  //fonaSS.begin(*fonaSerial); // if you're using software serial
  fonaSS.begin(9600);
  if (! fona.begin(fonaSS)) {           // can also try fona.begin(Serial1) 
    Serial.println(F("Couldn't find FONA"));
    return false;
  }
  fonaSS.println("AT+CMEE=2");
  //Serial.println(F("FONA is OK"));
  //Watchdog.reset();
 // Serial.println(F("Checking for network..."));
  while (fona.getNetworkStatus() != 1) {
   delay(500);
  }

  Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  Watchdog.reset();
  
  fona.setGPRSNetworkSettings(apn, username, password);

  //Serial.println(F("Disabling GPRS"));
  fona.enableGPRS(false);
  
  Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  Watchdog.reset();

  Serial.println(F("Enabling GPRS"));
  if (!fona.enableGPRS(true)) {
    //Serial.println(F("Failed to turn GPRS on"));  
    return false;
  }

  Watchdog.reset();
  
  return true;
}


boolean GPS() {
  if (gps_enabled == 0 ) {
    //Serial.println(F("Enabling GPS"));
    if (!fona.enableGPS(true)) {
      Serial.println(F("Failed to turn GPS on"));  
      return false;
    }
    else {
      gps_enabled = 1;
    }
  }

  Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  Watchdog.reset();
  
  Watchdog.reset();
  //Serial.println(F("Looking for satelites"));
  byte stat;
  stat = fona.GPSstatus();
  if (stat < 0 or stat == 0  or stat == 1) {
    Serial.println(F("Not fixed!"));
    return false;
  }
  if (stat >= 2) {
    Serial.println(F("GPS Fixed"));
  }

  Watchdog.reset();
  
  return true;
}


void GPS_Data(float *fdata, int *idata) {
  float latitude, longitude, speed_kph, heading, altitude;
  boolean gps_success = fona.getGPS(&latitude, &longitude, &altitude, &speed_kph, &heading, &utime[0]);
  sscanf(utime,"%04d%02d%02d%02d%02d%02d",&YY,DATEOTHER[0],DATEOTHER[1],DATEOTHER[2],DATEOTHER[3],DATEOTHER[4]);

  // put data into array
  fdata[0] = latitude;
  fdata[1] = longitude;
  fdata[2] = speed_kph;
  fdata[3] = altitude;
  fdata[4] = heading;
  idata[0] = DATEOTHER[2];
  idata[1] = DATEOTHER[3];
  idata[2] = DATEOTHER[4];
  idata[3] = DATEOTHER[1];
  idata[4] = DATEOTHER[0];
  idata[5] = YY;
  if (&gps_success) {
    //delete utime;
    return &fdata;
    
  }
}

