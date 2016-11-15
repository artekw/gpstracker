#include "config.h"
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>
#define FONA_DEFAULT_TIMEOUT_MS 750


extern Adafruit_FONA fona;
extern SoftwareSerial fonaSS;

//extern SoftwareSerial *fonaSerial;


// flags
byte gps_enabled = 0;


//send AT command and wait for responce and print
/*

uint8_t sendATcommand(char* ATcommand,unsigned int *timeout){
    
    uint8_t x=0,  answer=0;
    char response[20];
    unsigned long previous;

    while( fonaSS.available() > 0) fonaSS.read();    // Clean the input buffer
    fonaSS.flush();
    if (ATcommand[0] != '\0')
    {
        fonaSS.println(ATcommand);    // Send the AT command 
        delay(20);
    }


    x = 0;
    previous = millis();
    Serial.println();
    Serial.print(F("Execute AT command: "));
    delay(50);
    Serial.println(ATcommand);

    Serial.print(F("Result: "));
    delay(50);
    // this loop waits for the answer
    do{
        if(fonaSS.available() > 0){    // if there are data in the UART input buffer, reads it and checks for the asnwer
            response[x] = fonaSS.read();
            
            Serial.print(response[x]);
            //delay(20);
            x++;
            answer = 1;
          } else {
            answer = 0;
         }
    }while(((millis() - previous) < &timeout));    // Waits for the asnwer with time out
    //delete buffer - freeup memory
    delete[] response,previous,timeout;
    return answer;

}

*/

boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password) {
 

    while (!fonaSS);
    fonaSS.begin(4800);
    delay(20);

 
  if (! fona.begin(fonaSS)) {           // can also try fona.begin(Serial1) 
    Serial.println(F("Couldn't find FONA"));
    delay(50);
    return false;
  } else {

  
    
  }
  
  while (!fonaSS);
  

  byte net =0;
  while (fona.getNetworkStatus() != 1) {
   Serial.println(F("Waiting for network ..."));
   delay(20000);
   while ( !fonaSS);
   net++;
   if (net == 20) {
   fonaSS.end();
   asm volatile ( "jmp 0");
   }
  }


  delay(20000);  // wait a few seconds to stabilize connection
  while (!fonaSS);
  fona.setGPRSNetworkSettings(apn, username, password);
  //Serial.println(F("Disabling GPRS"));
  fona.enableGPRS(false);
  delay(2000);  // wait a few seconds to stabilize connection
  Serial.println(F("Enabling GPRS"));
  while (!fonaSS);
  if (!fona.enableGPRS(true)) {
    Serial.println(F("Failed to turn GPRS on")); 
    delay(50); 
    return false;
  }

  fona.enableRTC(1);
  
  return true;
}

boolean GPS() {
  
  if (gps_enabled == 0 ) {
    Serial.println(F("Enabling GPS"));
    delay(50);
    while ( !fonaSS);
    fona.enableGPS(false);
    delay(2000);
    while ( !fonaSS);
    if(!fona.enableGPS(true)) {
      Serial.println(F("Failed to turn GPS on"));
      delay(50);  
      return false;
    } else {
      gps_enabled = 1;
      delay(20000);
    }
  }
  byte stat;
  while ( !fonaSS);
  stat = fona.GPSstatus();
  delay(1000);
  if (stat < 0 or stat == 0  or stat == 1) {
    
    Serial.println(F("Not fixed!"));
    delay(50);
    return false;
    //delay(5000);
   }
   if (stat >= 2) {
    Serial.println(F("GPS Fixed"));
    delay(50);
    return true;
   } else {
    return false;
   }

return false;
}


void GPS_Data(float *fdata, int *idata) {
  
    while ( !fonaSS);
    while (fona.GPSstatus() < 2) {
      fona.enableGPS(false);
      delay(1000);
      while ( !fonaSS);
      fona.enableGPS(true);
      delay(10000);
      byte checking=0;
      do{
        Serial.println(F("Not fixed!"));
        delay(2000);
        checking++;
        while ( !fonaSS);
      } while (fona.GPSstatus() < 2 or checking == 10);
      
   }
  
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

