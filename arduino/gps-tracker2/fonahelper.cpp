#include "config.h"
#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"


#define halt(s) { Serial.println(F( s )); while(1);  }


extern Adafruit_FONA fona;
extern SoftwareSerial fonaSS;
extern SoftwareSerial *fonaSerial;



// flags
byte gps_enabled = 0;


//send AT command and wait for responce and print

uint8_t sendATcommand(char* ATcommand,unsigned int *timeout){

    uint8_t x=0,  answer=0;
    char response[30];
    unsigned long previous;

    while( fonaSerial->available() > 0) fonaSerial->read();    // Clean the input buffer
    fonaSS.flush();
    if (ATcommand[0] != '\0')
    {
        fonaSerial->println(ATcommand);    // Send the AT command 
        delay(20);
    }


    x = 0;
    previous = millis();
    Serial.println();
    Serial.print(F("Execute AT command: "));
    Serial.println(ATcommand);

    Serial.print(F("Result: "));
    // this loop waits for the answer
    do{
        if(fonaSerial->available() > 0){    // if there are data in the UART input buffer, reads it and checks for the asnwer
            response[x] = fonaSerial->read();
            
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



boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password) {
  //Watchdog.reset();
  fonaSerial->begin(9600);
  if (! fona.begin(*fonaSerial)) {           // can also try fona.begin(Serial1) 
    Serial.println(F("Couldn't find FONA"));
    return false;
  } else {
  //fona.deleteSMS(1);
  
  //Serial.print(F("Check FONA err: - expect OK"));
 //set additionalcommands to modem  
 //   sendATcommand("AT+CMEE=2", 200);
//  Serial.print(F("Delete all SMS - expect OK"));
//  sendATcommand("AT+CMGD=1,4",200);
   //sendATcommand("AT&W",200);
   //Serial.print(F("Registration IP - expect OK"));
   //sendATcommand("AT+SAPBR=2,1",200);
   //reset GPS
   //led blink 0 1 2
   //AT+CSGS=2
   //delete smses from sim
   //"AT+CMGD=1" 
   //check errors 
   //"AT+CMEE=2"
   //disable battery charge - power consumption
   //"AT+CGPSRST=0"
   //"AT&W"
   // reset to default
   //"ATZ0";
   //"AT&W";
   //delay(1000);
  }
  
  while (! fona.getNetworkStatus()) {
   Serial.println(F("Waiting for network..."));
    delay(1000);
    /*   
        uint8_t n = fona.getRSSI();
        int8_t r;

        Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(": ");
        if (n == 0) r = -115;
        if (n == 1) r = -111;
        if (n == 31) r = -52;
        if ((n >= 2) && (n <= 30)) {
          r = map(n, 2, 30, -110, -54);
        }
   Serial.print(r); Serial.println(F(" dBm"));
   
   
   */
           
   //Watchdog.reset();
  }

  //Watchdog.reset();
  delay(7000);  // wait a few seconds to stabilize connection
  //Watchdog.reset();
  fona.setGPRSNetworkSettings(apn, username, password);
  delay(20);
  //Serial.println(F("Disabling GPRS"));
  fona.enableGPRS(false);
  //Watchdog.reset();
  delay(10000);  // wait a few seconds to stabilize connection
  //Watchdog.reset();

  Serial.println(F("Enabling GPRS"));
  while (!fona.enableGPRS(true)) {
    delay(10000);
    Serial.println(F("Failed to turn GPRS on"));  
    
    return false;
  }

  //Watchdog.reset();
  
  return true;
}


boolean GPS() {
  if (gps_enabled == 0 ) {
    Serial.println(F("Enabling GPS"));
    if (!fona.enableGPS(true)) {
      Serial.println(F("Failed to turn GPS on"));  

      return false;
    }
    else {
      gps_enabled = 1;
    }
  }

  //Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  //Watchdog.reset();
  byte stat;
  stat = fona.GPSstatus();
  
  if (stat < 0 or stat == 0  or stat == 1) {
    
    Serial.println(F("Not fixed!"));

    return false;
    //delay(5000);
  }
  if (stat >= 2) {
    Serial.println(F("GPS Fixed"));
  }
  //Watchdog.reset();
  return true;
}

void GPS_Data(float *fdata, int *idata) {
  char utime[14];
  int YY;
  int MM;
  int DD;
  int hh;
  int mm;
  int ss;
  //float latitude, longitude, speed_kph, heading, altitude;
  boolean gps_success = fona.getGPS(&fdata[0], &fdata[1], &fdata[3], &fdata[2], &fdata[4], &utime[0]);
  delay(20);
  sscanf(utime,"%04d%02d%02d%02d%02d%02d",&YY,&MM,&DD,&hh,&mm,&ss);
  delay(20);
  idata[0] = hh;
  idata[1] = mm;
  idata[2] = ss;
  idata[3] = DD;
  idata[4] = MM;
  idata[5] = YY;

  delay(20);
    
  if (&gps_success) {
    
    return &fdata;
    
  }
}



