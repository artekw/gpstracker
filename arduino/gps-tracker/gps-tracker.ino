/*
GPS Tracker v0.1

TODO:
- wysyłanie danych na sparkfun po wysłaniu SMSa
- obsługa błędów
- optymalizacja
- obudowa 
- LCD OLED
- wysyłanie danych jak jest ruch (akcelerometr?)
*/

// GSM
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>
#include "config.h"

//OLED
#define I2C_ADDRESS 0x3C
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

// GSM Conf
#define FONA_RX 2 //connect to FONA RX
#define FONA_TX 3 //connect to FONA TX
#define FONA_RST 4

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX); //initialize software serial
SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
SSD1306AsciiAvrI2c oled;

char Lat[15];
char Lon[15];
char Speed[15];
char HDOP[3];

char Time[23];

byte gprs_connected = 0;


void setup() {
    Serial.begin(115200);
    Serial.println("Start Tracker");
    fonaSerial->begin(9600);
    if (! fona.begin(*fonaSerial)) {
      Serial.println(F("Couldn't find FONA"));
      while (1);
    }

    Serial.println("GPS Enable");
    enableGPS();
    Serial.println("Setup done !");

    // APN
    fona.setGPRSNetworkSettings(F(APN));
    // SSL
    //fona.setHTTPSRedirect(true);

    // OLED
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
    oled.setFont(Adafruit5x7);
    Serial.println(F("Start..."));
    oled.clear();
    oled.print("Start...");
}


void loop() {
    if (getLoc()) {
      Serial.println("GPS Fixed!");
      //displayGUI();
      //delay(10000);
      oled.clear();
      oled.println("GPS Fixed!");
      oled.println();
      oled.print("Lat: ");
      oled.println(Lat);
      oled.print("Lon: ");
      oled.println(Lon);
      oled.print("HDOP: ");
      oled.println(hdopbar(HDOP));
      oled.print("GSM: ");
      if (gsm_status()) {
        oled.println("ok");
      }
      else {
        oled.println("!!");
      }
      getLoc();
        if (gprs_connected == 0) {
          if (!fona.enableGPRS(true)) {
            fona.enableGPRS(false);
            Serial.println(F("Failed to turn on"));
            Serial.println(F("Wait 10 seconds..."));
            delay(10000);
            }
            else {
              gprs_connected = 1;
              Serial.println(F("GPRS Connected!"));
              oled.clear();
              oled.print("GPRS Connected!");
            }
        }
        else {
        uint16_t statuscode;
        int16_t length;
        char loc[50];

        sprintf(loc, "{\"longitude\":%s,\"latitude\":%s}", Lon, Lat);

        if (!fona.HTTP_POST_start(POST_URL, F("application/json"), (uint8_t *) loc, strlen(loc), &statuscode, (uint16_t *)&length)) {
          
          if (!statuscode == 201) {
            gprs_connected = 0;
          }
          if (statuscode == 201) {
            Serial.println(F("Data send !"));
            oled.clear();
            oled.print("Wyslano na serwer");
            delay(3000);
            fona.HTTP_POST_end();
          }
        }
      }
      Serial.println(F("Czekam..."));
      oled.clear();
      oled.print("Czekam ");
      oled.print(DELAY);
      oled.println(" sek...");
      oled.println();
      oled.print("HDOP: ");
      oled.println(hdopbar(HDOP));
      oled.print("Lat: ");
      oled.println(Lat);
      oled.print("Lon: ");
      oled.println(Lon);
      delay(1000*DELAY);
      
    }
    else 
    {
      Serial.println(F("Looking for GPS satellites"));
      oled.clear();
      oled.println("Szukanie satelitow...");
      oled.println();
      oled.print("HDOP: ");
      oled.println(hdopbar(HDOP));
      delay(5000);
    }
}


void enableGPS() {
  Serial.print(F("Enable GPS:"));
  if (!fona.enableGPS(true))
    Serial.println(F("Failed to turn on"));
}


boolean gps_status() {
    int8_t stat;
    stat = fona.GPSstatus();
    if (stat == 0)
      return false;
    if (stat >=2)
      return true;
}


boolean gsm_status() {
  uint8_t n = fona.getNetworkStatus();
  if (n == 0) {
    return true;
  }
  else {
    return false;
  }
}


boolean getLoc() {
    float latitude, longitude, speed_kph, hdop;
    boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &hdop);
    if (gps_success) {
      dtostrf(longitude, 6, 6, Lon);
      dtostrf(latitude, 6, 6, Lat);
      dtostrf(speed_kph, 6, 6, Speed);
      dtostrf(hdop, 1, 1, HDOP);
    }
    else
    {
      dtostrf(hdop, 1, 1, HDOP);
      return false;
    }
}


char *hdopbar(char *_hdop) {
  float hdop = 999;
  hdop = atof(_hdop);
  if (hdop < 1) {
    return "***";
  }
  else if (hdop >= 7 && hdop <= 8) {
    return ".**";
  }
  else if (hdop >= 9 && hdop <= 20) {
    return "..*";
  }
  else if (hdop > 20) {
    return "...";
  }
}


void displayGUI() {
  oled.clear();
  oled.print("GPS: ");
  oled.print("***");
  oled.setCol(80);
  oled.print("GSM: ");
  oled.println("***");
  
  
}

