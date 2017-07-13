 
/*define mqtt brocker 
 * https://customer.cloudmqtt.com/login
 * register your server (for free 10 connections allowed)
 * set direct access to server - config as standard owntrack config
 * create user i.e. piesek
 * create topic for user RW access owntracks/#
 * 
 * 
 */
//soft serial buffer
#define _SS_MAX_RX_BUFF 128
//GSM 
#define GSM_APN "internet"
#define GSM_USERNAME ""
#define GSM_PASSWORD ""



//#define AIO_SERVER      "m20.cloudmqtt.com"
//#define AIO_SERVER      "broker.mqttdashboard.com"
//#define AIO_SERVER "broker.hivemq.com"
#define AIO_SERVER "xxx.xxx.xxx.xxx"
#define AIO_SERVERPORT 1883
#define AIO_USERNAME    "xxxxxx"
#define AIO_PASSWORD    "xxxxxxxxx"
#define TOPIC "owntracks/piesek/yasmina"
#define TID "yb" // "tracker-ID which is used by the auto-faces feature to display, say, initials of a user."
#define SPEED_TR 3  // bellow that number = 0
#define HEARTBIT 120 // there ic multiply by DELAY and sending heartbit - set for every 30 min when movement trying is 5 sec.
#define DELAY 1 // 8 WDT seconds +- ~5 
#define MOVEMENT 45.0  //metters moved to report

//#define DEBUG
