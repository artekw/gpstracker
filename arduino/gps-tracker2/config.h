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
#define GSM_APN "internet"
#define GSM_USERNAME ""
#define GSM_PASSWORD ""



//#define AIO_SERVER      "m20.cloudmqtt.com"
//#define AIO_SERVER      "broker.mqttdashboard.com"
//#define AIO_SERVER "broker.hivemq.com"
#define AIO_SERVER "155.133.46.194"
#define AIO_SERVERPORT 1883
#define AIO_SERVER "155.133.46.194"
#define AIO_SERVERPORT 1883
//#define AIO_SERVERPORT  11971
//#define AIO_SERVERPORT  10429
#define AIO_USERNAME    "xxxxxxxxxxx"
#define AIO_PASSWORD    "xxxxxxxxxxx"
#define TOPIC "owntracks/piesek/yasmina"
#define TID "yb" // "tracker-ID which is used by the auto-faces feature to display, say, initials of a user."
#define SPEED_TR 3  // bellow that number = 0
#define SPEED_TR 3  // bellow that number = 0
#define HEARTBIT 90 // there ic multiply by DELAY and sending heartbit - set for every 30 min when movement trying is 5 sec.
#define DELAY 13 // seconds + ~6 
