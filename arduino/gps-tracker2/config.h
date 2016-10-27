/*define mqtt brocker 
 * https://customer.cloudmqtt.com/login
 * register your server (for free 10 connections allowed)
 * set direct access to server - config as standard owntrack config
 * create user i.e. piesek
 * create topic for user RW access owntracks/#
 * 
 * 
 */

#define GSM_APN         "internet"
#define GSM_USERNAME    ""
#define GSM_PASSWORD    ""

#define AIO_SERVER      "xxxxxx.cloudmqtt.com"
#define AIO_SERVERPORT  11971
#define AIO_USERNAME    "xxxxxxx"
#define AIO_PASSWORD    "xxxxxxx"

#define TOPIC           "owntracks/piesek/yasmina"
#define TID             "yb" // "tracker-ID which is used by the auto-faces feature to display, say, initials of a user."
#define SPEED_TR        2  // bellow that number = 0

#define DELAY           5  // seconds
