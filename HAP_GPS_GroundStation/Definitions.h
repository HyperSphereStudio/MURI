/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////********************* Main Variable Definitions **********************///////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
// *********** RADIO **************//
#define RADIO_BAUD 230400
#define RADIO_SIZE 19 // Change this to change the packet size of a single GPSsonde trnasmission - Change this with the GPSONDE code
#define RF95_FREQ 915.0
byte buf[3];
uint8_t len = RADIO_SIZE;

uint8_t outPack[RADIO_SIZE + 8];

// packet info

uint8_t numSats = 0;  //Number of Satellites
uint8_t utcHour = 0;  //UTC Time - Hour
uint8_t utcMin = 0;   //UTC Time - Minutes
uint8_t utcSec = 0;   //UTC Time - Seconds
int i = 0;
//GPS INFORMATION
#define GPS_BAUD 9600
