/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////********************* Main Variable Definitions **********************///////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

// packet header
uint16_t id_erau = 0b101010101 << 7;             // 9 bit Header for packet
uint16_t payloadID = 3;                     //ID number of the payload. unique idsentifier for each payload being flown
uint16_t packNum;                            // Packet number counter 
uint16_t header = id_erau | (payloadID << 4);

uint8_t NUM_PAYLOADS = 6;   // Number of payloads to be launched

unsigned long timer = 0;
// *********** GPS **************//
bool ack = false;
// GPS Communicatoin
#define GPS_BAUD 9600

// GPS Data
float alt;
float latitude;
float longitude;
float velN;
float velE;
float velD;

uint8_t stat = 0;        //Status
uint16_t numSats = 0;  //Number of Satellites in View
uint8_t utcHour = 0;  //UTC Time - Hour
uint8_t utcMin = 0;   //UTC Time - Minutes
uint8_t utcSec = 0;   //UTC Time - Seconds
uint8_t prevSec = 0;
uint8_t count1 = 0;

// Timing
#define UPDATE_INTERVAL_GPS 2500
unsigned long gpsTimer;
uint8_t newData = 0;    //new data set to high when gps data available. must be high to transmit data

// *********** Battery Voltage **************/
#define V_BAT A7
uint16_t vbat;

// *********** RADIO **************/
#define RADIO_BAUD 230400
#define RADIO_SIZE 19  //Change this in accordance with the Ground Station Radio Size - Packet Size of one transmission
#define RF95_FREQ 915.0

// *********** TEMPERATURE **************/
#define TEMP_PIN A1
uint16_t tempCounts = 0;
//ADC *adc = new ADC();
// *********** PRESSURE *************/
float pressure;
uint16_t presdata;
#define PRESSURE_CONVERSION 0.537392

byte dataPack[RADIO_SIZE];

// *********** PRESSURE *************/
#define led 13   //pin that is connected to Feathers' LED indicator light
