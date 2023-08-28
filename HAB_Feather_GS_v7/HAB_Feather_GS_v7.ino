/**********************************************************************
   NAME: V1_GF_Feather_Com_Server.ino

   PURPOSE: Feather client to server testing - Basic GPS Payload Integrated.

   DEVELOPMENT HISTORY:
     Date    Author  Version            Description Of Change
   --------  ------  -------  ---------------------------------------
   12/06/21   GMF      1.0     Code for basic radio communication of GPS data
   06/06/22   JAG      6.1     Add GPS to ground station. get UTC time and add to packet
   08/02/22   PAR      7.0     Added partial packet receiving and parsing.
 *********************************************************************/
/******SERIAL DEFINITIONS*******
  FRFM95              Serial
  GPS UBlox           Serial1
********************************/

#include <SPI.h>
#include <RH_RF95.h>
#include <UbloxGPS.h>
#include "Definitions.h"

// Seeeduino Xiao with RFM95
RH_RF95 rf95(8, 3);
UbloxGPS gps(&Serial1);


void setup() {
  Serial.begin(RADIO_BAUD);
  Serial1.begin(GPS_BAUD);
  if (!rf95.init()) Serial.println("init failed");

  //  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

  rf95.setTxPower(20, false);
  // You can change the modulation parameters with eg
  rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);   // Min Bw - Max Sfr for max range
  //GPS setup
  initGPS();
}

void loop() {
  // Read Packets from payloads
  updateGPS();
  if (rf95.available())
  {
    len = RADIO_SIZE;
    // Should be a message for us now
    if (rf95.recv(outPack, &len))
    {
      //get current time when packet is received
      //add UTC time to packet
      outPack[RADIO_SIZE] = utcSec;
      outPack[RADIO_SIZE+1] = utcMin;
      outPack[RADIO_SIZE+2] = utcHour;
      outPack[RADIO_SIZE+3] = numSats;
      // populate rssi data, then send to radio
      uint16_t rssi = rf95.lastRssi();
      //      Serial.println(rssi,HEX);
      outPack[RADIO_SIZE+4] = rssi;
      outPack[RADIO_SIZE+5] = rssi >> 8;
      for (i = 0; i < sizeof(outPack); i++) {
        Serial.write(outPack[i]);
      }
    }
  }

  // Read commands from GS
  if (Serial.available() > 2) {
    buf[0] = Serial.read();
    buf[1] = Serial.read();
    buf[2] = Serial.read();

    // send to radio
    rf95.send(buf, sizeof(buf));
  }
}
