/**********************************************************************
   NAME: HAB_GPS_Standalone.ino

   PURPOSE: HIGH ALTITUDE BALLOON - Basic GPS Payload.

   DEVELOPMENT HISTORY:
     Date    Author  Version            Description Of Change
   --------  ------  -------  ---------------------------------------
   01/02/22   JCG      3.0     Code for basic radio communication of GPS data
   01/28/22   JCG      3.1     Fixed configuration of GPS module
   02/28/22   JCG      4.0     Shrunk packet down to bare minimum
   06/04/22   JAG      6.0     Removed packet number and UTC time and added temperature to packet
   06/06/22   JAG      6.1     Nothing updated in sent packet. version updated to match other codes
   07/30/22   PAR      6.2     Added Pressure Sensor data into the data packet
   08/03/22   PAR      7.0     Version update to keep consistant with the GS and MATLAB codes
   12/01/22   PAR      7.1     Higher resolution ADC
   05/17/23   JAG      7.2     Comment code. Remove redundant/obsolete sections
 *********************************************************************/
/******SERIAL DEFINITIONS*******
  Startup Monitor     Serial
  GPS UBlox           Serial1
********************************/

#include <SPI.h>
#include <RH_RF95.h>
#include <UbloxGPS.h>
#include <MS5611.h> // Barometer Library
//#include <ADC.h>              // analog to digital conversion
#include "Definitions.h"
#include "ATSAMD21_ADC.h"   //ADC modification Library

RH_RF95 rf95(8, 3); // Adafruit Feather M0 with RFM95-
UbloxGPS gps(&Serial1); //Assigns GPS to serial port
MS5611 baro; //Barometer Object

void setup() {
  /*********SERIAL PORTS*********/
  Serial.begin(9600);
  Serial1.begin(GPS_BAUD);
  //  SPI.begin();
  delay(2000);

  /*********LORA RADIO SETUP*********/
  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }

  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  rf95.setTxPower(20);  //set power of LoRa to adjust dBm 

  rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);   // Min Bw - Max Sfr for max range
  //adc->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);

  pinMode(led, OUTPUT);
  /*********GPS SETUP*********/
  initGPS();
  /*********Barometer Setup********/
  if (!baro.begin(MS5611_ULTRA_HIGH_RES))
    {
      Serial.println(F("Could not find a valid MS5611 sensor, check wiring!"));
    } else Serial.println("BAROMETER OK");
  Serial.println(header,BIN);
  
  // ADC Setup
  analogReadExtended(16);   //increas analog pins to resolution to 16-bits
}

// In order for GPS sonde to send data, it must be connected to a GPS and receive utcSec time

void loop() {
  updateGPS();
  tempCounts = analogRead(TEMP_PIN);
  if (tempCounts < 4096) {    //add in leading zeroes to keep packet length uniform
    Serial.print(0, HEX);
    if (tempCounts < 256) {
      Serial.print(0, HEX);
      if (tempCounts < 8) {
        Serial.print(0, HEX);
      }
    }
  } 
  Serial.println(tempCounts, HEX);  //display temperature counts
  Serial.println(utcSec);
  // Send Packet based on timed interval i.e. modulus of timer with sync number
  delay(100);
  
  if ( (((utcSec % (NUM_PAYLOADS * 2)) + 1 ) == (payloadID * 2)) && newData == 1 ) {  //transmits at rate dependent upon number of payloads available. one payload transmits every 2 seconds, so resolution is 2 seconds * number of payloads
    newData = 0;  //sets false so that it does not retransmit the data until new data available
    sendDataRadio(); // Data sending command
    Serial.println("OK");
  }
}
