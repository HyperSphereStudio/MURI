/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////********************* Packet Definitions **********************///////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

void sendDataRadio() {
  /****** IDENTIFIER *******/
  dataPack[0] = header;       //9 bit ERAU ID , 3 bit Payload ID, 4 bit numSats
  dataPack[1] = header >> 8;

  /*******GPS DATA*******/
  byte* point_alt = (byte*) &alt;
  dataPack[2] = point_alt[0];
  dataPack[3] = point_alt[1];
  dataPack[4] = point_alt[2];
  dataPack[5] = point_alt[3];

  byte* point_lat = (byte*) &latitude;
  dataPack[6] = point_lat[0];
  dataPack[7] = point_lat[1];
  dataPack[8] = point_lat[2];
  dataPack[9] = point_lat[3];

  byte* point_lon = (byte*) &longitude;
  dataPack[10] = point_lon[0];
  dataPack[11] = point_lon[1];
  dataPack[12] = point_lon[2];
  dataPack[13] = point_lon[3];

//  /****** BATTERY VOLTAGE *******/
//  vbat = analogRead(A0);
//  
//  dataPack[16] = vbat >> 2;
//  /****** TEMPERATURE *******/
  tempCounts = analogRead(TEMP_PIN);
  dataPack[14] = tempCounts;
  dataPack[15] = tempCounts >> 8;
//  /****** PRESSURE *******/
    pressure = baro.readPressure();
    presdata = pressure*PRESSURE_CONVERSION;
  dataPack[16] = presdata;      
  dataPack[17] = presdata >> 8;

//  /****** PACKET NUMBER *******/
  dataPack[18] = count1; 
  count1++;   //increment packet number 
  
  //extra useable bytes. 20 byte max for 2Hz datarate

  rf95.send(dataPack, RADIO_SIZE);    //transmit the packet using LoRa radio
  for (int i = 0; i < RADIO_SIZE; i++) {  //display the packet in hex
    Serial.print(dataPack[i], HEX); Serial.print(" ");
  }
  Serial.write("\n");
  delay(10);
  
}
