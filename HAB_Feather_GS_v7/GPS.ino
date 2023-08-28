/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////********************* GPS Functions **********************///////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

/************************** GPS ******************************/
void initGPS() {
  unsigned long gpsTimer = millis();
  bool set_airborne = gps.setAirborne();  //sets airborne mode so the module can operate at high altitudes

  while (Serial1.available()) Serial1.read(); //read all data available in gps serial port to clear it

  delay(50);
  while (!set_airborne && (millis() - gpsTimer < 1 * 60 * 1000 )) { //sets airborne mode. if not successful in 60s, stops attempting to set
    set_airborne = gps.setAirborne();
    delay(1);
    Serial.println("Airborne Mode Not Set!");
  }
  if (set_airborne) Serial.println("GPS OK"); //if airborne set, print this
  else Serial.println("GPS Failure"); //if airborne mode faild to initialize, print this
}

void updateGPS() {
  //  Serial.println("gps loop");
  gps.update();

  //get measurements for location and time
  numSats = gps.getSats();
  utcSec = gps.getSecond();
  utcMin = gps.getMinute();
  utcHour = gps.getHour();
}