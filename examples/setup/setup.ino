#include "Wire.h"
#include "bq34z100.h"
bq34z100 bq;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);//Start Serial comms
  Wire.begin();
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  //Auto calibration (Run this once on the bq34z100 to calibrate most of the internal registers
  bq.setup(0x101,3,8000,12369,1000);
  //Battery Chemistry
  //Number of series Batteries
  //Battery capacity in mAh
  //Current voltage on the pack in mV (eg 12.369V)
  // Current being applied to the pack for currentShunt Cal in mA (must be > 200mA)
  delay(200);
}

void loop() {

  digitalWrite(13, HIGH);
  Serial.println(bq.getVoltage(), DEC);
  digitalWrite(13, LOW);
  delay(2000);//delay 2 seconds
}
