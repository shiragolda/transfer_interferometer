/*
  Feedforward foor laser current. 
  Reads the voltage signal being sent to the laser piezos 
    and sends a proportional voltage to the current modulation
 */
 
#include <analogShield.h>
#include <SPI.h>  //required for ChipKIT but does not affect Arduino

float f = 1;

void setup() {
 SPI.setClockDivider(SPI_CLOCK_DIV2);
 /* Open serial communications, initialize output ports: */
 Serial.begin(115200);
 Serial.println("Feedforward to current");
 Serial.println("Set line-ending to Newline");
 Serial.println("Feedforward scaling term: " + String(f,4) + " - to change, type [f]");

}

float toVoltage(float bits) {
  return (bits-32768)/6553.6;
}

float toBits(float voltage) {
  return voltage*6553.6+32768;
}

/* Accept a serial input - float */
float floatIn() {
  while(!Serial.available()){} //Wait for a serial input
  if(Serial.available()) //Throw away the newline
    Serial.read();
  while(!Serial.available()){} //Wait again
  return Serial.parseFloat(); //parse the next float
}

void loop() {
  // Listen for serial input
  byte byte_read = Serial.read();

  if(byte_read == 'f') {
    Serial.println("Feedforward scaling = " + String(f,4) + " - enter a new value for feedforward:");
    f = floatIn();
    Serial.print("Feedforward scaling set to: ");
    Serial.println(f,4);
  }

  // Write out the feedforward voltage
  // sensitivity for 423 is 100uA/1V
  float sig_in = toVoltage(analog.read(0));
  float sig_out = f*sig_in;
  analog.write(0,toBits(sig_out));
  //Serial.println(sig_out,6);

}
