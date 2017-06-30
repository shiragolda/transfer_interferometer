
#include <analogShield.h>
#include <SPI.h>  //required for ChipKIT but does not affect Arduino

/* Adjust Initial Ramp Parameters */
float freq = 97.5; //in Hz
float amp = 0.1; // in V

/* Global variables */
float period = 1000000/freq; //in micros
unsigned long current_time; // time since program begun
unsigned long ramp_time; // time since ramp started
long time_at_ramp_reset = 0;

void setup() {
 SPI.setClockDivider(SPI_CLOCK_DIV2);
 /* Open serial communications, initialize output ports: */
 Serial.begin(115200);
 Serial.println("Locking the interferometer - use OUT0 for interferometer piezo");
 Serial.println("Set line-ending to Newline");
 Serial.println("Scan amplitude: " + String(amp,6) + " - to change, type [a]."); 

}

float toVoltage(float bits) {
  return (bits-32768)/6553.6;
}

float toBits(float voltage) {
  return voltage*6553.6+32768;
}

/* Generate triangle ramp from 0V to Amp */
float rampOut() {
  float ramp;
  float offset = 2.5-amp/2; // centre the scan between 0 and 5 V
  if(ramp_time<=(period/2))
    ramp = (amp/(period/2))*ramp_time+offset;
  else 
    ramp = -(amp/(period/2))*ramp_time + 2*amp + offset;
  return ramp;
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
  byte byte_read = Serial.read();
  
  if(byte_read == 'a') {
    Serial.println("Scan amplitude = " + String(amp,6) + " V - enter a new value for amplitude:");
    amp = floatIn();
    Serial.print("Amplitude set to [V]: ");
    Serial.println(amp,6);
  }

  if(byte_read == 'f') {
    Serial.println("Scan frequency = " + String(freq) + " Hz - enter a new value for frequency:");
    freq = floatIn();
    period = 1000000/freq;
    Serial.print("Frequency set to [Hz]: ");
    Serial.println(freq);
  }

  
  // ramp the 423 laser
  time_at_ramp_reset = micros();
  do {
    // write out ramp + pi calculated from previous iteration
    current_time = micros();
    ramp_time = current_time-time_at_ramp_reset;
    analog.write(toBits(rampOut()),toBits(rampOut()),true);
  } while(ramp_time<period);

}
