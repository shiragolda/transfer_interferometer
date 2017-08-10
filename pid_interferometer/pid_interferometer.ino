/*
  PID control for transfer interferometer
  Use output pin 0 for interferometer piezo
 */

#include <analogShield.h>
#include <SPI.h>  //required for ChipKIT but does not affect Arduino

typedef struct {
  String param_name;
  float param_value;
} GlobalParameter ;

typedef struct {
  String param_name;
  float param_value[3];
} ChannelParameter ;

/* Number of channels to lock */
GlobalParameter channels {"Number of channels to lock",2};
int n = channels.param_value;

/* Adjust Initial Ramp Parameters */
GlobalParameter freq = {"Scan Frequency [Hz]",97.5}; //in Hz
float period = 1000000/freq.param_value; //in micros
GlobalParameter amp = {"Scan Amplitude [V]",0.6}; // in V


/* Input Initial PID Control Parameters */
ChannelParameter p_term = {"Proportional gain",{0.03,0.03,0.1}};   // proportional gain terms
ChannelParameter i_time = {"Integration time [ms]",{1000,1000,500000}};    // integration time constant in microseconds
ChannelParameter alpha = {"Low-pass filter constant",{0.4,0.4,0.4}};  // low pass filter constant

/* Measured Values: */
ChannelParameter fringe_separation = {"Fringe period [us]",{3200,1320,0}}; //in micros

/* Global variables */
unsigned long current_time; // time since program begun
unsigned long ramp_time; // time since ramp started
long time_at_ramp_reset = 0;

int lock_state = 0;  // 0 is scanning, 1 is locked

const int s = 4;

float limit=2;

float lock_point[3];
float set_point[3] = {0,0,0};
float error[3];
float accumulator[3] = {0,0,0};
float pi_out[3] = {0,0,0};
float p_prime[3] = {0,0,0};

float previous_min_sig[3];
float amplitude[3];
float sig_offset[3];

unsigned long loop_counter = 0;

float zerov = 32768.0; //Zero volts

const float pi = 3.141592654;

float sin_table[92];
void GenerateLookupTable(void) {
  for(int x=0;x<92;x++) {
    sin_table[x] = sin(2*pi*x/360);
  }
}

void setup() {
 SPI.setClockDivider(SPI_CLOCK_DIV2);
 /* Open serial communications, initialize output ports: */
 Serial.begin(115200);
 Serial.println("Locking the interferometer - use OUT0 for interferometer piezo");
 Serial.println("Set line-ending to Newline");
 Serial.println("Number of channels to lock: " + String(n) + " - to change, type [n]");
 Serial.println("To toggle between scan/lock, type [y]."); 
 Serial.println("");
 Serial.println("Scan amplitude [V]: " + String(amp.param_value,4) + " - to change, type [a]."); 
 Serial.println("Scan frequency [Hz]: " + String(freq.param_value,4) + " - to change, type [f]."); 
 Serial.println("Proportional gains: " + String(p_term.param_value[0],3) + ", " + String(p_term.param_value[1],3) + ", " + String(p_term.param_value[2],3) + " - to change, type [p].");
 Serial.println("Integral gain times [ms]: " + String(i_time.param_value[0],4) + ", " + String(i_time.param_value[1],2) + ", " + String(i_time.param_value[2],2) + " us - to change, type [i].");
 Serial.println("Low-pass filter constant alpha: " + String(alpha.param_value[0],4) + ", " + String(alpha.param_value[1],2) + ", " + String(alpha.param_value[2],2) + " - to change, type [l].");
 Serial.println("Measured fringe periodicity [us]: " + String(fringe_separation.param_value[0],4) + ", " + String(fringe_separation.param_value[1],2) + ", " + String(fringe_separation.param_value[2],2) + " - to change, type [s].");
 Serial.println("");
 Serial.println("scanning mode");

 analog.write(zerov,zerov,zerov,zerov,true);

 //GenerateLookupTable();
}

float toVoltage(float bits) {
  return (bits-32768)/6553.6; // Convert a number of bits 0<b<65536 to a voltage -5V<V<5V 
}

float toBits(float voltage) {
  return voltage*6553.6+32768; // Convert a voltage -5V<V<5V to a number of bits 0<b<65536
}

/* Accept a serial input - float */
float floatIn() {
  while(!Serial.available()){} //Wait for a serial input
  if(Serial.available()){Serial.read();} //Throw away the newline  
  while(!Serial.available()){} //Wait again
  return Serial.parseFloat(); //parse the next float
}

/* Accept a serial input - integer */
int intIn() {
  while(!Serial.available()){} //Wait for a serial input
  if(Serial.available()){Serial.read();} //Throw away the newline
  while(!Serial.available()){} //Wait again
  return Serial.parseInt(); //parse the next int
}

GlobalParameter UpdateGlobalParameter(GlobalParameter param) {
  Serial.print(param.param_name + " = ");
  Serial.print(param.param_value,4);
  Serial.println(" - enter a new value for " + param.param_name + ":");
  float new_value = floatIn();
  Serial.print(param.param_name + " = ");
  Serial.println(new_value,4);
  GlobalParameter new_param = {param.param_name,new_value};
  return new_param;
}

ChannelParameter UpdateChannelParameter(ChannelParameter param) {
  Serial.println("Pick a channel (0 = cavity, 1 = 423 laser ,2 = 453 laser)");
  int channel = intIn();
  Serial.print(param.param_name + " = ");
  Serial.print(param.param_value[channel],4);
  Serial.println(" - Enter a new value for " + param.param_name + ":");
  float new_value = floatIn();
  Serial.print(param.param_name + " = ");
  Serial.println(new_value,4);
  ChannelParameter new_param = param;
  if(channel == 0 || channel == 1 || channel == 2)
    new_param.param_value[channel] = new_value;
  return new_param;
}

/* Generate triangle ramp with amplitude Amp and offset 2.5V */
float rampOut() {
  float ramp;
  if(ramp_time<=(period/2))
    //ramp = (amp.param_value/(period/2))*ramp_time + (2.5-amp.param_value/2);
    ramp = (2*amp.param_value)/(period)*ramp_time + (2.5-amp.param_value/2);
  else 
    //ramp = -(amp.param_value/(period/2))*ramp_time + 2*amp.param_value + (2.5-amp.param_value/2);
    ramp = (-2*amp.param_value)/(period)*ramp_time +(2.5+3*amp.param_value/2);
  return ramp;
}

float GetPIValue(int i) { //note: increasing piezo voltage moves peaks to the left
    error[i] = 0.001*(lock_point[i]-set_point[i]); //Scale down because error is in microseconds - big number!
    accumulator[i] += error[i];  // accumulator is sum of errors 
    float p = p_term.param_value[i]*error[i];
    p = (alpha.param_value[i]*p_prime[i])+(1-alpha.param_value[i])*p;
    float pi = (p+(p_term.param_value[i]*(1/i_time.param_value[i])*accumulator[i]*(period/1000)));
    //Limit output voltage to -limit V < Vout < limitV */
    if(pi>=limit)
      pi = limit;
    if(pi<=-limit)
      pi = -limit;
    p_prime[i] = p;

    return pi;
}


void scanMode() {
  time_at_ramp_reset = micros(); 
  do {
    current_time = micros(); //time in mircoseconds
    ramp_time = current_time-time_at_ramp_reset;
    analog.write(toBits(rampOut()),toBits(2.5),toBits(2.5),true);
  } while(ramp_time<period);
}

/* 
 *  scans and reads in photodiode signal
 *  calculates max,min,amplitude,offset
 *  if detect_lock_point = 1: calculates lock point
 */
void analyzeSignal(int detect_lock_point) { 
  int count = 0;
  float sig_in[3];
  float previous_sig_in[3];
  float max_sig[3] = {-5,-5,-5};
  float min_sig[3] = {5,5,5};
  int detect_flag[3] = {0,0,0};
  int start_detect_flag[3] = {0,0,0};

  time_at_ramp_reset = micros();
  do {
    // write out ramp + pi calculated from previous iteration
    count ++;
    current_time = micros();
    ramp_time = current_time-time_at_ramp_reset;
    analog.write(toBits(rampOut()+pi_out[0]),toBits(2.5-1.0*pi_out[1]),toBits(2.5+pi_out[2]),true);
    //analog.write(toBits(rampOut()),toBits(-1.0*pi_out[1]),toBits(pi_out[2]),true);

    for(int i=0;i<n;i++) {
      // read signal
      sig_in[i] = toVoltage(analog.read(i));

      if(sig_in[i] > max_sig[i]) {max_sig[i] = sig_in[i];}
      if(sig_in[i] < min_sig[i]) {min_sig[i] = sig_in[i];}

      if(detect_lock_point == 1) {
        // find first minimum to trigger lock-point detection
//        if(start_detect_flag[i]==0 && abs(sig_in[i]-previous_min_sig[i])<= 0.2*amplitude[i]) {
//          start_detect_flag[i] = 1;
//        }
        
        // detect lock-point
        if(count>1 && detect_flag[i] == 0 && previous_sig_in[i] <= sig_offset[i] && sig_in[i]>sig_offset[i]) {
          lock_point[i] = ramp_time;
          detect_flag[i] = 1;
        }
      }
      previous_sig_in[i] = sig_in[i];
    }
  } while(ramp_time<period);

  for(int i=0;i<n;i++) {
    amplitude[i] = max_sig[i] - min_sig[i];
    sig_offset[i] = (max_sig[i]+min_sig[i])/2;
    previous_min_sig[i] = min_sig[i];
  }
//  Serial.print("Max: ");
//  Serial.println(max_sig[0],4);
//  Serial.print("Min: ");
//  Serial.println(min_sig[0],4);
//  Serial.print("Amplitude: ");
//  Serial.println(amplitude[0],4);
//  Serial.print("Lock point: ");
//  Serial.println(lock_point[0],4);
}


void lockMode() {
  analyzeSignal(1);
  for(int i=0;i<n;i++) {
    pi_out[i] = GetPIValue(i);
  }
}




void loop() {
 
  // Listen for serial input
  byte byte_read = Serial.read();
  if(byte_read == 'n') {channels = UpdateGlobalParameter(channels); }
  if(byte_read == 'a') {amp = UpdateGlobalParameter(amp); }
  if(byte_read == 'f') {freq = UpdateGlobalParameter(freq); }
  if(byte_read == 'p') {p_term = UpdateChannelParameter(p_term); }
  if(byte_read == 'i') {i_time = UpdateChannelParameter(i_time); }
  if(byte_read == 'l') {alpha = UpdateChannelParameter(alpha); }
  if(byte_read == 's') {fringe_separation = UpdateChannelParameter(fringe_separation); }
  
  
  n = channels.param_value;
  period = 1000000/freq.param_value; //in micros

  
  /* Listen for a scan/lock toggle command: */
  if(byte_read == 'y') {
    lock_state = !lock_state;
    
    if(lock_state == 0) {
      Serial.println("scanning mode");
    }

    if(lock_state == 1) {
      Serial.println("locking mode");
      loop_counter = 0;
      for(int i=0;i<n;i++) {
        accumulator[i] = 0;
        pi_out[i] = 0;
        p_prime[i] = 0;
      }
      analyzeSignal(0); // detect signal params: max,min,amplitude,offset
      analyzeSignal(1); // use signal params to detect lock-point
      for(int i=0;i<n;i++) {
        set_point[i] = lock_point[i];
        Serial.print("Set-point: ");
        Serial.print(set_point[i],3);
        Serial.println(" us");
      }
    }   
  }


  
  /* Write output for scanning mode: */
  if(lock_state == 0) {
    scanMode();
  }

  /* Write output for locking mode: */
  if(lock_state == 1) {
    loop_counter ++;
    lockMode(); // detect lock point + calculate PI output for next iteration

    /* AUTORELOCK */
    



    /*Write to serial for data logging */
    if(loop_counter%500 == 0) {  
      Serial.print(error[0],4); //ms
      Serial.print(',');
      Serial.print(pi_out[0],4); //V
      Serial.print(',');
      Serial.print(error[1],4); //ms
      Serial.print(',');
      Serial.println(pi_out[1],4); //V
    } 
  }

}

