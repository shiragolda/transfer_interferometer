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
GlobalParameter channels {"Number of channels to lock",1};
int n = channels.param_value;

/* Adjust Initial Ramp Parameters */
GlobalParameter freq = {"Scan Frequency [Hz]",97.5}; //in Hz
float period = 1000000/freq.param_value; //in micros
GlobalParameter amp = {"Scan Amplitude [V]",4}; // in V


/* Input Initial PID Control Parameters */
ChannelParameter p_term = {"Proportional gain",{0.7,0.01,0.1}};   // proportional gain terms
ChannelParameter i_time = {"Integration time [us]",{100000,50000,500000}};    // integration time constant in microseconds
ChannelParameter alpha = {"Low-pass filter constant",{0.4,0.4,0.4}};  // low pass filter constant

/* Measured Values: */
ChannelParameter fringe_separation = {"Fringe period [us]",{3000,1320,0}}; //in micros

/* Global variables */
unsigned long current_time; // time since program begun
unsigned long ramp_time; // time since ramp started
long time_at_ramp_reset = 0;

int lock_state = 0;  // 0 is scanning, 1 is locked

const int s = 4;

float lock_point[3];
float set_point[3] = {0,0,0};
float error[3];
float accumulator[3] = {0,0,0};
float pi_out[3] = {0,0,0};
float p_prime[3] = {0,0,0};

unsigned long loop_counter = 0;

float zerov = 32768.0; //Zero volts

void setup() {
 SPI.setClockDivider(SPI_CLOCK_DIV2);
 /* Open serial communications, initialize output ports: */
 Serial.begin(115200);
 Serial.println("Locking the interferometer - use OUT0 for interferometer piezo");
 Serial.println("Set line-ending to Newline");
 Serial.println("Number of channels to lock: " + String(n) + " - to change, type [n]");
 Serial.println("To toggle between scan/lock, type [y]."); 
 Serial.println("");
 Serial.println("Scan amplitude: " + String(amp.param_value,4) + " - to change, type [a]."); 
 Serial.println("Scan frequency: " + String(freq.param_value,4) + " - to change, type [f]."); 
 Serial.println("Proportional gains: " + String(p_term.param_value[0],3) + ", " + String(p_term.param_value[1],3) + ", " + String(p_term.param_value[2],3) + " - to change, type [p].");
 Serial.println("Integral gain times: " + String(i_time.param_value[0],4) + ", " + String(i_time.param_value[1],2) + ", " + String(i_time.param_value[2],2) + " us - to change, type [i].");
 Serial.println("Low-pass filter constant alpha: " + String(alpha.param_value[0],4) + ", " + String(alpha.param_value[1],2) + ", " + String(alpha.param_value[2],2) + " - to change, type [l].");
 Serial.println("");
 Serial.println("scanning mode");

 analog.write(zerov,zerov,zerov,zerov,true);
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


float getOffset() {
  float offset = toVoltage(analog.read(3));
  offset = (offset/3.3)+0.5; //scales 0-5V down to 0.5 - 2 V (allows for +-0.5 mV servo voltage to be added safely)
  return offset;
}


/* Generate triangle ramp from 0V to Amp */
float rampOut() {
  float ramp;
  if(ramp_time<=(period/2))
    ramp = (amp.param_value/(period/2))*ramp_time;
  else 
    ramp = -(amp.param_value/(period/2))*ramp_time + 2*amp.param_value;
  return ramp;
}

/* Calculate rolling derivative of a signal */
//NOTE: new values are stored at the BEGINNING of the array!
//NOTE: produces nonsense until slice_count values have been added to the array
float rollingDerivative(int i, int slice_count, float history_array[][s], float new_value) {
  float left_sum = 0;
  float right_sum = 0;
  for(int j=slice_count-1;j>0;j--) {
    history_array[i][j] = history_array[i][j-1];
    if(j>=slice_count/2) {
      right_sum += history_array[i][j];
    }
    if(j<slice_count/2) {
      left_sum += history_array[i][j];
    }
  }
  history_array[i][0] = new_value;
  left_sum += history_array[i][0];
  return((left_sum-right_sum)/(float)slice_count);
}


void scanMode() {
  //float offset_voltage = getOffset();
  time_at_ramp_reset = micros(); 
  do {
    current_time = micros(); //time in mircoseconds
    ramp_time = current_time-time_at_ramp_reset;
    analog.write(toBits(rampOut()),toBits(0),toBits(0),true);
  } while(ramp_time<period);
}


// integers "detectLockPoint" and "getPI" toggle those parts of the code
void lockMode(int getPI) {
  float sig_in[3];
  float slope[3];
  float new_max_sig_in[3] = {-5,-5,-5};
  float new_min_sig_in[3] = {5,5,5};
  float smallest_slope[3] = {100,100,100};
  long start_time[3] = {-1,-1,-1};
  float storage_array[3][s];
  float max_slope[3] = {-100,-100,-100};
  long max_slope_time[3] = {0,0,0};
  int count = 0;

  //float offset_voltage = getOffset();
  time_at_ramp_reset = micros();
  do {
    // write out ramp + pi calculated from previous iteration
    count ++;
    current_time = micros();
    ramp_time = current_time-time_at_ramp_reset;
    analog.write(toBits(rampOut()+pi_out[0]),toBits(-1.0*pi_out[1]),toBits(pi_out[2]),toBits(pi_out[0]),true);

    for(int i=0;i<n;i++) {
      // read signal
      sig_in[i] = toVoltage(analog.read(i));

      // calculate derivative
      slope[i] = rollingDerivative(i,s,storage_array,sig_in[i]);
             
      // wait to store s values for slope averaging, find first peak
      if(count>s && ramp_time<fringe_separation.param_value[i] && abs(slope[i])<smallest_slope[i]) {
        smallest_slope[i] = slope[i];
        start_time[i] = ramp_time;
      }
      
      // find first rising zero crossing (steepest positive slope) after the first peak has passed
      if(ramp_time>start_time[i] && ramp_time < fringe_separation.param_value[i]+start_time[i] && slope[i] > max_slope[i]){
        max_slope[i] = slope[i];
        max_slope_time[i] = ramp_time;
      }
    }

  } while(ramp_time<period);
  
  for(int i=0;i<n;i++) {
    lock_point[i] = max_slope_time[i];

    if(getPI == 1) { //note: increasing piezo voltage moves peaks to the left
      error[i] = 0.001*(lock_point[i]-set_point[i]); //Scale down because error is in microseconds - big number!
      accumulator[i] += error[i];  // accumulator is sum of errors 
      float p = p_term.param_value[i]*error[i];
      p = (alpha.param_value[i]*p_prime[i])+(1-alpha.param_value[i])*p;
      pi_out[i] = (p+(p_term.param_value[i]*(1/i_time.param_value[i])*accumulator[i]*period));
      //Limit output voltage to -0.5V < Vout < 0.5V */
      if(pi_out[i]>=0.5)
        pi_out[i] = 0.5;
      if(pi_out[i]<=-0.5)
        pi_out[i] = -0.5;
      p_prime[i] = p;
    }
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
  
  
  n = channels.param_value;
  period = 1000000/freq.param_value; //in micros

  
  /* Listen for a scan/lock toggle command: */
  if(byte_read == 'y') {
    lock_state = !lock_state;
    
    if(lock_state == 0)
      Serial.println("scanning mode");
      for(int i=0;i<n;i++) {
        accumulator[i] = 0;
        pi_out[i] = 0;
        p_prime[i] = 0;
      }
    if(lock_state == 1) {
      Serial.println("locking mode");
      loop_counter = 0;
      for(int i=0;i<n;i++) {
        accumulator[i] = 0;
        pi_out[i] = 0;
        p_prime[i] = 0;
      }
      lockMode(0); // detect lock-point - set peak time as PI setpoint
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
    lockMode(1); // detect lock point + calculate PI output for next iteration


    /*Write to serial for data logging */
    if(loop_counter%400 == 0) {
      Serial.print(error[0],4);
      Serial.print(',');
      Serial.print(pi_out[0],4);
      Serial.print(',');
      Serial.print(error[1],4);
      Serial.print(',');
      Serial.println(pi_out[1],4);
    } 
  }

}

