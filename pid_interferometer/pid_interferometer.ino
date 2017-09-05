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
GlobalParameter channels {"Number of channels to lock",3};
int n = channels.param_value;

/* Adjust Initial Ramp Parameters */
//GlobalParameter freq = {"Scan Frequency [Hz]",148.9}; //in Hz
GlobalParameter freq = {"Scan Frequency [Hz]",200.1}; //in Hz
float period = 1000000/freq.param_value; //in micros
GlobalParameter amp = {"Scan Amplitude [V]",0.6}; // in V

GlobalParameter error_ignore = {"Error signal cut-off [us]",500}; //ignore error signals larger than this
GlobalParameter error_chan = {"Error signal monitor channel", 0}; //which channel to monitor on OUT3
int m = error_chan.param_value;


/* Input Initial PID Control Parameters */
ChannelParameter p_term = {"Proportional gain",{0.02,0.01,0.005}};   // proportional gain terms
ChannelParameter i_time = {"Integration time [ms]",{2,20,20}};    // integration time constant in microseconds
ChannelParameter alpha = {"Low-pass filter constant",{0.9,0.8,0.8}};  // low pass filter constant
ChannelParameter setpoint = {"Lock-point setpoint [us]",{500,100,100}};

/* Global variables */
unsigned long ramp_time; // time since ramp started
unsigned long time_at_ramp_reset = 0;

float sig_in[3];
float previous_sig_in[3];
unsigned long previous_ramp_time;
float sig_offset[3];

boolean lock_state = false;  // 0 is scanning, 1 is locked
boolean automatic_setpoint = true; //0 will use user-defined setpoint, 1 will use automatic setpoint detection

float limit=2; //limit output voltage (on top of 2.5V offset), in V

float pi_out[3] = {0,0,0};
float lock_point[3];
float error[3];
float accumulator[3] = {0,0,0};
float error_previous[3] = {0,0,0};

unsigned int count = 0;
unsigned int loop_counter = 0;

void setup() {
 SPI.setClockDivider(SPI_CLOCK_DIV2);
 /* Open serial communications, initialize output ports: */
 Serial.begin(115200);
 Serial.println("Locking the interferometer - use OUT0 for interferometer piezo");
 Serial.println("Set line-ending to 'no line-ending'");
 Serial.println("Number of channels to lock: " + String(n) + " - to change, type [n]");
 Serial.println("To toggle between scan/lock, type [y]."); 
 Serial.println("To change the locking setpoint, type [s]."); 
 Serial.println("To toggle between automatic setpoint determination and user-defined setpoint, type [t].");
 Serial.println("");
 Serial.println("Scan amplitude [V]: " + String(amp.param_value,4) + " - to change, type [a]."); 
 Serial.println("Scan frequency [Hz]: " + String(freq.param_value,4) + " - to change, type [f]."); 
 Serial.println("");
 Serial.println("To display all current locking parameters, type [d].");
 Serial.println("To change proportional gain, type [p].");
 Serial.println("To change integral gain time constant, type [i]");
 Serial.println("To change low-pass filter constant alpha, type [l]");
 Serial.println("");
 Serial.println("automatic setpoint detection on");
 Serial.println("scanning mode");
 

 analog.write(32768,32768,32768,32768,true);

}

float toVoltage(float bits) {
  return (bits-32768)/6553.6; // Convert a number of bits 0<b<65536 to a voltage -5V<V<5V 
}

float toBits(float voltage) {
  return voltage*6553.6+32768; // Convert a voltage -5V<V<5V to a number of bits 0<b<65536
}

/* Accept a serial input - float */
float floatIn() {
//  while(!Serial.available()){} //Wait for a serial input
//  if(Serial.available()){Serial.read();} //Throw away the newline  
  while(!Serial.available()){} //Wait again
  return Serial.parseFloat(); //parse the next float
}

/* Accept a serial input - integer */
int intIn() {
//  while(!Serial.available()){} //Wait for a serial input
//  if(Serial.available()){Serial.read();} //Throw away the newline
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

/* Generate assymetrical triangle ramp with amplitude Amp and offset 2.5V */
float rampOut() {
  if(ramp_time<=(0.9*period))
    return ((amp.param_value)/(0.9*period))*ramp_time + (2.5-amp.param_value/2);
  else 
    return (-amp.param_value/(0.1*period))*(ramp_time-(0.9*period)) + (2.5+amp.param_value/2);
}

float GetPIValue(int i) { //note: increasing piezo voltage moves peaks to the left
    error[i] = 0.001*(lock_point[i]-setpoint.param_value[i]); // [ms]
    error[i] = (alpha.param_value[i]*error_previous[i]) + (1-alpha.param_value[i])*error[i];

    accumulator[i] += error[i];  // accumulator is sum of errors 
    
    error_previous[i] = error[i];
    
    float pi = p_term.param_value[i]*(error[i]+((1/i_time.param_value[i])*accumulator[i]*(period/1000)));

    //Limit output voltage to -limit V < Vout < limitV */
    if(pi>=limit) {pi = limit;}
    else if(pi<=-limit) {pi = -limit;}  

    return pi;
}


void scanMode() {
  time_at_ramp_reset = micros(); 
  do {
    ramp_time = micros()-time_at_ramp_reset;
    analog.write(toBits(rampOut()),toBits(2.5),toBits(2.5),true);
  } while(ramp_time<period);
}

void findSetPoint() {

  for(int j=0;j<2;j++) {
    count = 0;
    float max_sig[3] = {-5,-5,-5};
    float min_sig[3] = {5,5,5};
    int detect_flag[3] = {0,0,0};
    float crossing_time[3];
    
    time_at_ramp_reset = micros();
    do {
      // write out ramp + pi calculated from previous iteration
      count ++;
      ramp_time = micros()-time_at_ramp_reset;
      analog.write(toBits(rampOut()),toBits(2.5),toBits(2.5),true);
  
      for(int i=0;i<n;i++) {
        // read signal
        sig_in[i] = toVoltage(analog.read(i));
  
        if(sig_in[i] > max_sig[i]) {max_sig[i] = sig_in[i];}
        else if(sig_in[i] < min_sig[i]) {min_sig[i] = sig_in[i];}
  
        if(j == 1) {        
          // detect lock-point
          if(count>1 && detect_flag[i] == 0 && previous_sig_in[i] <= sig_offset[i] && sig_in[i]>sig_offset[i]) {
            crossing_time[i] = ramp_time;
            detect_flag[i] = 1;
          }
        }
        previous_sig_in[i] = sig_in[i];
      }
      previous_ramp_time = ramp_time;
    } while(ramp_time<period);
    

  setpoint.param_value[0] = crossing_time[0];
  setpoint.param_value[1] = crossing_time[0] - crossing_time[1]; //compare with interferometer/780 setpoint to filter out fast interferometer fluctuations
  setpoint.param_value[2] = crossing_time[0] - crossing_time[2];
  
  for(int i=0;i<n;i++) {
    sig_offset[i] = (max_sig[i]+min_sig[i])/2;
    }
  }
}

void lockMode() { 
  count = 0;
  float max_sig[3] = {-5,-5,-5};
  float min_sig[3] = {5,5,5};
  float detect_flag[3] = {0,0,0};
  
  time_at_ramp_reset = micros();
  do {
    // write out ramp + pi calculated from previous iteration
    count ++;
    ramp_time = micros()-time_at_ramp_reset;
    analog.write(toBits(rampOut()+pi_out[0]),toBits(2.5-1.0*pi_out[1]),toBits(2.5+pi_out[2]),toBits(error[m]),true);

    for(int i=0;i<n;i++) {
      // read signal
      sig_in[i] = toVoltage(analog.read(i));

      if(sig_in[i] > max_sig[i]) {max_sig[i] = sig_in[i];}
      else if(sig_in[i] < min_sig[i]) {min_sig[i] = sig_in[i];}

      if(count>1 && detect_flag[i] == 0 && previous_sig_in[i] <= sig_offset[i] && sig_in[i]>sig_offset[i]) {
        lock_point[i] = ramp_time-((sig_in[i]-sig_offset[i])/(sig_in[i]-previous_sig_in[i]))*(ramp_time-previous_ramp_time); //linear interpolation
        detect_flag[i] = 1;
        if(abs(lock_point[i]-setpoint.param_value[i]) > error_ignore.param_value) {lock_point[i] = setpoint.param_value[i];} //ignore errors > 2ms
      }
     
      previous_sig_in[i] = sig_in[i];
      
    }
    previous_ramp_time = ramp_time;
  } while(ramp_time<period);
  
  lock_point[1] = lock_point[0] - lock_point[1]; //compare with interferometer/780 setpoint to filter out fast interferometer fluctuations
  lock_point[2] = lock_point[0] - lock_point[2];
  
  for(int i=0;i<n;i++) {
    sig_offset[i] = (((max_sig[i]+min_sig[i])/2)+sig_offset[i])/2; //slow averaging of the offset
    pi_out[i] = GetPIValue(i);
  }

//  Serial.print("Max: ");
//  Serial.println(max_sig[0],4);
//  Serial.print("Min: ");
//  Serial.println(min_sig[0],4);
//  Serial.print("Lock point 423: ");
//  Serial.println(lock_point[1],4);
//    Serial.println(count);
}


void loop() {
 
  // Listen for serial input
  byte byte_read = Serial.read();
  if(byte_read == 'n') {channels = UpdateGlobalParameter(channels); n = channels.param_value;}
  if(byte_read == 'm') {error_chan = UpdateGlobalParameter(error_chan); m = error_chan.param_value;}
  if(byte_read == 'a') {amp = UpdateGlobalParameter(amp); }
  if(byte_read == 'f') {freq = UpdateGlobalParameter(freq); period = 1000000/freq.param_value;}
  if(byte_read == 'p') {p_term = UpdateChannelParameter(p_term); }
  if(byte_read == 'i') {i_time = UpdateChannelParameter(i_time); }
  if(byte_read == 'l') {alpha = UpdateChannelParameter(alpha); }
  if(byte_read == 's') {setpoint = UpdateChannelParameter(setpoint); }
  if(byte_read == 'e') {error_ignore = UpdateGlobalParameter(error_ignore); }

  if(byte_read == 'd') {
    Serial.println("Current Locking Parameters:");
    Serial.print("Proportional gains:                 ");
    Serial.print(p_term.param_value[0],4);    
    Serial.print(",");
    Serial.print(p_term.param_value[1],4);
    Serial.print(",");
    Serial.println(p_term.param_value[1],4);
    Serial.print("Integral gain time constants [ms]:  ");
    Serial.print(i_time.param_value[0],4);    
    Serial.print(",");
    Serial.print(i_time.param_value[1],4);
    Serial.print(",");
    Serial.println(i_time.param_value[1],4);
    Serial.print("Low-pass filter constants alpha:    ");
    Serial.print(alpha.param_value[0],4);    
    Serial.print(",");
    Serial.print(alpha.param_value[1],4);
    Serial.print(",");
    Serial.println(alpha.param_value[1],4);
  }


  if(byte_read == 't') {
    automatic_setpoint =!automatic_setpoint;
    if(automatic_setpoint == true) {Serial.println("Automatic setpoint detection on");}
    else {Serial.println("Automatic setpoint detection off - user defined setpoint");}
  }

  
  /* Listen for a scan/lock toggle command: */
  if(byte_read == 'y') {
    lock_state = !lock_state;
    
    if(lock_state == false) {
      Serial.println("scanning mode");
    }

    else if(lock_state == true) {
      Serial.println("locking mode");
      loop_counter = 0;
      if(automatic_setpoint == 1) {findSetPoint();}

      for(int i=0;i<n;i++) {
        accumulator[i] = 0;
        pi_out[i] = 0;
        error_previous[i] = 0;
        Serial.print("Set-point: ");
        Serial.print(setpoint.param_value[i],3);
        Serial.println(" us");
      }
    }   
  }
  
  /* Write output for scanning mode: */
  if(lock_state == false) {
    scanMode();
  }

  /* Write output for locking mode: */
  if(lock_state == true) {
    loop_counter ++;
    lockMode(); // detect lock point + calculate PI output for next iteration

    /* AUTO-RELOCK */
    for(int i=0;i<n;i++) {
      if(abs(pi_out[i]) == limit) {
        Serial.print("Voltage limit reached on channel ");
        Serial.println(i);
        accumulator[i] = 0;
        pi_out[i] = 0;
        error_previous[i] = 0;
        Serial.println("Relocked");
      }
    }
    if(loop_counter==1) { 
      Serial.print("Samples per loop = ");
      Serial.println(count);
    }
    /*Write to serial for data logging */
    if(loop_counter%500 == 0) {  
      Serial.print(error[0],4); //ms
      Serial.print(',');
      Serial.print(pi_out[0],4); //V
      Serial.print(',');
      Serial.print(error[1],4); //ms
      Serial.print(',');
      Serial.print(pi_out[1],4); //V
      Serial.print(',');
      Serial.print(error[2],4); //ms
      Serial.print(',');
      Serial.println(pi_out[2],4); //V
    } 
  }

}

