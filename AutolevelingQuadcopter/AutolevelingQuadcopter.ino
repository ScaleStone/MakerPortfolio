//https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
#include <EnableInterrupt.h>
#include <Servo.h>
#include <Wire.h>

#define SERIAL_PORT_SPEED 57600
#define RC_NUM_CHANNELS  4

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

#define RC_CH1_INPUT  8
#define RC_CH2_INPUT  9
#define RC_CH3_INPUT  10
#define RC_CH4_INPUT  11

Servo esc4;
Servo esc5;
Servo esc6;
Servo esc7;

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];
int roll, pitch, yaw, throttle;
int esc4_output, esc5_output, esc6_output, esc7_output;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, temp;
double gyro_roll, gyro_pitch, gyro_yaw;
long  acc_total_vector, acc_z, acc_y, acc_x;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
float pid_error_temp;
int pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint;
float pid_i_mem_roll, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
const int MPU = 0x68;
byte lowByte, highByte;
int i, battery_voltage;

float pid_p_gain_roll = 1.3;              //Gain setting for the roll P-controller
float pid_i_gain_roll = 0;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 17;           //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

int start = 0;
void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() {
  calc_input(RC_CH1, RC_CH1_INPUT);
}
void calc_ch2() {
  calc_input(RC_CH2, RC_CH2_INPUT);
}
void calc_ch3() {
  calc_input(RC_CH3, RC_CH3_INPUT);
}
void calc_ch4() {
  calc_input(RC_CH4, RC_CH4_INPUT);
}

void read_gyro () {
  /*Wire.beginTransmission(MPU);                             //Start communication with the gyro 
    Wire.write(0x43);                                            //Start reading @ register 43h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(MPU,6);                                 //Request 6 bytes from the gyro
    while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
    gyro_roll=Wire.read()<<8|Wire.read();                        //Read high and low part of the angular data
    if(i == 1000)gyro_roll -= gyro_roll_cal;               //Only compensate after the calibration
    gyro_pitch=Wire.read()<<8|Wire.read();                       //Read high and low part of the angular data
    if(i == 1000)gyro_pitch -= gyro_pitch_cal;             //Only compensate after the calibration
    gyro_yaw=Wire.read()<<8|Wire.read();                         //Read high and low part of the angular data
    if(i == 1000)gyro_yaw -= gyro_yaw_cal;                 //Only compensate after the calibration*/


    

 Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  temp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  gyro_roll = GyX;
  gyro_pitch = GyY;
  gyro_yaw = GyZ * -1;
  acc_x = AcY;
  acc_y = AcX*-1;
  acc_z = AcZ;
  if (i == 1000) {
    gyro_roll -= gyro_roll_cal;                                       //Only compensate after the calibration.
    gyro_pitch -= gyro_pitch_cal;                                       //Only compensate after the calibration.
    gyro_yaw -= gyro_yaw_cal;
    //Only compensate after the calibration.
  }
}

void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);
  esc4.attach(4);
  esc5.attach(5);
  esc6.attach(6);
  esc7.attach(7);

  Wire.begin();

/* Wire.beginTransmission(MPU);                             //Start communication with the gyro 
    Wire.write(0x6B);                                            //PWR_MGMT_1 register
    Wire.write(0x00);                                            //Set to zero to turn on the gyro
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(MPU);                             //Start communication with the gyro
    Wire.write(0x6B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(MPU, 1);                                //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x6B is set to:"));
    Serial.println(Wire.read(),BIN);
    
    Wire.beginTransmission(MPU);                             //Start communication with the gyro
    Wire.write(0x1B);                                            //GYRO_CONFIG register
    Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(MPU);                             //Start communication with the gyro (adress 1101001)
    Wire.write(0x1B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(MPU, 1);                                //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x1B is set to:"));
    Serial.println(Wire.read(),BIN);
                    //End the transmission*/

 Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

   Wire.beginTransmission(MPU);                                      //Start communication with the address found during search.
   Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
   Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
   Wire.endTransmission();        

    Wire.beginTransmission(MPU);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();    

    Wire.beginTransmission(MPU);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro   


    


  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(A0, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);

  start = 0;

  for (i = 0; i < 1000 ; i ++) {                          //Take 2000 readings for calibration.
    if (i % 15 == 0)digitalWrite(12, HIGH);
    //Change the led status to indicate calibration.
    read_gyro();
    //Read the gyro output.
    gyro_roll_cal += gyro_roll;                                       //Ad roll value to gyro_roll_cal.
    gyro_pitch_cal += gyro_pitch;
    gyro_yaw_cal += gyro_yaw; //Ad pitch value to gyro_pitch_cal.                                     //Ad yaw value to gyro_yaw_cal.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    esc4.write(90);
    esc5.write(90);
    esc6.write(90);
    esc7.write(90);                                                //Set digital poort 4, 5, 6 and 7 high.                                               //Set digital poort 4, 5, 6 and 7 low.
    delay(1);                                                               //Wait 3 milliseconds before the next loop.
  }
  gyro_roll_cal /= 1000;
  gyro_pitch_cal /= 1000;
  gyro_yaw_cal /= 1000;

  battery_voltage = (analogRead(0) + 65) * 1.2317;
  
  digitalWrite(12, LOW);
}

void loop() {
  rc_read_values();
  read_gyro();

  roll = rc_values[RC_CH1] - 1500;
  roll /= 2;
  pitch = rc_values[RC_CH2] - 1500;
  pitch /= 2;
  yaw = rc_values[RC_CH4] - 1500;
  yaw /= 2;

  throttle = rc_values[RC_CH3];

  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);  //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  pid_roll_setpoint = 0;
  pid_pitch_setpoint = 0;
  pid_yaw_setpoint = 0;
  if(rc_values[RC_CH1] > 1508) {
    pid_roll_setpoint = rc_values[RC_CH1] - 1508;
  }
  else if (rc_values[RC_CH1] < 1492) {
    pid_roll_setpoint = rc_values[RC_CH1] - 1492;
  }

  pid_roll_setpoint = pid_roll_setpoint / 3;

  if(rc_values[RC_CH2] > 1508) pid_pitch_setpoint = rc_values[RC_CH2] - 1508;
  else if (rc_values[RC_CH2] < 1492) pid_pitch_setpoint = rc_values[RC_CH2] - 1492;

  pid_pitch_setpoint /= 3;

    if(rc_values[RC_CH4] > 1508) pid_yaw_setpoint = rc_values[RC_CH4] - 1508;
  else if (rc_values[RC_CH4] < 1492) pid_yaw_setpoint = rc_values[RC_CH4] - 1492;

  pid_yaw_setpoint /= 3;

  calculate_pid();

   battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  //Turn on the led if battery voltage is to low.
  while(battery_voltage < 1000 && battery_voltage > 600)
  {
    digitalWrite(12, HIGH);
  }

  if(rc_values[RC_CH3] < 1050 && rc_values[RC_CH4] < 1050)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && rc_values[RC_CH3] < 1050 && rc_values[RC_CH4] > 1450){
    start = 2;
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
   if(start == 2 && rc_values[RC_CH3] < 1050 && rc_values[RC_CH4] > 1950)start = 0;

  if(start == 2){
    digitalWrite(12, HIGH);
  esc4_output = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;
  esc5_output = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;
  esc6_output = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;
  esc7_output = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;

 /* if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc4_output += esc4_output * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc5_output += esc5_output * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc6_output += esc6_output * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc7_output += esc7_output * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } */
    
  esc4_output = map(esc4_output, 1000, 2000, 90, 180);
  esc5_output = map(esc5_output, 1000, 2000, 90, 180);
  esc6_output = map(esc6_output, 1000, 2000, 90, 180);
  esc7_output = map(esc7_output, 1000, 2000, 90, 180);

  if (esc4_output < 90) esc4_output = 90;
  if (esc5_output < 90) esc5_output = 90;
  if (esc6_output < 90) esc6_output = 90;
  if (esc7_output < 90) esc7_output = 90;
  if (esc4_output > 170) esc4_output = 170;
  if (esc5_output > 170) esc5_output = 170;
  if (esc6_output > 170) esc6_output = 170;
  if (esc7_output > 170) esc7_output = 170;
  }
  else{
    digitalWrite(12, LOW);
    esc4_output = 90;
    esc5_output = 90;
    esc6_output = 90;
    esc7_output = 90;
  }
    esc4.write(esc4_output);
    esc5.write(esc5_output);
    esc6.write(esc6_output);
    esc7.write(esc7_output);
    
    /*Serial.print(esc4_output);
    Serial.print(" ");
    Serial.print(esc5_output);
    Serial.print(" ");
    Serial.print(esc6_output);
    Serial.print(" ");
    Serial.println(esc7_output);*/
    //Serial.println(start);
    Serial.print(gyro_roll_input);
    Serial.print("    ");
    Serial.print(gyro_pitch_input);
    Serial.print("    ");
    Serial.println(gyro_yaw_input);
}
