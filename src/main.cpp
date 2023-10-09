#include <Arduino.h>
#include <CAN.h>
#include <SPI.h>
#include <math.h>
#include <iostream>
#include <Print.h>
#include <LibPrintf.h>
#include <vector>
#include <CubeMarsAK.h>
#include <Controllers.h>

CubeMarsAK Roll;
CubeMarsAK Pitch;
Controllers ControlRoll;
Controllers ControlPitch;

// Value Limits
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 10.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -8.5f
#define T_MAX 8.5f

// Set Values

float enable = 0;
float zero = 0;
float sine = 0;
float step = 0.01;
float i = 0;

float gains = 0;
float vel_sp = 0;
float pos_prev = 0;


long previousMillis =0;
boolean newData = false;

String str = "";
const char separator = ',';
const int dataLength = 11;
double data[dataLength];

volatile unsigned prev_Time, prev_Time2, current_time, current_time2, prev_Time3;
volatile unsigned cycle_time, cycle_time2, test_time;
float sample_time, uart_time;

float test;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  
  while (!Serial);

  Serial.println("CAN Receiver");

  // start the CAN bus at 1000 kbps
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  } 

  prev_Time = 0;
  prev_Time2 = 0;
  cycle_time = 0;
  sample_time = 900;
  uart_time = 10000;
  
  Roll.setup(0x01, 12.5, 20, 10, 5, 4); // CAN, pos, vel, kp, kd, torque (LIMITS)
  Pitch.setup(0x02, 12.5, 20, 10, 5, 4); // CAN, pos, vel, kp, kd, torque (LIMITS)

  ControlRoll.setup(0, 0, 0, 0);
  ControlRoll.setupSP(0, 0);

  ControlPitch.setup(0, 0, 0, 0);
  ControlPitch.setupSP(0, 0);

}

////////////////////////////////////////////////

void loop() {
  

  
  //////////////////////////////
  current_time = micros();
  cycle_time = current_time - prev_Time;
  if (cycle_time >= sample_time) {
    prev_Time = current_time;
    //// -------------------

    if (Serial.available()) {
    str = Serial.readStringUntil('\n');

    for (int i = 0; i < dataLength; i++)
    {
      int index = str.indexOf(separator);
      data[i] = str.substring(0, index).toFloat();
      str = str.substring(index + 1);
    }
    /*
    Roll.p_in = data[0];
    Roll.v_in = data[1];
    Roll.kp_in = data[2];
    Roll.kd_in = data[3];
    Roll.t_in = data[4]; */

    ControlRoll.SetPoint = data[0];
    ControlRoll.gainP =  data[1];
    ControlRoll.gainI = data[2];
    ControlRoll.gainD = data[3];

    //t_in= constrain(t_in,T_MIN, T_MAX);
    enable = data[4];
    zero = data[5];
    sine = data[6];

    sample_time = data[7];

    ControlRoll.k1 = data[8];
    ControlRoll.k2 = data[9];

    ControlPitch.SetPoint = data[10];

    if(enable == 1) {   // ENABLE MOTOR
      Roll.EnterMotorMode();
      Pitch.EnterMotorMode();
    }

    if(enable == 0) {   //DISABLE MOTOR
      Roll.ExitMotorMode();
      Pitch.ExitMotorMode();
    } 

    if(zero == 1) {   //DISABLE MOTOR
      Roll.Zero();
      Roll.p_in = 0;
      Roll.t_in = 0;
      ControlRoll.delta1 = 0;
      ControlRoll.delta2 = 0;

      Pitch.Zero();
      Pitch.p_in = 0;
      Pitch.t_in = 0;
      ControlRoll.delta1 = 0;
      ControlRoll.delta2 = 0;
    }
    
  }
    //// -------------------
    vel_sp = (Roll.p_out - pos_prev) / (sample_time / 1E6);
    pos_prev = Roll.p_out;
    
    // receive CAN
    if(CAN.parsePacket()) {   // check if data coming
      Roll.unpack_reply();
    } 
    if(CAN.parsePacket()) {   // check if data coming
      Pitch.unpack_reply();
    }
    

    if (sine  == 1) {
    if ( (millis() - prev_Time3) > (0.001 * 1000)) {
      prev_Time3 = millis();
      i = i + 0.01;
      ControlRoll.SetPoint = 1.5 *sin(i);
      ControlPitch.SetPoint = 1.5 *cos(i);
      }
    }  
    Roll.t_in = ControlRoll.getOutputSP(Roll.p_out, uart_time, -Roll.v_out*3); //PID control

    

    //Roll.t_in = ControlRoll.getOutput(Roll.p_out, sample_time); //PID control
    
    //Roll.t_in = ControlRoll.superTwisting(Roll.p_out, sample_time, ControlRoll.gainP, ControlRoll.gainD);

    if (abs(Roll.p_out) > 2.5)
    {
      Roll.ExitMotorMode();
      Pitch.ExitMotorMode();
    }

    // send CAN  
    Roll.pack_cmd();
    Pitch.pack_cmd();

  }   
    
  //////////////////////////////////
  current_time2 = micros();
  cycle_time2 = current_time2 - prev_Time2;
  if (cycle_time2 >= uart_time) {
    
    //Serial.print(test_time);
    //Serial.print(",");
    //Serial.println(cycle_time2);
    //printf(" %i, %i \n", test_time, cycle_time2);
    printf("%.3f, %.3f, %.3f, %.3f, ", Roll.p_out, -Roll.v_out*3, Roll.t_out, ControlRoll.SetPoint);
    printf("%.3f, %.3f, %.3f, %.3f, ", ControlRoll.derivateE, ControlRoll.error, ControlRoll.delta1, ControlRoll.delta2);
    printf("%.3f, %.3f, %.3f, %.3f, ", Pitch.p_out, -Pitch.v_out*3, Pitch.t_out, ControlPitch.SetPoint);
    printf("%.3i, %.3i \n", cycle_time, cycle_time2);
    
    prev_Time2 = current_time2;
  }
  
  /*
  while ((micros() - prev_Time) < (sample_time)){ 
    
  } */

  
  
  
  
}