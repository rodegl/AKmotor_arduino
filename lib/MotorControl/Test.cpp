#include <Arduino.h>
#include <CAN.h>
#include <SPI.h>
#include <math.h>
#include <iostream>
#include <Print.h>
#include <LibPrintf.h>
#include <vector>

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
float p_in = 0.0f;
float v_in = 0.0f;
float kp_in = 0.0f;
float kd_in = 0.0f;
float t_in = 0.0f;
float enable = 0;
float zero = 0;
float sine = 0;
float step = 0.01;
float i = 0;



// Measured values - responses from the motor
float p_out = 0.0f;  // actual position
float v_out = 0.0f;  // actual velocity
float t_out = 0.0f;  // actual torque

long previousMillis =0;
boolean newData = false;

String str = "";
const char separator = ',';
const int dataLength = 8;
double data[dataLength];

long prev_Time, prev_Time2, current_time, current_time2;
float cycle_time, cycle_time2;
float sample_time, uart_time;


//new controllers




void EnterMotorMode(){
  
  byte buf[8];
  buf[0]=0xFF;
  buf[1]=0xFF;
  buf[2]=0xFF;
  buf[3]=0xFF;
  buf[4]=0xFF;
  buf[5]=0xFF;
  buf[6]=0xFF;
  buf[7]=0xFC;
  CAN.beginPacket(0x01);
  CAN.write(buf,8);
  CAN.endPacket();
 
}

void ExitMotorMode(){
  
  byte buf[8];
  buf[0]=0xFF;
  buf[1]=0xFF;
  buf[2]=0xFF;
  buf[3]=0xFF;
  buf[4]=0xFF;
  buf[5]=0xFF;
  buf[6]=0xFF;
  buf[7]=0xFD;
  CAN.beginPacket(0x01);
  CAN.write(buf,8);
  CAN.endPacket();
 
}

void Zero(){
  
  byte buf[8];
  buf[0]=0xFF;
  buf[1]=0xFF;
  buf[2]=0xFF;
  buf[3]=0xFF;
  buf[4]=0xFF;
  buf[5]=0xFF;
  buf[6]=0xFF;
  buf[7]=0xFE;
  CAN.beginPacket(0x01);
  CAN.write(buf,8);
  CAN.endPacket();
 
}

unsigned int float_to_uint(float x, float x_min, float x_max, float bits){
  
  //convert a float to an unsigned int, given range and number of bits  ///
  float span = x_max-x_min;
  float offset = x_min;
  unsigned int pgg=0;

  if(bits ==12){
    pgg= (unsigned int)((x-offset)*4095.0/span);
  }

  if(bits ==16){
    pgg= (unsigned int)((x-offset)*65535.0/span);
  }

  return pgg;
  
 }

float uint_to_float(unsigned int x_int, float x_min,float x_max, int bits){
  
  /// converts unsined int to float, given range and numbers of bits///
  float span = x_max - x_min;
  float offset = x_min;
  float pgg=0;

  if (bits ==12){
    pgg=((float)x_int)*span/4095.0 + offset;    
  }  

  if (bits ==16){
    pgg=((float)x_int)*span/65535.0 + offset;  
  }

  return pgg;

}

void unpack_reply(){

  //byte len=0;
  byte buf[8];
  //CAN.readMsgBuf(&len,buf);
  //unsigned long canId=CAN.getCanId();

  ///unpack ints from can buffer///

  buf[0]= CAN.read();
  buf[1]= CAN.read();
  buf[2]= CAN.read();
  buf[3]= CAN.read();
  buf[4]= CAN.read();
  buf[5]= CAN.read();

  unsigned int id = buf[0];
  unsigned int p_int = (buf[1]<<8)| buf[2];
  unsigned int v_int =(buf[3]<<4)|(buf[4]>>4);
  unsigned int i_int = ((buf[4]&0xF )<<8)| buf[5];

  /// convert uints to floats ///

  p_out = uint_to_float(p_int,P_MIN,P_MAX, 16);
  v_out = uint_to_float(v_int,V_MIN,V_MAX, 12);
  t_out = uint_to_float(i_int,-T_MAX,T_MAX, 12);

}

void pack_cmd(){
  
  byte buf[8];

  // limit data to  be within bounds ///

  float p_des= constrain(p_in,P_MIN,P_MAX);
  float v_des= constrain(v_in,V_MIN,V_MAX);
  float kp= constrain(kp_in,KP_MIN,KP_MAX);
  float kd= constrain(kd_in,KD_MIN,KD_MAX);
  float t_ff= constrain(t_in,T_MIN,T_MAX);

  unsigned int p_int = float_to_uint(p_des,P_MIN,P_MAX,16);
  unsigned int v_int = float_to_uint(v_des,V_MIN,V_MAX,12);
  unsigned int kp_int = float_to_uint(kp,KP_MIN,KP_MAX,12);
  unsigned int kd_int = float_to_uint(kd,KD_MIN,KD_MAX,12);
  unsigned int t_int = float_to_uint(t_ff,T_MIN,T_MAX,12);

  //pack ints into can buffer ///

  buf[0]=p_int >>8;
  buf[1]=p_int & 0xFF;
  buf[2]=v_int >> 4;
  buf[3]=((v_int&0xF)<< 4)| (kp_int >>8);
  buf[4]= kp_int &0xFF;
  buf[5]= kd_int >>4;
  buf[6]= ((kd_int&0xF)<< 4)| (t_int >>8);
  buf[7]= t_int &0xFF;
  CAN.beginPacket(0x01);
  CAN.write(buf,8);
  CAN.endPacket(); 
 
}

///////////

typedef enum {    
    CAN_PACKET_SET_DUTY = 0,        // Duty cycle mode
    CAN_PACKET_SET_CURRENT,         // Current loop mode
    CAN_PACKET_SET_CURRENT_BRAKE,   // Current brake mode
    CAN_PACKET_SET_RPM,             // Velocity mode
    CAN_PACKET_SET_POS,             // Position mode
    CAN_PACKET_SET_ORIGIN_HERE,     // Set origin mode
    CAN_PACKET_SET_POS_SPD,         // Position velocity loop mode     
} CAN_PACKET_ID;

void setDuty(uint8_t motor_id, uint32_t controller_id, float duty) {
  

  uint32_t can_id = motor_id |(controller_id<< 8);
  uint8_t buffer[4];
  //int32_t number = duty * 100000.0;
  int32_t number = duty * 1000.0;
  //int32_t number = duty;
  buffer[0] = number >> 24;
  buffer[1] = number >> 16;
  buffer[2] = number >> 8;
  buffer[3] = number;
  CAN.beginExtendedPacket(can_id, 4, true);
  CAN.write(buffer[0]);
  CAN.write(buffer[1]);
  CAN.write(buffer[2]);
  CAN.write(buffer[3]);  
  CAN.endPacket();
}

////////

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
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
  //sample_time = 0.005;
  uart_time = 0.01;
  

}


void loop() {

  //prev_Time = millis();
  char rc;
  while (Serial.available() > 0 && newData == false) {
      
        rc = Serial.read();
        Serial.println(rc);
        
        if(rc=='s'){
          Serial.println("True");
        }
    

  if (rc=='o') {
    EnterMotorMode();
  }
    
  if (rc=='f') {
    ExitMotorMode();
  } 

  }

  /*
  if (Serial.available())
  {
    str = Serial.readStringUntil('\n');

    for (int i = 0; i < dataLength; i++)
    {
      int index = str.indexOf(separator);
      data[i] = str.substring(0, index).toFloat();
      str = str.substring(index + 1);
    }
    p_in = data[0];
    v_in = data[1];
    kp_in = data[2];
    kd_in = data[3];
    t_in = data[4];
    //t_in= constrain(t_in,T_MIN, T_MAX);
    enable = data[5];
    zero = data[6];
    sine = data[7];

    if(enable == 1) {   // ENABLE MOTOR
      EnterMotorMode();
    }

    if(enable == 0) {   //DISABLE MOTOR
      //ExitMotorMode();
    } 

    if(zero == 1) {   //DISABLE MOTOR
      //Zero();
      //p_in = 0;
    }
    
  }


  if (sine  == 1) {
    if ( (millis() - prev_Time) > (0.001 * 1000)) {
      prev_Time = millis();
      i = i + 0.01;
      p_in = 1.5 *sin(i);
    }
  }
  */

  

  // send CAN   
  //pack_cmd(); 
  //EnterMotorMode();
  setDuty(1,1, 0.5);
  

  // receive CAN
  if(int packetSize = CAN.parsePacket()) {// check if data coming
    unpack_reply();
  }

  

  //current_time2 = millis();

  printf("%.2f, %.2f, %.2f, %.2f, %.2f \n", p_out, v_out, t_out, p_in, sine);


  
  /*
  Serial.print(motor_id);
  Serial.print(" , ");
  Serial.print(controller_id);
  Serial.print(" , ");
  Serial.println(can_id); */


  /*
  if (current_time2 - prev_Time2 > (uart_time * 1000)) // verifica que hayan pasado 10ms
  {
    //float diff = current_time2 - prev_Time2;
    prev_Time2 = current_time2;
    // SERIAL PORT WRITE

    //printf("%.2f, %.2f, %.2f, %.2f \n", p_out, v_out, t_out, p_in);
  } */

  //while ((millis()-prev_Time) < (sample_time * 1000)){ }

  //cycle_time = (millis() - prev_Time);
  //prev_Time = millis();
  
}