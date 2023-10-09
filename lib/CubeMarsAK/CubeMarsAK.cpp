#include "CubeMarsAK.h"

#include "CAN.h"
#include "Arduino.h"

typedef enum {    
    CAN_PACKET_SET_DUTY = 0,        // Duty cycle mode
    CAN_PACKET_SET_CURRENT,         // Current loop mode
    CAN_PACKET_SET_CURRENT_BRAKE,   // Current brake mode
    CAN_PACKET_SET_RPM,             // Velocity mode
    CAN_PACKET_SET_POS,             // Position mode
    CAN_PACKET_SET_ORIGIN_HERE,     // Set origin mode
    CAN_PACKET_SET_POS_SPD,         // Position velocity loop mode     
} CAN_PACKET_ID;

CubeMarsAK::CubeMarsAK()
{
}

CubeMarsAK::~CubeMarsAK()
{
}

void CubeMarsAK::setup(uint8_t CAN_id, float pMax, float vMax, float kpMax, float kdMax, float tMax)
{
    _motor_id = CAN_id;

    P_MIN = -pMax;
    P_MAX = pMax;
    V_MIN = -vMax;
    V_MAX = vMax;
    KP_MIN = 0;
    KP_MAX = kpMax;
    KD_MIN = 0;
    KD_MAX = kdMax;
    T_MIN = -tMax;
    T_MAX = tMax;

    p_in = 0;
    v_in = 0;
    kp_in = 0;
    kd_in = 0;
    t_in = 0;

    p_out = 0;  // actual position
    v_out = 0;  // actual velocity
    t_out = 0;  // actual torque    
    temp_out = 0;
    error_out = 0;
}

void CubeMarsAK::EnterMotorMode()
{
    byte buf[8];
    buf[0]=0xFF;
    buf[1]=0xFF;
    buf[2]=0xFF;
    buf[3]=0xFF;
    buf[4]=0xFF;
    buf[5]=0xFF;
    buf[6]=0xFF;
    buf[7]=0xFC;
    CAN.beginPacket(_motor_id);
    CAN.write(buf,8);
    CAN.endPacket();
}

void CubeMarsAK::ExitMotorMode()
{
    byte buf[8];
    buf[0]=0xFF;
    buf[1]=0xFF;
    buf[2]=0xFF;
    buf[3]=0xFF;
    buf[4]=0xFF;
    buf[5]=0xFF;
    buf[6]=0xFF;
    buf[7]=0xFD;
    CAN.beginPacket(_motor_id);
    CAN.write(buf,8);
    CAN.endPacket();
}

void CubeMarsAK::Zero()
{
    byte buf[8];
    buf[0]=0xFF;
    buf[1]=0xFF;
    buf[2]=0xFF;
    buf[3]=0xFF;
    buf[4]=0xFF;
    buf[5]=0xFF;
    buf[6]=0xFF;
    buf[7]=0xFE;
    CAN.beginPacket(_motor_id);
    CAN.write(buf,8);
    CAN.endPacket();
}

unsigned int CubeMarsAK::float_to_uint(float x, float x_min, float x_max, float bits)
{
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

float CubeMarsAK::uint_to_float(unsigned int x_int, float x_min,float x_max, int bits)
{
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

void CubeMarsAK::unpack_reply()
{
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

void CubeMarsAK::pack_cmd()
{
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
    CAN.beginPacket(_motor_id);
    CAN.write(buf,8);
    CAN.endPacket();
}









void CubeMarsAK::comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len)
{
    uint8_t i=0;

    if (len > 8) {
        len = 8;
    }
    CAN.beginExtendedPacket(id, len, true);

    for(i=0;i<len;i++) CAN.write(data[i]);

    CAN.write(data,len);
    CAN.endPacket(); 
}

void CubeMarsAK::buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void CubeMarsAK::buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}
// DUTY CYCLE MODE - #0
void CubeMarsAK::setDuty(uint8_t controller_id, float duty)
{

    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
    comm_can_transmit_eid(controller_id |((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);

    // int32_t number = duty * 100000.0;
    //sendCan(0, number);
}
// CURRENT LOOP MODE - #1
// The current value is of int32 type, and the value (-60000, 60000) represents -60-60A.
void CubeMarsAK::setCurrent(uint8_t controller_id, float current)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
    //int32_t number = current * 1000.0;
    //sendCan(1, number);
}
// CURRENT BRAKE MODE - #2
// The braking current value is of int32 type, and the value (0, 60000) represents 0-60A.
void CubeMarsAK::setBrakeCurrent(uint8_t controller_id, float current)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}
// VELOCITY MODE - #3
// the speed value is int32 type, and the range (-100000, 100000) represents (-100000, 100000) electrical speed.
void CubeMarsAK::setVel(uint8_t controller_id, float rpm)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)rpm, &send_index);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}
// POSITION LOOP MODE - #4
// Position as int32 type，range (-360000000, 360000000) represents position (-36000°,36000°)
void CubeMarsAK::setPosition(uint8_t controller_id, float pos)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}
// SET ORIGIN MODE - #5
// The setting command is uint8_t type, 0 means setting the temporary origin (power failure elimination)
// 1 means setting the permanent zero point (automatic parameter saving)
// 2 means restoring the default zero point (automatic parameter saving)
void CubeMarsAK::setOrigin(uint8_t controller_id, uint8_t set_origin_mode)
{
    int32_t send_index = 0;
    uint8_t buffer[1];
    buffer[0] = set_origin_mode;
    comm_can_transmit_eid(controller_id | ((uint32_t) CAN_PACKET_SET_ORIGIN_HERE << 8), buffer, send_index);
}

void CubeMarsAK::setPosVel(uint8_t controller_id, float pos,int16_t spd, int16_t RPA)
{
    int32_t send_index = 0;
    int16_t send_index1 = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
    buffer_append_int16(buffer,spd, & send_index1);
    buffer_append_int16(buffer,RPA, & send_index1);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index);
}

void CubeMarsAK::servoMode(float position, float velocity, float torque)
{
    byte buf[8];

    // limit data to  be within bounds ///

    float p_des= constrain(position,P_MIN,P_MAX);
    float v_des= constrain(velocity,V_MIN,V_MAX);
    float t_ff= constrain(torque,T_MIN,T_MAX);

    unsigned int p_int = float_to_uint(p_des,P_MIN,P_MAX,16);
    unsigned int v_int = float_to_uint(v_des,V_MIN,V_MAX,16);
    unsigned int t_int = float_to_uint(t_ff,T_MIN,T_MAX,16);

    //pack ints into can buffer ///

    buf[0]= p_int >>8;
    buf[1]= p_int & 0xFF;
    buf[2]= v_int >> 8;
    buf[3]= v_int & 0xFF;
    buf[4]= t_int >> 8;
    buf[5]= t_int & 0xFF;
    buf[6]= 0;
    buf[7]= 0;
    CAN.beginPacket(_motor_id);
    CAN.write(buf,8);
    CAN.endPacket();


}

void CubeMarsAK::unpackServo()
{
    byte buf[8];

    buf[0]= CAN.read();
    buf[1]= CAN.read();
    buf[2]= CAN.read();
    buf[3]= CAN.read();
    buf[4]= CAN.read();
    buf[5]= CAN.read();
    buf[6]= CAN.read();
    buf[7]= CAN.read();


    int16_t pos_int = buf[0] << 8 | buf[1];
    int16_t spd_int = buf[2] << 8 | buf[3];
    int16_t cur_int = buf[4] << 8 | buf[5];

    p_out = (float)( pos_int * 0.1f); //motor position
    v_out = (float)( spd_int * 10.0f);//motor speed
    t_out = (float)( cur_int * 0.01f);//motor current

    temp_out = buf[6];//motor temperature
    error_out = buf[7];//motor error mode
}