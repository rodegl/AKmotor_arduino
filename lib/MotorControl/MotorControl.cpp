//Control mode contain {0,1,2,3,4,5,6,7} Seven eigenvalues correspond to seven control modes respectively
//Duty cycle mode: 0
//Current loop mode: 1
//Current brake mode: 2
//Velocity mode: 3
//Position mode: 4
//Set origin mode:5
//Position velocity loop mode: 6
//Examples of various mode control motors are provided below
//The following are library functions and macro definitions for each instance

#include <Arduino.h>
#include <CAN.h>
#include <SPI.h>
#include <iostream>

typedef enum {    
    CAN_PACKET_SET_DUTY = 0,        // Duty cycle mode
    CAN_PACKET_SET_CURRENT,         // Current loop mode
    CAN_PACKET_SET_CURRENT_BRAKE,   // Current brake mode
    CAN_PACKET_SET_RPM,             // Velocity mode
    CAN_PACKET_SET_POS,             // Position mode
    CAN_PACKET_SET_ORIGIN_HERE,     // Set origin mode
    CAN_PACKET_SET_POS_SPD,         // Position velocity loop mode     
} CAN_PACKET_ID;

void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {

    uint8_t i=0;

    if (len > 8) {
        len = 8;
    }
    /*
    CanTxMsg TxMessage;
    TxMessage.StdId = 0;
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.ExtId = id;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = len; //memcpy(txmsg.data8, data, len);
    */
    CAN.beginExtendedPacket(id, len, true);

    //for(i=0;i<len;i++) TxMessage.Data[i]=data[i];

    // CAN_Transmit(CHASSIS_CAN, &TxMessage);

    CAN.write(data,len);
    CAN.endPacket(); 
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

// DUTY CYCLE MODE - #0
void comm_can_set_duty(uint8_t controller_id, float duty) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
    comm_can_transmit_eid(controller_id |((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

// CURRENT LOOP MODE - #1
// The current value is of int32 type, and the value (-60000, 60000) represents -60-60A.
int comm_can_set_current(uint8_t controller_id, float current) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
    
}

// CURRENT BRAKE MODE - #2
// The braking current value is of int32 type, and the value (0, 60000) represents 0-60A.
void comm_can_set_cb(uint8_t controller_id, float current) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

// VELOCITY MODE - #3
// the speed value is int32 type, and the range (-100000, 100000) represents (-100000, 100000) electrical speed.
void comm_can_set_rpm(uint8_t controller_id, float rpm) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)rpm, &send_index);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

// POSITION LOOP MODE - #4
// Position as int32 type，range (-360000000, 360000000) represents position (-36000°,36000°)
void comm_can_set_pos(uint8_t controller_id, float pos) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

// SET ORIGIN MODE - #5
// The setting command is uint8_t type, 0 means setting the temporary origin (power failure elimination)
// 1 means setting the permanent zero point (automatic parameter saving)
// 2 means restoring the default zero point (automatic parameter saving)
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode) {
    int32_t send_index = 0;
    uint8_t buffer[1];
    buffer[0] = set_origin_mode;
    comm_can_transmit_eid(controller_id | ((uint32_t) CAN_PACKET_SET_ORIGIN_HERE << 8), buffer, send_index);
}

// POSITION AND VELOCITY LOOP MODE - #6
// The position is int32 type, and the range (-360000000, 360000000) represents the position (-36000°, -36000°)
// The speed is int16 type, and the range (-32768, 32767) represents (-32768, 32767) electrical speed;
// The acceleration is int16 type, and the range (0, 200) represents (0, 400000) electrical speed/s². 
// 1 unit equals 20000 electrical speed /s².
void comm_can_set_pos_spd(uint8_t controller_id, float pos,int16_t spd, int16_t RPA ) {
    int32_t send_index = 0;
    int16_t send_index1 = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
    buffer_append_int16(buffer,spd, & send_index1);
    buffer_append_int16(buffer,RPA, & send_index1);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index);
}


// SERVO MODE
// In servo mode, motor packets are uploaded in timing mode. 
// The upload frequency can be set to 1-500Hz, and the upload byte is 8 bytes.
// The position is int16 type, and the range (-32000, 32000) represents the position (-3200°, 3200°).
// The speed is int16 type, and the range (-32000, 32000) represents (-320000, 320000) rpm electrical speed;
// The current is of type int16, and the value [-6000,6000] represents [-60,60] A. 
// Among them, the temperature is int8 type, and the range of [-20,127] represents the temperature of the driver board: [-20℃,127]℃;
// The error code is uint8 type, 0 means no fault, 1 means over temperature fault, 2 means over current fault.
// 3 means over voltage fault, 4 means under voltage fault, 5 means encoder fault.
// 6 means phase current unbalance fault (The hardware may be damaged).

/*

void motor_receive(float* motor_pos,float* motor_spd,float* motor_cur, int8_t* motor_temp,int8_t* motor_error, rx_message) {

    int16_t pos_int = (rx_message)->Data[0] << 8 | (rx_message)->Data[1];
    int16_t spd_int = (rx_message)->Data[2] << 8 | (rx_message)->Data[3];
    int16_t cur_int = (rx_message)->Data[4] << 8 | (rx_message)->Data[5];
    &motor_pos= (float)( pos_int * 0.1f); //motor position
    &motor_spd= (float)( spd_int * 10.0f);//motor speed
    &motor_cur= (float)( cur_int * 0.01f);//motor current

    &motor_temp= (rx_message)->Data[6] ;//motor temperature
    &motor_error= (rx_message)->Data[7] ;//motor error mode
}
*/