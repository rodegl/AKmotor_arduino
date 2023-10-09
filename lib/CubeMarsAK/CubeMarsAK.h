#ifndef __CubeMarsAK__
#define __CubeMarsAK__

#include <Arduino.h>


/*
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 10.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -8.5f
#define T_MAX 8.5f */

class CubeMarsAK
{
    public:
    CubeMarsAK();
    ~CubeMarsAK();
    void setup(uint8_t motor_id, float pMax, float vMax, float kpMax, float kdMax, float tMax);
    void EnterMotorMode();
    void ExitMotorMode();
    void Zero();
    unsigned int float_to_uint(float x, float x_min, float x_max, float bits);
    float uint_to_float(unsigned int x_int, float x_min,float x_max, int bits);
    void unpack_reply();
    void pack_cmd();

    void setDuty(uint8_t controller_id, float duty);
    void setCurrent(uint8_t controller_id, float current);
    void setBrakeCurrent(uint8_t controller_id, float current);
    void setVel(uint8_t controller_id, float rpm);
    void setPosition(uint8_t controller_id, float pos);
    void setOrigin(uint8_t controller_id, uint8_t set_origin_mode);
    void setPosVel(uint8_t controller_id, float pos,int16_t spd, int16_t RPA);
    void setServo();
    void servoMode(float position, float velocity, float torque);


    void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);
    void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
    void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index);

    void unpackServo();
    float p_in;
    float v_in;
    float kp_in;
    float kd_in;
    float t_in;

    float p_out;  // actual position
    float v_out;  // actual velocity
    float t_out;  // actual torque
    float temp_out;  // actual temperature
    float error_out;  // motor error mode

    private:

    uint8_t _motor_id;
    //uint32_t controller_id;

    float P_MIN;
    float P_MAX;
    float V_MIN;
    float V_MAX;
    float KP_MIN;
    float KP_MAX;
    float KD_MIN;
    float KD_MAX;
    float T_MIN;
    float T_MAX;

    protected:

};

#endif // __CubeMarsAK__