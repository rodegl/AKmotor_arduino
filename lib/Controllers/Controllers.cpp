#include "Controllers.h"

Controllers::Controllers()
{
}

Controllers::~Controllers()
{
}

void Controllers::setup(float P, float I, float D, float SP)
{
    gainP = P;
    gainD = D;
    gainI = I;
    SetPoint = SP;
    error = 0;
    sumIntegral = 0;
    _lastActual = 0;
}

void Controllers::setupSP(float gaink1, float gaink2)
{
    k1 = -gaink1;
    k2 = -gaink2;
    delta1 = 0;
    delta2 = 0;
    delta1_prev = 0;
    delta2_prev = 0;
}

float Controllers::getOutput(float ActualValue, float T_time)
{
    _T_time = T_time / 1E6;

    error = SetPoint - ActualValue;

    // P term
    _Poutput = gainP * error;

    // I term
    sumIntegral += error * _T_time;
    _Ioutput = gainI * sumIntegral;

    // D term
    derivateE = (error - _lastActual) / _T_time;
    _Doutput = 0;
    //_Doutput = gainD * vel;
    _lastActual = error;    
 
    // output
    _output = _Poutput + _Doutput;

    return _output;
}

float Controllers::getOutputSP(float ActualValue, float T_time, float vel)
{
    _T_time = T_time / 1E6;

    error = SetPoint - ActualValue;
    errorST =  error - delta1_prev;
    
    // Estimacion de posición y velocidad

    delta1 = delta1_prev + _T_time * (delta2_prev + k1 * (sqrt(abs(errorST)) * sign(errorST)));
    delta2 = delta2_prev + _T_time * (k2*sign(errorST));

    float e = SetPoint - ActualValue;
    // P term
    _Poutput = gainP * e;

    // I term
    sumIntegral += error * _T_time;
    _Ioutput = gainI * sumIntegral;

    // D term
    derivateE = (e - _lastActual) / _T_time;
    negDelta2 = -delta2;
    _Doutput = gainD * delta2;
    //_Doutput = gainD * ((error - _lastActual) / _T_time);

    _lastActual = e;
    delta1_prev = delta1;
    delta2_prev = delta2;

    // output
    _output = _Poutput + _Doutput;

    return _output;
}

float Controllers::superTwisting(float ActualValue, float T_time, float gainU, float gainE)
{
    _T_time = T_time;

    error = ActualValue - SetPoint;
    errorST = error - delta1_prev;
    
    // Estimacion de posición y velocidad
    delta1 = delta1_prev + _T_time * (delta2_prev + k1 * (sqrt(abs(errorST)) * sign(errorST)));
    delta2 = delta2_prev + _T_time * (k2*sign(errorST));

    // output
    //sigma =  delta2 + gainE*error; 
    sigma =  ((error - _lastActual) / _T_time) + gainE*error; 
    w = w - _T_time*(1.1*gainU)*sign(sigma);
    _output = - sqrt(gainU)*sqrt(abs(sigma))*sign(sigma) + w;        //Super Twisting

    _lastActual = error;
    delta1_prev = delta1;
    delta2_prev = delta2;

    return _output;
}

float Controllers::slidingModes(float ActualValue, float T_time, float gain1, float gain2)
{
    _T_time = T_time;
    error = SetPoint - ActualValue;

    float error_dev = (error - _lastActual) / _T_time;
    sigma = error_dev + gain2 * error;

    _output = -gain1  + sign(sigma);

    return _output;
}

float Controllers::sign(float value)
{
    float signo;
    if (value > 0) {
        signo = 1;
    } else if (value < 0) {
        signo = -1;
    } else {
        signo = 0;
    }

    return signo;
}
