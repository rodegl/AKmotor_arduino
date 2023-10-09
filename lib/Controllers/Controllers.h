#ifndef __Controllers_H__
#define __Controllers_H__

#include <Arduino.h>

class Controllers
{
    public: 
        Controllers();
        ~Controllers();
        void setup(float gainP, float I, float gainD, float SetPoint);
        void setupSP(float gaink1, float gaink2);
        float getOutput(float ActualValue, float T_time);
        float getOutputSP(float ActualValue, float T_time, float vel);

        float superTwisting(float ActualValue, float T_time, float gainU, float gainE);
        float slidingModes(float ActualValue, float T_time, float gain1, float gain2);

        float sign(float value);
        float read();
        float gainP;
        float gainI;
        float gainD;
        float SetPoint;
        float error;
        float sumIntegral;
        float derivateE;
        float negDelta2;

        //SUPER TWISTING
        //ganancias del algoritmo de Super Twisting
        float k1;
        float k2;
        float delta1; //posicion estimada
        float delta2; //velocidad estimada
        float delta1_prev;
        float delta2_prev;
        float errorST; //error estimado por Super Twisting
        //float sign; //signo del error estimado

        float gainU;
        float fainE;
        float sigma;
        float w;

    private: 
        bool _firstRun; 
        float _T_time;
        float _lastActual;
        float _output;
        float _Doutput;
        float _Ioutput;
        float _Poutput;

    protected: 
};

#endif // __Controllers_H__ 