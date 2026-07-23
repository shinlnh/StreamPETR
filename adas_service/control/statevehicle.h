#ifndef STATEVEHICLE_H
#define STATEVEHICLE_H

#include <iostream>
#include <cmath>
extern "C"{
#include <math.h>
}


using namespace std;

class StateVehicle
{
public:
    StateVehicle();
    StateVehicle(float xNew, float yNew, float yawNew, float vNew);
    ~StateVehicle();

    void SaturationAngle(float &angle, float angleMin, float angleMax);
    void NormalizeAngle(float &angle);
    void UpdateState(float delta, float acceleration);

    void SetState(int idx, float value);
    float GetState(int idx);
    float GetCarLength();

private:
    float pi = 3.1415926f;
    float dt = 0.1f;
    float L = 0.29f;
    float deltaMax = 60.0f*pi/180.0f;

    float x;   // unit m
    float y;   // unit m
    float yaw; // unit rad
    float v;   // unit m/s
};

#endif // STATEVEHICLE_H
