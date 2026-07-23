#include "statevehicle.h"

StateVehicle::StateVehicle()
{

}

StateVehicle::StateVehicle(float xNew, float yNew, float yawNew, float vNew)
{
    x = xNew;
    y = yNew;
    yaw = yawNew;
    v = vNew;
}

StateVehicle::~StateVehicle()
{

}

void StateVehicle::SaturationAngle(float &angle, float angleMin, float angleMax)
{
    if     (angle < angleMin) angle = angleMin;
    else if(angle > angleMax) angle = angleMax;
}

void StateVehicle::NormalizeAngle(float &angle)
{
    while(angle > pi) angle -= 2.0f*pi;
    while(angle < pi) angle += 2.0f*pi;
}

void StateVehicle::UpdateState(float delta, float acceleration)
{
    SaturationAngle(delta, -deltaMax, deltaMax);

    x += v*cos(yaw)*dt;
    y += v*sin(yaw)*dt;
    yaw += v/L*tan(delta)*dt; NormalizeAngle(yaw);
    v += acceleration*dt;
}

void StateVehicle::SetState(int idx, float value)
{
    if     (idx == 0) x = value;
    else if(idx == 1) y = value;
    else if(idx == 2) yaw = value;
    else if(idx == 3) v = value;   
}

float StateVehicle::GetState(int idx)
{
    if     (idx == 0) return x;
    else if(idx == 1) return y;
    else if(idx == 2) return yaw;
    else if(idx == 3) return v;
    else              return nanf(""); // not a number in float
}

float StateVehicle::GetCarLength()
{
    return L;
}
