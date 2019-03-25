//
// Created by stdplus on 25.03.19.
//
#include "MotorController.h"

#include <rbc.h>
#include <future>

//конструкторы+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
MotorController::MotorController()
{
    port_L = motorC;
    port_R = motorA;
    Dr =190;
    Dw = 45.5;
    w = 2.3;
    pwrTurn = 10;
    sensoreOn = true;
}

MotorController::MotorController(tMotor portLeft, tMotor portRight, bool Sensore)
{
    port_L = portLeft;
    port_R = portRight;
    Dr =190;
    Dw = 45.5;
    w = 2.3;
    pwrTurn = 10;
    sensoreOn = Sensore;
}

MotorController::MotorController(tMotor Left, tMotor Right, float robotRadius, float Dw, float w, int pwrTurn, bool Sensore)
{
    port_L = Left;
    port_R = Right;
    Dr = robotRadius;
    this -> Dw = Dw;
    this -> w = w;
    this ->pwrTurn = pwrTurn;
    sensoreOn = Sensore;
}
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-

void MotorController::Move(int speed,float distance)
{
    float Dist = distance;
    resetMotorEncoder(port_L);
    resetMotorEncoder(port_R);
    setMotorSpeed(port_L, speed);
    setMotorSpeed(port_R, speed);

    while((getMotorEncoder(port_L) < Dist / (PI * Dw) * 360) || (getMotorEncoder(port_R) < Dist / (PI * Dw) * 360))
    {

        if((getMotorEncoder(port_L) > 0.9 * Dist / (PI * Dw) * 360) ||
           (getMotorEncoder(port_R) > 0.9 * Dist / (PI * Dw) * 360))
        {
            setMotorSpeed(port_L, 10);
            setMotorSpeed(port_R, 10);
        }
        delay(1);
    }
    setMotorSpeed(port_L,0);
    setMotorSpeed(port_R,0);
    //delay(1000);
}

void MotorController::Turn(float angle)
{
    resetMotorEncoder(port_L);
    setMotorSpeed(port_L, pwrTurn * sgn(angle));
    setMotorSpeed(port_R, pwrTurn * sgn(angle) * -1);

    while(fabs(getMotorEncoder(port_L)) < fabs(Dr / Dw * angle))
    {
        delay(1);
    }
    setMotorSpeed(port_L,0);
    setMotorSpeed(port_R,0);
    delay(1000);
}

void  MotorController::CircularMoveLeft(float radius, float distance_w)
{
    float distance_l = 2*PI*radius, distance_r = 2*PI*(radius + Dr);

    //Поток для инкодера левого двигателя
    auto j1 = std::async([this, distance_l, radius, distance_w]
                         {
                             resetMotorEncoder(port_L);
                             resetMotorEncoder(port_R);
                             setMotorSpeed(port_L, static_cast<int8_t>(this -> pwrCircularM / ((Dr + radius) / (radius))));

                             while(getMotorEncoder(port_L) < (distance_l * distance_w) / (PI * Dw) * 360 )
                             {
                                 if(getMotorEncoder(port_L) > 0.9 * (distance_l * distance_w) / (PI * Dw) * 360)
                                 {
                                     setMotorSpeed(port_L, static_cast<int8_t>(this->pwrCircularM / ((Dr + radius) / (radius))));
                                 }
                                 if ((getMotorEncoder(port_R) / (Dr + radius) / (radius))  > getMotorEncoder(port_L))
                                 {
                                     setMotorSpeed(port_L, static_cast<int8_t>(this->pwrCircularM / ((Dr + radius) / (radius))) + 1);
                                 }
                                 if ((getMotorEncoder(port_R) / (Dr + radius) / (radius))  < getMotorEncoder(port_L))
                                 {
                                     setMotorSpeed(port_L, static_cast<int8_t>(this->pwrCircularM / ((Dr + radius) / (radius))) - 1);
                                 }
                                 delay(1);
                             }
                             setMotorSpeed(port_L, 0);
                             this->pwrCircularM = 80;
                         });


    auto j2 = std::async([this, distance_r,radius, distance_w]
                         {
                             resetMotorEncoder(port_R);
                             resetMotorEncoder(port_L);
                             setMotorSpeed(port_R, this->pwrCircularM);

                             while(getMotorEncoder(port_R) < (distance_r * distance_w) / (PI * Dw) * 360 )
                             {
                                 if(getMotorEncoder(port_R) > 0.9 * (distance_r * distance_w) / (PI * Dw) * 360)
                                 {
                                     this->pwrCircularM = 30;
                                     setMotorSpeed(port_R, this->pwrCircularM);
                                 }
                                 delay(1);
                             }
                             setMotorSpeed(port_R, 0);
                         });
    // delay(1000);
}

void  MotorController::CircularMoveRight(float radius, float distance_w)
{
    float distance_l = 2*PI*(radius + Dr), distance_r = 2*PI*radius;

    //Поток для инкодера левого двигателя
    auto j1 = std::async([this, distance_r, radius, distance_w]
                         {
                             resetMotorEncoder(port_L);
                             resetMotorEncoder(port_R);
                             setMotorSpeed(port_R, static_cast<int8_t>(this -> pwrCircularM / ((Dr + radius) / (radius))));

                             while(getMotorEncoder(port_R) < (distance_r * distance_w) / (PI * Dw) * 360 )
                             {
                                 if(getMotorEncoder(port_R) > 0.9 * (distance_r * distance_w) / (PI * Dw) * 360)
                                 {
                                     setMotorSpeed(port_R, static_cast<int8_t>(this->pwrCircularM / ((Dr + radius) / (radius))));
                                 }
                                 if ((getMotorEncoder(port_L) / (Dr + radius) / (radius))  > getMotorEncoder(port_R))
                                 {
                                     setMotorSpeed(port_R, static_cast<int8_t>(this->pwrCircularM / ((Dr + radius) / (radius))) + 1);
                                 }
                                 if ((getMotorEncoder(port_L) / (Dr + radius) / (radius))  < getMotorEncoder(port_R))
                                 {
                                     setMotorSpeed(port_R, static_cast<int8_t>(this->pwrCircularM / ((Dr + radius) / (radius))) - 1);
                                 }
                                 delay(1);
                             }
                             setMotorSpeed(port_R, 0);
                             this->pwrCircularM = 80;
                         });


    auto j2 = std::async([this, distance_l,radius, distance_w]
                         {
                             resetMotorEncoder(port_L);
                             resetMotorEncoder(port_R);
                             setMotorSpeed(port_L, this->pwrCircularM);

                             while(getMotorEncoder(port_L) < (distance_l * distance_w) / (PI * Dw) * 360 )
                             {
                                 if(getMotorEncoder(port_L) > 0.9 * (distance_l * distance_w) / (PI * Dw) * 360)
                                 {
                                     this->pwrCircularM = 30;
                                     setMotorSpeed(port_L, this->pwrCircularM);
                                 }
                                 delay(1);
                             }
                             setMotorSpeed(port_L, 0);
                         });
    //delay(1000);
}

void MotorController::SmoothTurn(float radius, float angle)
{
    MotorController GoOn(port_L, port_R, Dr, Dw, w, pwrTurn, sensoreOn);
    float w = abs(((PI * radius * angle) / 180) / ((PI * radius * 360) / 180));
    if(angle > 0)
    {
        GoOn.CircularMoveRight(radius,w);
    }
    else
    {
        GoOn.CircularMoveLeft(radius,w);
    }
}
