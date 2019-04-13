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
    sensoreModeOn = true;
    senPort = S1;
}

MotorController::MotorController(tMotor portLeft, tMotor portRight, bool SensoreMode, tSensor sensorPort)
{
    port_L = portLeft;
    port_R = portRight;
    Dr =190;
    Dw = 45.5;
    w = 2.3;
    pwrTurn = 10;
    sensoreModeOn = SensoreMode;
    senPort = sensorPort;
}

MotorController::MotorController(tMotor Left, tMotor Right, float robotRadius, float Dw, float w, int pwrTurn, bool SensoreMode, tSensor sensorPort)
{
    port_L = Left;
    port_R = Right;
    Dr = robotRadius;
    this -> Dw = Dw;
    this -> w = w;
    this ->pwrTurn = pwrTurn;
    sensoreModeOn = SensoreMode;
    senPort = sensorPort;
}
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-

void MotorController::SensoreOn()
{
    if(sensoreModeOn == true)
    {
        SensoreStatus_ = true;
    }
    else
    {
        displayTextLine(3,"No Sensore detected");
        SensoreStatus_ = false;
        delay(5000);
    }
}

void MotorController::SensoreOff()
{
    if(sensoreModeOn == true)
    {
        SensoreStatus_ = false;
    }
    else
    {
        displayTextLine(3,"No Sensore detected");
        SensoreStatus_ = false;
        delay(5000);
    }

}

void MotorController::Targeting(float distance_to_object)
{
    float k, diff;
    if(sensoreModeOn == true) {
        while (getSensorValue(senPort) <= 1500) {
            if (getSensorValue(senPort) > distance_to_object)
            {
               diff = getSensorValue(senPort) - distance_to_object;
               k = diff/255;
               setMotorSpeed(port_L,1 + static_cast<int8_t>(k*100));
               setMotorSpeed(port_R,1 + static_cast<int8_t>(k*100));
               delay(1);
            }
            else if (getSensorValue(senPort) < distance_to_object)
            {
                diff = getSensorValue(senPort) - distance_to_object;
                k = diff/255;
                setMotorSpeed(port_L,-1 + static_cast<int8_t>(k*100));
                setMotorSpeed(port_R,-1 + static_cast<int8_t>(k*100));
                delay(1);
            }
            else if (getSensorValue(senPort) == distance_to_object)
            {
                setMotorSpeed(port_L, 0);
                setMotorSpeed(port_R, 0);
            }
        }
    }
    else
    {
        displayTextLine(3, "No Sensore Detected");
        delay(5000);
    }
}

void MotorController::Move(int speed,float distance, int delay_)
{
    auto j1  = std::async([this, distance]
                          {
                              float Dist = distance;
                              getMotorEncoder(port_L);
                              while ((getMotorEncoder(port_L) < Dist / (PI * Dw) * 360) || (getMotorEncoder(port_R) < Dist / (PI * Dw) * 360)) {
                                  if (SensoreStatus_ == true) {
                                      this->sensoreDistance = getSensorValue(this->senPort);
                                      if (sensoreDistance <= 55) {
                                          this->SensoreDangerDistance_ = true;
                                      }
                                      else
                                      {
                                          this -> SensoreDangerDistance_ = false;
                                      }
                                  }
                              }
                          });

    auto j2 = std::async([this, distance, speed, delay_] {
        float Dist = distance;
        resetMotorEncoder(port_L);
        resetMotorEncoder(port_R);

        while ((getMotorEncoder(port_L) < Dist / (PI * Dw) * 360) ||
               (getMotorEncoder(port_R) < Dist / (PI * Dw) * 360)) {
            if(this -> SensoreDangerDistance_ == false) {
                setMotorSpeed(port_L, speed);
                setMotorSpeed(port_R, speed);
                if ((getMotorEncoder(port_L) > 0.9 * Dist / (PI * Dw) * 360) ||
                    (getMotorEncoder(port_R) > 0.9 * Dist / (PI * Dw) * 360)) {
                    setMotorSpeed(port_L, 10);
                    setMotorSpeed(port_R, 10);
                }
                delay(1);
            }
            else
            {
                setMotorSpeed(port_L, 0);
                setMotorSpeed(port_R, 0);
            }
        }
        setMotorSpeed(port_L, 0);
        setMotorSpeed(port_R, 0);
        delay(delay_);
    });
}

void MotorController::Turn(float angle, int delay_)
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
    delay(delay_);
}

void  MotorController::CircularMoveLeft(float radius, float distance_w, int delay_) {

    float distance_l = 2 * PI * radius, distance_r = 2 * PI * (radius + Dr);

    auto jS  = std::async([this,distance_r,distance_w] {
        getMotorEncoder(port_R);
        while (getMotorEncoder(port_R) < (distance_r * distance_w) / (PI * Dw) * 360) {
            if (SensoreStatus_ == true) {
                this->sensoreDistance = getSensorValue(this->senPort);
                if (sensoreDistance <= 55) {
                    this->SensoreDangerDistance_ = true;
                } else {
                    this->SensoreDangerDistance_ = false;
                }
            }
        }
    });

    //Поток для инкодера левого двигателя
    auto j1 = std::async([this, distance_l, radius, distance_w] {
        resetMotorEncoder(port_L);
        resetMotorEncoder(port_R);

        while (getMotorEncoder(port_L) < (distance_l * distance_w) / (PI * Dw) * 360) {
            if(SensoreDangerDistance_ == false) {
                setMotorSpeed(port_L, static_cast<int8_t>(this->pwrCircularM / ((Dr + radius) / (radius))));
                if (getMotorEncoder(port_L) > 0.9 * (distance_l * distance_w) / (PI * Dw) * 360) {
                    setMotorSpeed(port_L, static_cast<int8_t>(this->pwrCircularM / ((Dr + radius) / (radius))));
                }
                if ((getMotorEncoder(port_R) / (Dr + radius) / (radius)) > getMotorEncoder(port_L)) {
                    setMotorSpeed(port_L, static_cast<int8_t>(this->pwrCircularM / ((Dr + radius) / (radius))) + 1);
                }
                if ((getMotorEncoder(port_R) / (Dr + radius) / (radius)) < getMotorEncoder(port_L)) {
                    setMotorSpeed(port_L, static_cast<int8_t>(this->pwrCircularM / ((Dr + radius) / (radius))) - 1);
                }
                delay(1);
            }
            else
            {
                setMotorSpeed(port_L, 0);
            }
        }
        setMotorSpeed(port_L, 0);
        this->pwrCircularM = 80;
    });


    auto j2 = std::async([this, distance_r, radius, distance_w] {
        resetMotorEncoder(port_R);
        resetMotorEncoder(port_L);


        while (getMotorEncoder(port_R) < (distance_r * distance_w) / (PI * Dw) * 360) {
            if(SensoreDangerDistance_ == false) {
                setMotorSpeed(port_R, this->pwrCircularM);
                if (getMotorEncoder(port_R) > 0.9 * (distance_r * distance_w) / (PI * Dw) * 360) {
                    this->pwrCircularM = 30;
                    setMotorSpeed(port_R, this->pwrCircularM);
                }
                delay(1);
            }
            else
            {
                setMotorSpeed(port_R, 0);
            }
        }
        setMotorSpeed(port_R, 0);
    });
    delay(delay_);
}

void  MotorController::CircularMoveRight(float radius, float distance_w, int delay_)
{
    float distance_l = 2*PI*(radius + Dr), distance_r = 2*PI*radius;

    auto jS  = std::async([this,distance_l,distance_w] {
        getMotorEncoder(port_L);
        while (getMotorEncoder(port_L) < (distance_l * distance_w) / (PI * Dw) * 360) {
            if (SensoreStatus_ == true) {
                this->sensoreDistance = getSensorValue(this->senPort);
                if (sensoreDistance <= 55) {
                    this->SensoreDangerDistance_ = true;
                } else {
                    this->SensoreDangerDistance_ = false;
                }
            }
        }
    });

    //Поток для инкодера левого двигателя
    auto j1 = std::async([this, distance_r, radius, distance_w]
                         {
                             resetMotorEncoder(port_L);
                             resetMotorEncoder(port_R);

                             while(getMotorEncoder(port_R) < (distance_r * distance_w) / (PI * Dw) * 360 )
                             {
                                 if(SensoreDangerDistance_ == false) {
                                     setMotorSpeed(port_R, static_cast<int8_t>(this -> pwrCircularM / ((Dr + radius) / (radius))));
                                     if (getMotorEncoder(port_R) > 0.9 * (distance_r * distance_w) / (PI * Dw) * 360) {
                                         setMotorSpeed(port_R, static_cast<int8_t>(this->pwrCircularM /
                                                                                   ((Dr + radius) / (radius))));
                                     }
                                     if ((getMotorEncoder(port_L) / (Dr + radius) / (radius)) >
                                         getMotorEncoder(port_R)) {
                                         setMotorSpeed(port_R, static_cast<int8_t>(this->pwrCircularM /
                                                                                   ((Dr + radius) / (radius))) + 1);
                                     }
                                     if ((getMotorEncoder(port_L) / (Dr + radius) / (radius)) <
                                         getMotorEncoder(port_R)) {
                                         setMotorSpeed(port_R, static_cast<int8_t>(this->pwrCircularM /
                                                                                   ((Dr + radius) / (radius))) - 1);
                                     }
                                     delay(1);
                                 }
                                 else{
                                     setMotorSpeed(port_R, 0);
                                 }
                             }
                             setMotorSpeed(port_R, 0);
                             this->pwrCircularM = 80;
                         });


    auto j2 = std::async([this, distance_l,radius, distance_w]
                         {
                             resetMotorEncoder(port_L);
                             resetMotorEncoder(port_R);

                             while(getMotorEncoder(port_L) < (distance_l * distance_w) / (PI * Dw) * 360 )
                             {
                                 if(SensoreDangerDistance_ == false) {
                                     setMotorSpeed(port_L, this->pwrCircularM);
                                     if (getMotorEncoder(port_L) > 0.9 * (distance_l * distance_w) / (PI * Dw) * 360) {
                                         this->pwrCircularM = 30;
                                         setMotorSpeed(port_L, this->pwrCircularM);
                                     }
                                     delay(1);
                                 }
                                 else
                                 {
                                     setMotorSpeed(port_L, 0);
                                 }
                             }
                             setMotorSpeed(port_L, 0);
                         });
    delay(delay_);
}

void MotorController::SmoothTurn(float radius, float angle, int delay_)
{
    MotorController GoOn(port_L, port_R, Dr, Dw, w, pwrTurn, sensoreModeOn, senPort);
    float w = abs(((PI * radius * angle) / 180) / ((PI * radius * 360) / 180));
    if(angle > 0)
    {
        GoOn.CircularMoveRight(radius,w);
    }
    else
    {
        GoOn.CircularMoveLeft(radius,w);
    }
    delay(delay_);
}

