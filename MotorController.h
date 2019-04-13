//
// Created by stdplus on 25.03.19.
//

#ifndef MOTORCONTROLLER_MOTORCONTROLLER_H
#define MOTORCONTROLLER_MOTORCONTROLLER_H

#include <rbc.h>
// MotorController.cpp
//libstdc++.a
//version 2.04.19

class MotorController{
public:
    MotorController();
    MotorController(tMotor Left, tMotor Right, bool Sensore, tSensor sensorePort);
    MotorController(tMotor Left, tMotor Right, float robotRadius, float Dw, float w, int pwrTurn, bool SensoreMode, tSensor sensorePort);

    //Функция движения по прямой на заданное растояние с заданной скоростью(расстояние указывается в мм, скорость в % от -100 до 100)
    void Move(int speed, float distance, int delay_ = 0);
    //Функция поворота принимающая угол в градусах(для поворота влево используются отрицательные значения, для поворота вправо - положительные)
    void Turn(float angle, int delay_ = 0);
    //Функция движения по окружности вправо с заданной скоростью и заданное число кругов(если передается 1, то это полная окружность, если передать 0.5, то это половина окружности и т.д)
    void CircularMoveLeft(float radius, float distance_w, int delay_ = 0);
    //Функция движения по окружности влево с заданной скоростью и заданное число кругов(если передается 1, то это полная окружность, если передать 0.5, то это половина окружности и т.д)
    void CircularMoveRight(float radius, float distance_w, int delay_ = 0);
    //Функция закругленного поворота принимающая радиус поворота и угол на который надо повернуть
    void SmoothTurn(float radius, float angle, int delay_ = 0);

    void Targeting(float distance_to_object);

    void SensoreOn();
    void SensoreOff();

private:
    float Dr;
    float Dw;
    float w ;
    int pwrTurn;
    int pwrMove = 50;
    float GlAl = 0;
    tMotor port_L, port_R;
    int pwrCircularM = 80;
    bool sensoreModeOn;
    tSensor senPort;
    bool SensoreStatus_ = false;
    float sensoreDistance;
    bool SensoreDangerDistance_;
};

#endif //MOTORCONTROLLER_MOTORCONTROLLER_H

