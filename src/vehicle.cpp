#include <Arduino.h>
#include "vehicle.hpp"

void motor_1_drive(int8_t speed)
{
    if (speed > 0)
    {
        analogWrite(MOTOR_1_PWM_L, speed * 2);
        analogWrite(MOTOR_1_PWM_R, 0);
    }
    else if (speed < 0)
    {
        analogWrite(MOTOR_1_PWM_L, 0);
        analogWrite(MOTOR_1_PWM_R, (speed * -2));
    }
    else
    {
        analogWrite(MOTOR_1_PWM_L, 0);
        analogWrite(MOTOR_1_PWM_R, 0);
    }
}

void motor_2_drive(int8_t speed)
{
    if (speed > 0)
    {
        analogWrite(MOTOR_2_PWM_L, speed * 2);
        analogWrite(MOTOR_2_PWM_R, 0);
    }
    else if (speed < 0)
    {
        analogWrite(MOTOR_2_PWM_L, 0);
        analogWrite(MOTOR_2_PWM_R, (speed * -2));
    }
    else
    {
        analogWrite(MOTOR_2_PWM_L, 0);
        analogWrite(MOTOR_2_PWM_R, 0);
    }
}

void motor_1_setup(int status)
{
    pinMode(MOTOR_1_EN_L, OUTPUT);
    pinMode(MOTOR_1_EN_R, OUTPUT);
    pinMode(MOTOR_1_PWM_L, OUTPUT);
    pinMode(MOTOR_1_PWM_R, OUTPUT);
    motor_1_drive(0);
    digitalWrite(MOTOR_1_EN_L, status);
    digitalWrite(MOTOR_1_EN_R, status);
}

void motor_2_setup(int status)
{
    pinMode(MOTOR_2_EN_L, OUTPUT);
    pinMode(MOTOR_2_EN_R, OUTPUT);
    pinMode(MOTOR_2_PWM_L, OUTPUT);
    pinMode(MOTOR_2_PWM_R, OUTPUT);
    motor_2_drive(0);
    digitalWrite(MOTOR_2_EN_L, status);
    digitalWrite(MOTOR_2_EN_R, status);
}

void drive(int8_t speed, int8_t steering)
{
    int16_t speed_left = speed + steering;
    int16_t speed_right = speed - steering;
    if (speed_left > 127)
    {
        speed_left = 127;
    }
    else if (speed_left < -127)
    {
        speed_left = -127;
    }

    if (speed_right > 127)
    {
        speed_right = 127;
    }
    else if (speed_right < -127)
    {
        speed_right = -127;
    }

    motor_1_drive(speed_left);
    motor_2_drive(speed_right);
}