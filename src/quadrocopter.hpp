#ifndef QUADROCOPTER_HPP__
#define QUADROCOPTER_HPP__
#include "mcapp.hpp"
#include <Servo.h>
#include "nrf24l01_provider.hpp"
#include <Adafruit_MPU6050.h>
#include <pid.hpp>

#if SENDER == 0 && VEHICLE == 0
#define SERVOS 4
#define PIDS 2
#define PIEZO 4
#define LED 5
#define INVOLTAGE A7
#define GRAVITY 4000

#define FRONT 0x00
#define BACK 0x02
#define LEFT 0x00
#define RIGHT 0x01

#define FRONT_LEFT FRONT | LEFT
#define FRONT_RIGHT FRONT | RIGHT
#define BACK_LEFT BACK | LEFT
#define BACK_RIGHT BACK | RIGHT

#define EXPONENTIAL_FACTOR 0.45
#define LP (1 - 0.45)

extern NRF24L01Provider radio;
extern MCAppReceiver app;

void initServos();

void setServos(int speed);

void setMotors();

bool setAlphaVals();

void setGravity();

void initGyro();

void setAlpha();

void setGyro();

void setValues();

void setSpeeds(int8_t sVals[4], bool motorsApply);
void printVoltage();
void initAccelerometer();

void initPIDS();

#endif
#endif