#ifndef MAIN_HPP
#define MAIN_HPP
#define SENDER 0
#define VEHICLE 0
#define PIEZO 4
#define LED 5
#define INVOLTAGE A7
#define RECEIVER_SLEEP 20
#define SENDER_SLEEP 40
#define NO_MSG 450
#define VOLTAGE 11.4
#define GRAVITY 4000
#define DEBUG 0

#if SENDER == 1
#define CE_PIN 7
#define CSN_PIN 8
#else
#define CE_PIN 7
#define CSN_PIN 8
#endif

#define FRONT 0x00
#define BACK 0x02
#define LEFT 0x00
#define RIGHT 0x01

#define FRONT_LEFT FRONT | LEFT
#define FRONT_RIGHT FRONT | RIGHT
#define BACK_LEFT BACK | LEFT
#define BACK_RIGHT BACK | RIGHT

#define EXPONENTIAL_FACTOR 0.45
#endif