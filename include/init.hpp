#ifndef MAIN_HPP
#define MAIN_HPP
#include <stdint.h>
#define SENDER 1
#define VEHICLE 0
#define DEBUG 1
#define BAUD 115200
#define RECEIVER_SLEEP 20
#define SENDER_SLEEP (RECEIVER_SLEEP * 2)
#define NO_MSG 0.45
#define SENDER_INPUT_NO_MSG 0.45
#define VOLTAGE 11.4
#define MESSAGES 10
#define PAYLOAD_LENGTH 35

#if SENDER == 1
#define CE_PIN 7
#define CSN_PIN 8
#else
#define CE_PIN 7
#define CSN_PIN 8
#endif

constexpr uint8_t address[][6] = {"Send1", "Recv1"};
#endif