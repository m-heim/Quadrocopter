#ifndef MAIN_HPP
#define MAIN_HPP
#include <stdint.h>
#define SENDER 0
#define VEHICLE 0
#define DEBUG 0
#define RECEIVER_SLEEP 20
#define SENDER_SLEEP (RECEIVER_SLEEP * 2)
#define NO_MSG 450
#define VOLTAGE 11.4
constexpr uint8_t address[][6] = {"Send1", "Recv1"};
#endif