#ifndef MAIN_HPP
#define MAIN_HPP
#include <stdint.h>
#include "utils.hpp"
#define SENDER 0
#define VEHICLE 0
#define DEBUG 0
#define BAUD 9600
#define RECEIVER_SLEEP 10
#define SENDER_SLEEP (RECEIVER_SLEEP * 4)
#define NO_MSG 0.45
#define SENDER_INPUT_NO_MSG 0.45
#define VOLTAGE 11.4
#define VOLTAGE_FACTOR 3.0
#define MESSAGES 10
#define PAYLOAD_LENGTH 35

#if SENDER == 1
#define CE_PIN 7
#define CSN_PIN 8
#define BUZZER_PIN NO_PIN
#define LED_PIN NO_PIN
#define VOLTAGE_PIN NO_PIN
#else
#define CE_PIN 7
#define CSN_PIN 8
#define BUZZER_PIN 4
#define LED_PIN 5
#define VOLTAGE_PIN A7
#endif

constexpr uint8_t address[][6] = {"Send1", "Recv1"};

#endif