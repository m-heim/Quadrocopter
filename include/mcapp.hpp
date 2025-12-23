#ifndef MCAPP_HPP_
#define MCAPP_HPP_
#define MCAPP_VERSION 1
#include "Arduino.h"
#include "communication_provider.hpp"

enum
{
    HELLO = 10,
    BYE,
    CONFIG,
    CONFIG_ACK,
    CONTROL,
    STATUS_SENDER,
    STATUS_RECEIVER,
} MessageType;

struct Message
{
    uint8_t msg;
    uint8_t length;
    uint8_t data[PAYLOAD_LENGTH - 2];
};

struct ReceiverPayload
{
    int8_t speed;
    int8_t pitch;
    int8_t roll;
    int8_t yaw;
};

class MCApp
{
public:
    MCApp() {}
    inline int getVersion()
    {
        return MCAPP_VERSION;
    }
    inline float getVoltage()
    {
        if (voltagePin == -1)
        {
            ledError();
        }
        else
        {
            (analogRead(voltagePin) / 1024) * voltageFactor;
        }
    }
    inline void initPiezo(int pin)
    {
        pinMode(pin, OUTPUT);
        piezoPin = pin;
    }
    inline void initVoltage(int pin, float factor)
    {
        pinMode(pin, INPUT);
        voltagePin = pin;
        voltageFactor = factor;
    }
    inline void buzz(int freq, int dur)
    {
        if (piezoPin == -1)
        {
            ledError();
        }
        tone(piezoPin, freq, dur);
    }
    inline void ledError()
    {
        while (1)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(1000);
            digitalWrite(LED_BUILTIN, LOW);
            delay(1000);
        }
    }
    inline void initLog(long baud)
    {
        if (Serial)
        {
            Serial.begin(baud);
            Serial.println("Serial init");
        }
    }
    inline void log(char *msg)
    {
        if (Serial)
        {
            Serial.println(msg);
        }
    }
    inline void initLed(int pin)
    {
        pinMode(pin, OUTPUT);
        this->ledPin = pin;
    }
    inline void setLed(int val)
    {
        if (this->ledPin == -1)
        {
            this->ledError();
        }
        else
        {
            digitalWrite(ledPin, val);
        }
    }
    inline void ledToggle()
    {
        this->setLed(ledState);
        ledState = !ledState;
    }
    CommunicationProvider &communication;

private:
    int voltagePin = -1;
    int piezoPin = -1;
    int ledPin = -1;
    float voltageFactor = 1;
    bool ledState = false;
};
#endif