#ifndef MCAPP_HPP_
#define MCAPP_HPP_
#define MCAPP_VERSION 1
#include "init.hpp"
#include "Arduino.h"
#include "communication_provider.hpp"

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
            return (analogRead(voltagePin) / 1024) * voltageFactor;
        }
    }
    inline bool verifyVoltage()
    {
        return getVoltage() > voltage;
    }
    inline void initPiezo(int pin)
    {
        pinMode(pin, OUTPUT);
        piezoPin = pin;
    }
    inline void initVoltage(int pin, float factor, float v)
    {
        pinMode(pin, INPUT);
        voltagePin = pin;
        voltageFactor = factor;
        voltage = v;
    }
    inline void buzz(int freq, int dur)
    {
        if (noPiezo)
        {
            return;
        }
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
        #if DEBUG == 1
        if (Serial)
        {
            Serial.begin(baud);
            Serial.println("Serial init");
        }
        #endif
    }
    inline void log(char *msg)
    {
        #if DEBUG == 1
        if (Serial)
        {
            Serial.println(msg);
        }
        #endif
    }
    inline void initLed(int pin)
    {
        pinMode(pin, OUTPUT);
        this->ledPin = pin;
    }
    inline void infinite()
    {
        while (1)
        {
        }
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
    inline void setNoPiezo(bool v)
    {
        noPiezo = v;
    }
    inline void ledToggle()
    {
        this->setLed(ledState);
        ledState = !ledState;
    }

private:
    int voltagePin = -1;
    int piezoPin = -1;
    int ledPin = -1;
    float voltageFactor = 1;
    float voltage = 0;
    bool ledState = false;
    bool noPiezo = false;
};
#endif