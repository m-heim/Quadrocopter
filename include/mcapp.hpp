#ifndef MCAPP_HPP_
#define MCAPP_HPP_
#define MCAPP_VERSION 1
#define NO_PIN -1
#include "init.hpp"
#include "utils.hpp"
#include "Arduino.h"
#include "communication_provider.hpp"

class MCApp
{
public:
    MCApp() : remote(nullptr)
    {
        pinMode(LED_BUILTIN, OUTPUT);
    }
    MCApp(CommunicationProvider *remote) : remote(remote)
    {
    }
    void output(int start, int stop, int step, float seconds)
    {
        int s = (int)((seconds / 1000));
        for (int i = start; i <= stop; i += step)
        {
            buzz(i, s);
            delay(s);
        }
    }
    CommunicationProvider *getRemote()
    {
        return this->remote;
    }

    bool recentMessage()
    {
        return (millis() - msg_a) < NO_MSG;
    }
    bool handle(SenderPayload &payload);

    bool handle2(int8_t *buf, bool gyroSetup);

    void noPackageAction()
    {
        log("No package");
        setLed(1);
        buzz(freq, 10);
        freq += 400;
        if (freq > 4000)
        {
            freq = FREQ_BASE;
        }
    }
    int getVersion()
    {
        return MCAPP_VERSION;
    }
    void printPayload(ReceiverPayload p)
    {
        if (Serial)
        {
            log("Payload from sender: ");
            Serial.print(p.speed);
            Serial.print(p.yaw);
            Serial.print(p.pitch);
            Serial.print(p.roll);
            Serial.print("\n");
        }
    }
    float getVoltage()
    {
        if (voltagePin == -1)
        {
            return 0;
        }
        else
        {
            return (analogRead(voltagePin) / 1024) * voltageFactor;
        }
    }
    bool verifyVoltage()
    {
        return (voltagePin == NO_PIN) || (getVoltage() > voltage);
    }
    void initPiezo(int pin)
    {
        pinMode(pin, OUTPUT);
        piezoPin = pin;
    }
    void initVoltage(int pin, float factor, float v)
    {
        pinMode(pin, INPUT);
        voltagePin = pin;
        voltageFactor = factor;
        voltage = v;
    }
    // buzz if possible
    void buzz(int freq, int dur)
    {
        if (piezoPin == NO_PIN)
        {
            return;
        }
        tone(piezoPin, freq, dur);
    }
    // infinite loop led error
    void ledError()
    {
        while (1)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(1000);
            digitalWrite(LED_BUILTIN, LOW);
            delay(1000);
        }
    }
    // init logger
    void initLog(long baud)
    {
#if DEBUG == 1
        if (Serial)
        {
            Serial.begin(baud);
            Serial.println("Serial init");
        }
#endif
    }
    void log(char *msg)
    {
#if DEBUG == 1
        if (Serial)
        {
            while (!Serial.availableForWrite())
            {
            }
            Serial.println(msg);
        }
#endif
    }
    void initLed(int pin)
    {
        pinMode(pin, OUTPUT);
        this->ledPin = pin;
    }
    void infinite()
    {
        while (1)
        {
        }
    }
    void setLed(int val)
    {
        if (this->ledPin == NO_PIN)
        {
            return;
        }
        else
        {
            digitalWrite(ledPin, val);
        }
    }
    // toggle led state and set
    void ledToggle()
    {
        this->setLed(ledState);
        ledState = !ledState;
    }
    CommunicationProvider *remote;

    ReceiverPayload &getPayload()
    {
        return payload;
    }

private:
    int voltagePin = NO_PIN;
    int piezoPin = NO_PIN;
    int ledPin = NO_PIN;
    float voltageFactor = 1.0;
    float voltage;
    bool ledState = false;
    int freq = FREQ_BASE;
    long msg_a; // last control packet
    bool isConnected = false; // connection status
    ReceiverPayload payload = {0, 0, 0, 0}; // current payload
    uint8_t msgBuf[PAYLOAD_LENGTH];
};
#endif