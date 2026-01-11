#ifndef MCAPP_HPP_
#define MCAPP_HPP_
#define MCAPP_VERSION 1
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
    inline void output(int start, int stop, int step, float seconds)
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

    inline bool recentMessage()
    {
        return (millis() - msg_a) < NO_MSG;
    }
    inline bool handle()
    {
        bool valid = false;
        if (remote->read() > 0)
        { // is there a payload? get the pipe number that recieved it
            valid = true;
            uint8_t *buf = remote->getBuf();
            if (buf[0] == HELLO)
            {
                isConnected = true;
                log("Hello from sender");
                output(500, 900, 100, 0.04);
            }
            else if (buf[0] == BYE)
            {
                isConnected = false;
                log("Bye from sender");
                output(900, 500, -100, 0.04);
            }
            else if (buf[0] == GYRO_SETUP)
            {
                log("Received gyro setup");
                // setGravity();
            }
            else if (buf[0] == MOTOR_SETUP)
            {
                log("Received motor setup");
            }
            else if (buf[0] == CONTROL)
            {
                log("Received payload");
                memcpy(&payload, buf + 2, sizeof(payload));
                printPayload(payload);
                msg_a = millis();
            }
            else
            {
                log("Invalid message");
                valid = false;
            }
        }
        else
        {
            log("Radio not available");
        }
        if (!recentMessage())
        {
            noPackageAction();
            valid = false;
            isConnected = false;
        }
        else
        {
        }
        return valid;
    }

    inline bool handle2(int8_t *buf, bool gyroSetup)
    {
        int payloadLength = 0;
        int action = 1;
        if (!isConnected)
        {
            msgBuf[0] = HELLO;
            msgBuf[1] = 0;
            payloadLength = 2;
            log("Sending hello");
        }
        else if (action == 1)
        {
            msgBuf[0] = CONTROL;
            msgBuf[1] = 4;
            payloadLength = 2 + sizeof(ReceiverPayload);
            payload.speed = buf[0];
            payload.pitch = buf[1];
            payload.yaw = buf[2];
            payload.roll = buf[3];
            memcpy(msgBuf + 2, &payload, sizeof(payload));
            log("Sending");
        }
        else
        {
            log("Unknown action");
        }
        bool report = remote->write(msgBuf, payloadLength);
        if (report)
        {
            log("Message was successfully transmitted");
            isConnected = true;
        }
        else
        {
            log("No ack from receiver");
            isConnected = false;
        }
        return isConnected;
    }

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
    inline int getVersion()
    {
        return MCAPP_VERSION;
    }
    inline void printPayload(ReceiverPayload p)
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
    inline float getVoltage()
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
    inline bool verifyVoltage()
    {
        return (voltagePin == -1) || (getVoltage() > voltage);
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
        if (piezoPin == -1)
        {
            return;
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
            return;
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
    CommunicationProvider *remote;

    ReceiverPayload &getPayload()
    {
        return payload;
    }

private:
    int voltagePin = -1;
    int piezoPin = -1;
    int ledPin = -1;
    float voltageFactor = 1;
    float voltage = 0;
    bool ledState = false;
    int freq = FREQ_BASE;
    // last control packet
    long msg_a;
    bool isConnected = false;
    ReceiverPayload payload = {0, 0, 0, 0};
    uint8_t msgBuf[PAYLOAD_LENGTH];
};
#endif