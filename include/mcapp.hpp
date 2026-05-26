#ifndef MCAPP_HPP_
#define MCAPP_HPP_
#define MCAPP_VERSION 1
#define NO_PIN -1
#include "Arduino.h"
#include "communication_provider.hpp"
#include "init.hpp"
#include "utils.hpp"

typedef struct
{
    int8_t speeds[4];
    uint8_t funs[4]; // for future use, can be used for extra features or to send
                     // more data without changing the structure of the message
    bool gyroSetup;
    bool hover;
} QuadrocopterMessage;

typedef struct
{
    uint8_t speeds[4];
    float orientation[3]; // roll, pitch, yaw or other data depending on the application
    float position[3];    // x, y, z or other data depending on the application
    float voltage;
} SenderPayload;

class InputHandler
{
    virtual bool handle(MessageHandler *messageHandler, Message *messages) = 0;
};
class UartInputHandler : public InputHandler
{
public:
    UartInputHandler()
    {
        if (Serial)
        {
            Serial.begin(115200);
            Serial.setTimeout(10);
        }
    }
    bool handle(MessageHandler *messageHandler, Message *messages) override
    {
        if (!Serial)
        {
            return false;
        }
        String s = Serial.readStringUntil('\0');
        Serial.println("Received: " + s);
        if (s.length() < 1) // if we didn't get any data, do nothing (instead of
                            // applying old data or random data)
        {
            return false;
        }
        uint8_t buf[PAYLOAD_LENGTH];
        for (int i = 0; (i * 2) < s.length() && i < sizeof(buf); i += 1)
        {
            buf[i] = (s.charAt(i * 2) - '0') << 4 | (s.charAt((i * 2) + 1) - '0');
        }
        if (buf[0] == PACKAGE && messageHandler->parsePackage(buf, messages, MESSAGES) > 0)
        {
            timer.start();
            return true;
        }
        return false;
    }
    bool isRecent() { return !timer.isExpired(); }

private:
    ArduinoTimer timer = ArduinoTimer(SENDER_INPUT_NO_MSG);
};

class Logger
{
public:
    Logger(bool enabled) : enabled(enabled) {}
    virtual void log(const char *msg) = 0;

protected:
    bool enabled;
};

class ArduinoLogger : public Logger
{
public:
    ArduinoLogger(bool enabled, int baud) : Logger(enabled)
    {
        if (enabled && Serial)
        {
            Serial.begin(baud);
            Serial.println("ArduinoLogger.Init 1");
        }
    }
    void log(const char *msg) override
    {
        if (enabled && Serial)
        {
            Serial.println(msg);
        }
    }
};

class MCApp
{
public:
    MCApp(CommunicationProvider *remote, Logger *logger) : remote(remote), logger(logger)
    {
        pinMode(LED_BUILTIN, OUTPUT);
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
    CommunicationProvider *getRemote() { return this->remote; }

    bool recentMessage() { return !timer.isExpired(); }
    int handle(SenderPayload &payload);

    bool handle2(const QuadrocopterMessage &p, bool valid);

    void noPackageAction()
    {
        logger->log("No package");
        setLed(1);
        buzz(freq, 10);
        freq += 400;
        if (freq > 4000)
        {
            freq = FREQ_BASE;
        }
    }
    int getVersion() { return MCAPP_VERSION; }
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
    Logger *logger;
    MessageHandler messageHandler;
    Message messages[MESSAGES];

private:
    int voltagePin = NO_PIN;
    int piezoPin = NO_PIN;
    int ledPin = NO_PIN;
    float voltageFactor = 1.0;
    float voltage;
    bool ledState = false;
    int freq = FREQ_BASE;
    bool isConnected = false; // connection status
    uint8_t msgBuf[PAYLOAD_LENGTH];
    ArduinoTimer timer = ArduinoTimer(NO_MSG);
    int msg_n = 0; // message counter, used for logging and other purposes
};
#endif