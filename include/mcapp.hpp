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

class InputPayload
{
public:
    InputPayload(uint8_t *defaultBuf, int defaultBufLen) : len(0), valid(false)
    {
        memcpy(this->defaultBuf, defaultBuf, defaultBufLen);
        this->defaultBufLen = defaultBufLen;
    }
    void setBuf(uint8_t *buf, int len)
    {
        memcpy(this->buf, buf, len);
        this->len = len;
        this->valid = true;
    }
    void invalidate() { this->valid = false; }
    uint8_t *getBuf() { return valid ? buf : defaultBuf; }
    int getLen() { return valid ? len : defaultBufLen; }
    bool isValid() { return valid; }

private:
    uint8_t buf[PAYLOAD_LENGTH];
    int len;
    bool valid;
    uint8_t defaultBuf[PAYLOAD_LENGTH];
    int defaultBufLen;
};

class InputHandler
{
    virtual bool handle(InputPayload &p) = 0;
};
class UartInputHandler : public InputHandler
{
public:
    UartInputHandler()
    {
        if (Serial)
        {
            // Serial.begin(115200);
            Serial.setTimeout(10);
        }
    }
    bool handle(InputPayload &p) override
    {
        if (!Serial)
        {
            p.invalidate();
            return false;
        }
        String s = Serial.readStringUntil('\0');
        if (s.length() < 2) // if we didn't get any data, do nothing (instead of
                            // applying old data or random data)
        {
            p.invalidate();
            return false;
        }
        // Serial.println("Received: " + s);
        uint8_t buf[PAYLOAD_LENGTH];
        uint32_t i = 0;
        while ((i * 2 + 1) < s.length() && i < sizeof(buf))
        {
            buf[i] = libmh::getNibble(s.charAt(i * 2)) << 4 | libmh::getNibble(s.charAt((i * 2) + 1));
            i++;
        }
        if (i >= sizeof(buf))
        {
            p.invalidate();
            return false;
        }
        p.setBuf(buf, i);
        return true;
    }
    bool handleOutput() {

    };
    bool isRecent() { return !timer.isExpired(); }

private:
    libmh::ArduinoTimer timer = libmh::ArduinoTimer(SENDER_INPUT_NO_MSG);
};

class Logger
{
public:
    Logger(bool enabled) : enabled(enabled) {}
    virtual void log(const char *msg) = 0;
    virtual void log(const __FlashStringHelper *msg) = 0;

protected:
    bool enabled;
};

class ArduinoLogger : public Logger
{
public:
    ArduinoLogger(bool enabled, unsigned long baud) : Logger(enabled), baud(baud)
    {
    }
    void init()
    {
        if (enabled && Serial)
        {
            Serial.begin(baud);
            Serial.println("AL.I 1");
        }
    }
    void log(const char *msg) override
    {
        if (enabled && Serial)
        {
            Serial.println(msg);
        }
    }
    void log(const __FlashStringHelper *msg) override
    {
        if (enabled && Serial)
        {
            Serial.println(msg);
        }
    }

private:
    unsigned long baud;
};

class Buzzer
{
public:
    Buzzer(int pin) : pin(pin)
    {
        if (pin != NO_PIN)
        {
            pinMode(pin, OUTPUT);
        }
    }
    void output(int start, int stop, int step, float seconds)
    {
        if (pin == NO_PIN)
        {
            return;
        }
        int s = (int)((seconds / 1000));
        for (int i = start; i <= stop; i += step)
        {
            buzz(i, s);
            delay(s);
        }
    }
    void buzz(int freq, int dur)
    {
        if (pin == NO_PIN)
        {
            return;
        }
        tone(pin, freq, dur);
    }

private:
    int pin;
};

class Led
{
public:
    Led(int pin) : pin(pin)
    {
        if (pin != NO_PIN)
        {
            pinMode(pin, OUTPUT);
        }
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
        this->pin = pin;
    }
    void setLed(int val)
    {
        if (this->pin == NO_PIN)
        {
            return;
        }
        digitalWrite(pin, val);
    }
    // toggle led state and set
    void ledToggle()
    {
        this->setLed(ledState);
        ledState = !ledState;
    }

private:
    int pin;
    bool ledState = false;
};

class VoltageHandler
{
public:
    VoltageHandler(int pin, float factor, float voltage) : pin(pin), voltageFactor(factor), voltage(voltage)
    {
        if (pin != NO_PIN)
        {
            pinMode(pin, INPUT);
        }
    }
    float getVoltage()
    {
        if (pin == NO_PIN)
        {
            return 0;
        }
        return (analogRead(pin) / 1024) * voltageFactor;
    }
    bool verifyVoltage()
    {
        return (pin == NO_PIN) || (getVoltage() > voltage);
    }

private:
    int pin = NO_PIN;
    float voltageFactor = 1.0;
    float voltage = 0;
};

class MCApp
{
public:
    MCApp(CommunicationProvider &remote, Logger &logger, Buzzer &buzzer, Led &led, VoltageHandler &voltageHandler) : remote(remote), logger(logger), buzzer(buzzer), led(led), voltageHandler(voltageHandler)
    {
        // delay(1000);
        pinMode(LED_BUILTIN, OUTPUT);
        // logger.log("A.I 1");
    }
    CommunicationProvider &getRemote() { return remote; }

    bool recentMessage() { return !timer.isExpired(); }

    void noPackageAction()
    {
        logger.log(FLASH_STRING("App.N"));
        led.setLed(1);
        buzzer.buzz(freq, 10);
        freq += 400;
        if (freq > 4000)
        {
            freq = FREQ_BASE;
        }
    }
    int getVersion() { return MCAPP_VERSION; }
    void infinite()
    {
        while (1)
        {
        }
    }
    CommunicationProvider &remote;
    Logger &logger;
    Buzzer &buzzer;
    MessageHandler messageHandler;
    Led &led;
    VoltageHandler &voltageHandler;
    Message messages[MESSAGES];

protected:
    int freq = FREQ_BASE;
    uint8_t msgBuf[PAYLOAD_LENGTH];
    libmh::ArduinoTimer timer = libmh::ArduinoTimer(NO_MSG);
    int msg_n = 0; // message counter, used for logging and other purposes
};

class MCAppReceiver : public MCApp
{
public:
    MCAppReceiver(CommunicationProvider &remote, Logger &logger, Buzzer &buzzer, Led &led, VoltageHandler &voltageHandler) : MCApp(remote, logger, buzzer, led, voltageHandler) {}
    int handle();
};

class MCAppSender : public MCApp
{
public:
    MCAppSender(CommunicationProvider &remote, Logger &logger, Buzzer &buzzer, Led &led, VoltageHandler &voltageHandler) : MCApp(remote, logger, buzzer, led, voltageHandler) {}
    int handle(InputPayload &p);
};
#endif