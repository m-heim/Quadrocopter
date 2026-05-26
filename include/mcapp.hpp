#ifndef MCAPP_HPP_
#define MCAPP_HPP_
#define MCAPP_VERSION 1
#define NO_PIN -1
#include "Arduino.h"
#include "communication_provider.hpp"
#include "init.hpp"
#include "utils.hpp"

class IOHandler {
public:
  virtual void send(const char *msg, int len) = 0;
};

class UartIOHandler : public IOHandler {
public:
  void send(const char *msg, int len) override { Serial.write(msg, len); }
  int recv(char *buf, int len) { return Serial.readBytes(buf, len); }
};

struct QuadrocopterMessage {
  int8_t speeds[4];
  uint8_t funs[4]; // for future use, can be used for extra features or to send
                   // more data without changing the structure of the message
  bool gyroSetup;
  bool hover;
};

template <typename T> class InputHandler {
  virtual bool handle(const std::string &s, T &payload) = 0;
};
class UartInputHandler : public InputHandler<QuadrocopterMessage> {
public:
  bool handle(std::string s, QuadrocopterMessage &p) override {
    if (s.length() < 1) // if we didn't get any data, do nothing (instead of
                        // applying old data or random data)
    {
      return false;
    } else if (s.length() >= (1 + 4 + 3) &&
               s.charAt(0) ==
                   'c') // we have a speed change command, it should be in the
                        // format c<speed0><speed1><speed2><speed3>\n, where
                        // speeds are between -127 and 127
    {
      int index = 1;
      for (int i = 0; i < 4; i++) {
        String numStr = s.substring(index, s.indexOf(',', index));
        if (numStr.length() >= 1) {
          p.speeds[i] = (int8_t)numStr.toInt();
        } else {
          p.speeds[i] = 0;
        }
        index += numStr.length() + 1;
      }
      timer.start();
      p.gyroSetup = false;
      p.hover = false;
    } else if (s.charAt(0) ==
               's') // we have a gyro setup command, it should be just s\n
    {
      p.gyroSetup = true;
      p.hover = false;
      memset(p.funs, 0, 4);
    } else {
      return false;
    }
    return true;
  }
  bool isRecent() { return !timer.isExpired(); }

private:
  ArduinoTimer timer = ArduinoTimer(SENDER_INPUT_NO_MSG);
}

class MCApp {
public:
  MCApp(CommunicationProvider *remote, IOHandler &iohandler)
      : remote(remote), iohandler(iohandler) {
    pinMode(LED_BUILTIN, OUTPUT);
  }
  void output(int start, int stop, int step, float seconds) {
    int s = (int)((seconds / 1000));
    for (int i = start; i <= stop; i += step) {
      buzz(i, s);
      delay(s);
    }
  }
  CommunicationProvider *getRemote() { return this->remote; }

  bool recentMessage() { return !timer.isExpired(); }
  bool handle(SenderPayload &payload);

  bool handle2(const QuadrocopterMessage &p, bool valid);

  void noPackageAction() {
    log("No package");
    setLed(1);
    buzz(freq, 10);
    freq += 400;
    if (freq > 4000) {
      freq = FREQ_BASE;
    }
  }
  int getVersion() { return MCAPP_VERSION; }
  float getVoltage() {
    if (voltagePin == -1) {
      return 0;
    } else {
      return (analogRead(voltagePin) / 1024) * voltageFactor;
    }
  }
  bool verifyVoltage() {
    return (voltagePin == NO_PIN) || (getVoltage() > voltage);
  }
  void initPiezo(int pin) {
    pinMode(pin, OUTPUT);
    piezoPin = pin;
  }
  void initVoltage(int pin, float factor, float v) {
    pinMode(pin, INPUT);
    voltagePin = pin;
    voltageFactor = factor;
    voltage = v;
  }
  // buzz if possible
  void buzz(int freq, int dur) {
    if (piezoPin == NO_PIN) {
      return;
    }
    tone(piezoPin, freq, dur);
  }
  // infinite loop led error
  void ledError() {
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
    }
  }
  // init logger
  void initLog(long baud) {
#if DEBUG == 1
    if (Serial) {
      Serial.begin(baud);
      Serial.println("Serial init");
    }
#endif
  }
  void log(char *msg) {
#if DEBUG == 1
    if (Serial) {
      Serial.println(msg);
    }
#endif
  }
  void log(__FlashStringHelper *msg) {
#if DEBUG == 1
    if (Serial) {
      Serial.println(msg);
    }
#endif
  }
  void initLed(int pin) {
    pinMode(pin, OUTPUT);
    this->ledPin = pin;
  }
  void infinite() {
    while (1) {
    }
  }
  void setLed(int val) {
    if (this->ledPin == NO_PIN) {
      return;
    } else {
      digitalWrite(ledPin, val);
    }
  }
  // toggle led state and set
  void ledToggle() {
    this->setLed(ledState);
    ledState = !ledState;
  }
  CommunicationProvider *remote;
  IOHandler &iohandler;

  ReceiverPayload &getPayload() { return payload; }

private:
  int voltagePin = NO_PIN;
  int piezoPin = NO_PIN;
  int ledPin = NO_PIN;
  float voltageFactor = 1.0;
  float voltage;
  bool ledState = false;
  int freq = FREQ_BASE;
  long msg_a;               // last control packet
  bool isConnected = false; // connection status
  uint8_t msgBuf[PAYLOAD_LENGTH];
  ArduinoTimer timer = ArduinoTimer(NO_MSG);
};
#endif