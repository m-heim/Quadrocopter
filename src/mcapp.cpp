#include "mcapp.hpp"
#include "WString.h"

bool MCApp::handle(SenderPayload &p) {
  msg_n += 1;
  bool valid = false;
  int l = remote->read();
  if (l > 0) { // is there a payload? get the pipe number that recieved it
    valid = true;
    const uint8_t *o = remote->getBuf();
    uint8_t *o2 = o;
    char *position = strchr((char *)o, ',');
    while (position != NULL) {
      if (o)
        // setup gyro
        if (o[0] == GYRO_SETUP) {
          log("Received gyro setup");
          // setGravity();
        }
        // setup motor
        else if (o[0] == MOTOR_SETUP) {
          log("Received motor setup");
        }
        // control message
        else if (o[0] == CONTROL) {
          log("Received payload");
          memcpy(&payload, o + 2, sizeof(payload));
          printPayload(payload);
          timer.start();
        } else {
          log("Invalid message");
          valid = false;
        }
    }
    else {
      log(F("Recv 0"));
    }
    if (!recentMessage()) {
      noPackageAction();
      valid = false;
    } else {
    }
    if ((msg_n % 10) == 0) {
      uint8_t buf[PAYLOAD_LENGTH];
      buf[0] = STATUS_RECEIVER;
      buf[1] = sizeof(p);
      memcpy(buf + 2, &p, sizeof(p));
      getRemote()->disableReceive();
      bool report = remote->write(buf, sizeof(p) + 2);
      getRemote()->enableReceive();
      if (report) {
        log(F("Send 1"));
      } else {
        log(F("Send 0"));
      }
    }
    return valid;
  }

  bool MCApp::handle2(const QuadrocopterMessage &p, bool valid) {
    msg_n += 1;
    int payloadLength = 0;
    int action = 1;
    if (p.gyroSetup) {
      msgBuf[0] = GYRO_SETUP;
      msgBuf[1] = 0;
      payloadLength = 2;
      log("Sending setup");
    } else if (action == 1) {
      msgBuf[0] = CONTROL;
      msgBuf[1] = 4;
      payloadLength = 2 + sizeof(ReceiverPayload);
      if (error) {
        memset(msgBuf + 2, 0, 4);
      } else {
        memcpy(msgBuf + 2, &p.speeds, 4);
      }
      log("Sending");
    } else {
      log("Unknown action");
    }
    getRemote()->disableReceive();
    bool report = getRemote()->write(msgBuf, payloadLength);
    getRemote()->enableReceive();
    if (report) {
      log("Message was successfully transmitted");
      msg_a = millis();
    } else {
      log("No ack from receiver");
    }

    if (!recentMessage()) {
      noPackageAction();
      log("No recent message");
    }
    if (getRemote()->read() >
        0) { // is there a payload? get the pipe number that recieved it
      const uint8_t *o = getRemote()->getBuf();
      if (o[0] == STATUS_RECEIVER) {
        log("Recv 1");
        memcpy(senderPayload, o + 2, sizeof(*senderPayload));
      } else {
        log(F("Recv 2"));
      }
    } else {
      log("Recv 0");
    }
    return report;
  }