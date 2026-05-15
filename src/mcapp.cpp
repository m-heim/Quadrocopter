#include "mcapp.hpp"
#include "WString.h"

    bool MCApp::handle(SenderPayload &p)
    {
        msg_n += 1;
        bool valid = false;
        if (remote->read() > 0)
        { // is there a payload? get the pipe number that recieved it
            valid = true;
            const uint8_t *o = remote->getBuf();
            // setup gyro
            if (o[0] == GYRO_SETUP)
            {
                log("Received gyro setup");
                // setGravity();
            }
            // setup motor
            else if (o[0] == MOTOR_SETUP)
            {
                log("Received motor setup");
            }
            // control message
            else if (o[0] == CONTROL)
            {
                log("Received payload");
                memcpy(&payload, o + 2, sizeof(payload));
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
            log(F("Recv 0"));
        }
        if (!recentMessage())
        {
            noPackageAction();
            valid = false;
        }
        else
        {
        }
        if ((msg_n % 10) == 0) {
            uint8_t buf[PAYLOAD_LENGTH];
            buf[0] = STATUS_RECEIVER;
            buf[1] = sizeof(p);
            memcpy(buf + 2, &p, sizeof(p));
            getRemote()->disableReceive();
            bool report = remote->write(buf, sizeof(p) + 2);
            getRemote()->enableReceive();
            if(report) {
                log(F("Send 1"));
            }
            else {
                log(F("Send 0"));
            }
        }
        return valid;
    }

bool MCApp::handle2(int8_t *buf, bool error, bool gyroSetup, SenderPayload *senderPayload)
    {
        msg_n += 1;
        int payloadLength = 0;
        int action = 1;
        if (gyroSetup) {
            msgBuf[0] = GYRO_SETUP;
            msgBuf[1] = 0;
            payloadLength = 2;
            log("Sending setup");
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
            if (error) {
                payload.speed = 0;
                payload.pitch = 0;
                payload.yaw = 0;
                payload.roll = 0;
            }
            memcpy(msgBuf + 2, &payload, sizeof(payload));
            log("Sending");
        }
        else
        {
            log("Unknown action");
        }
        getRemote()->disableReceive();
        bool report = getRemote()->write(msgBuf, payloadLength);
        getRemote()->enableReceive();
        if (report)
        {
            log("Message was successfully transmitted");
            msg_a = millis();
        }
        else
        {
            log("No ack from receiver");
        }
        if (!recentMessage())
        {
            noPackageAction();
            log("No recent message");
        }
        if (getRemote()->read() > 0)
        { // is there a payload? get the pipe number that recieved it
            const uint8_t *o = getRemote()->getBuf();
            if (o[0] == STATUS_RECEIVER)
            {
                log("Recv 1");
                memcpy(senderPayload, o + 2, sizeof(*senderPayload));
            } else {
                log(F("Recv 2"));
            }
        }
        else
        {
            log("Recv 0");
        }
        return report;
    }