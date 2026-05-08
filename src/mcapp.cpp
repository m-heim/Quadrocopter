#include "mcapp.hpp"

    bool MCApp::handle()
    {
        bool valid = false;
        if (remote->read() > 0)
        { // is there a payload? get the pipe number that recieved it
            valid = true;
            const uint8_t *buf = remote->getBuf();
            // setup gyro
            if (buf[0] == GYRO_SETUP)
            {
                log("Received gyro setup");
                // setGravity();
            }
            // setup motor
            else if (buf[0] == MOTOR_SETUP)
            {
                log("Received motor setup");
            }
            // control message
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
        }
        else
        {
        }
        return valid;
    }

bool MCApp::handle2(int8_t *buf, bool gyroSetup)
    {
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
            msg_a = millis();
        }
        else
        {
            log("No ack from receiver");
        }
        if (!recentMessage())
        {
            log("No recent message");
        }
        return report;
    }