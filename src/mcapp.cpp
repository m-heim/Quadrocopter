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
    if ((msg_n % 10) == 0)
    {
        bool report = command(STATUS_RECEIVER, (uint8_t *)&p, sizeof(p));
        if (report)
        {
            log(F("Send 1"));
        }
        else
        {
            log(F("Send 0"));
        }
    }
    return valid;
}

bool MCApp::handleInput()
{
    bool gyroSetup = false;
    int8_t uartData[35] = {0};
    if (Serial) // read from uart
    {
        String s = Serial.readStringUntil('\n');
        if (s.length() < 1) // if we didn't get any data, do nothing (instead of applying old data or random data)
        {
            log("No action");
        }
        else if (s.length() >= (1 + 4 + 3) && s.charAt(0) == 'c') // we have a speed change command, it should be in the format c<speed0><speed1><speed2><speed3>\n, where speeds are between -127 and 127
        {
            int index = 1;
            for (int i = 0; i < 4; i++)
            {
                String numStr = s.substring(index, s.indexOf(',', index));
                if (numStr.length() >= 1)
                {
                    vals[i] = (int8_t)numStr.toInt();
                }
                else
                {
                    vals[i] = 0;
                }
                index += numStr.length() + 1;
            }
            log("Got speed change");
            msg_b = millis();
        }
        else if (s.charAt(0) == 's') // we have a gyro setup command, it should be just s\n
        {
            log("Sending setup");
            gyroSetup = true;
        }
    }
    handle2(vals, (millis() - msg_b) > NO_MSG, gyroSetup, &senderPayload);
    char buf[35];
    sprintf(buf, "%d %d", (int)senderPayload.position2[0] / 127 * 180, (int)senderPayload.position2[1] / 127 * 180);
    log(buf);
    delay(SENDER_SLEEP);
    return true;
}

bool MCApp::startupSender()
{
    log("S.Sup.Init");
    msg_b = millis() - NO_MSG - 1; // set last message time to a while ago so that it doesn't fail if we don't get a message right away
    Serial.setTimeout(100); // 100 ms timeout for serial read, adjust if needed
    remote->setEndpoint(address[0]);
    remote->setRead(address[1]);
    remote->disableReceive();
    log("S.Sup.Init 1");
    return true;
}

bool MCApp::handle2(int8_t *buf, bool error, bool gyroSetup, SenderPayload *senderPayload)
{
    msg_n += 1;
    int payloadLength = 0;
    int action = 1;
    if (gyroSetup)
    {
        msgBuf[0] = GYRO_SETUP;
        msgBuf[1] = 0;
        payloadLength = 2;
        command(GYRO_SETUP, nullptr, 0);
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
        if (error)
        {
            payload.speed = 0;
            payload.pitch = 0;
            payload.yaw = 0;
            payload.roll = 0;
        }
        command(CONTROL, (uint8_t *)&payload, sizeof(payload));
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
        }
        else
        {
            log(F("Recv 2"));
        }
    }
    else
    {
        log("Recv 0");
    }
    return report;
}