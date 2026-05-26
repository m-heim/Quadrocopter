#include "mcapp.hpp"
#include "WString.h"

int MCApp::handle(SenderPayload &p)
{
    msg_n += 1;
    int msgs = -1;
    int l = remote->read();
    if (l > 0)
    { // is there a payload? get the pipe number that recieved it
        uint8_t *o = remote->getBuf();
        logger->log("Receiver.Received");
        if (o[0] == PACKAGE)
        {
            msgs = messageHandler.parsePackage(o, messages, MESSAGES);
            if (msgs >= 1)
            {
                timer.start();
            }
            logger->log("Receiver.Received 1");
        }
        else
        {
            logger->log("Receiver.Received Err");
        }
        if (timer.isExpired())
        {
            noPackageAction();
        }
        else
        {
        }
        if ((msg_n % 10) == 0)
        {
            Message voltageMessage;
            float voltage = getVoltage();
            voltageMessage.init(STATUS_VOLTAGE, sizeof(float), (uint8_t *)&p.voltage);
            Message orientationMessage;
            float orientation[2] = {p.orientation[0], p.orientation[1]};
            orientationMessage.init(STATUS_ORIENTATION, sizeof(orientation), (uint8_t *)orientation);
            Message msgs[2] = {voltageMessage, orientationMessage};
            int payloadLength = messageHandler.buildPackage(msgs, sizeof(msgs) / sizeof(msgs[0]), msgBuf);
            getRemote()->disableReceive();
            bool report = remote->write(msgBuf, payloadLength);
            getRemote()->enableReceive();
            if (report)
            {
                logger->log("Receiver.Send 1");
            }
            else
            {
                logger->log("Receiver.Send 0");
            }
        }
    }
    else
    {
        logger->log("Receiver.Received 0");
    }
    return msgs;
}

bool MCApp::handle2(const QuadrocopterMessage &p, bool valid)
{
    logger->log("Sender.Handle");
    msg_n += 1;
    Message gyroSetupMessage;
    Message speedMessage;
    uint8_t gyroSetupMessageBuf[1];
    gyroSetupMessageBuf[0] = p.gyroSetup ? (uint8_t)'1' : (uint8_t)'0';
    gyroSetupMessage.init(GYRO_SETUP, 1, gyroSetupMessageBuf);
    uint8_t speedMessageBuf[4];
    if (valid)
    {
        memcpy(speedMessageBuf, p.speeds, 4);
    }
    else
    {
        memset(speedMessageBuf, 0, 4);
    }
    speedMessage.init(CONTROL, 4, speedMessageBuf);
    Message msgs[2] = {gyroSetupMessage, speedMessage};
    int payloadLength = messageHandler.buildPackage(msgs, sizeof(msgs) / sizeof(msgs[0]), msgBuf);
    getRemote()->disableReceive();
    bool report = getRemote()->write(msgBuf, payloadLength);
    getRemote()->enableReceive();
    if (report)
    {
        logger->log("Sender.Send 1");
        timer.start();
    }
    else
    {
        logger->log("Sender.Send 0");
    }
    if (timer.isExpired())
    {
        noPackageAction();
        logger->log("Sender.Send.Err 1");
    }
    if (getRemote()->read() > 0)
    { // is there a payload? get the pipe number that recieved it
        logger->log("Sender.Received 1");
        uint8_t *o = getRemote()->getBuf();
        int msgs = messageHandler.parsePackage(o, messages, MESSAGES);
        for (int i = 0; i < msgs; i++)
        {
            if (messages[i].getMsg() == STATUS_VOLTAGE)
            {
                float voltage = *((float *)messages[i].getData());
                logger->log("Sender.Received.Voltage:");
                logger->log(String(voltage).c_str());
            }
            /*else if (messages[i].getMsg() == STATUS_SPEEDS)
            {
                logger->log("Sender.Received.Speeds:");
                char buf[20];
                for (int j = 0; j < 4; j++)
                {
                    snprintf(buf + j * 5, 5, "%d ", (int8_t)messages[i].getData()[j]);
                }
                logger->log(buf);
            }
            else if (messages[i].getMsg() == STATUS_HEIGHT)
            {
                float height = *((float *)messages[i].getData());
                logger->log("Sender.Received.Height:");
                logger->log(String(height).c_str());
            }
            else if (messages[i].getMsg() == STATUS_ORIENTATION)
            {
                float pitch = *((float *)messages[i].getData());
                float roll = *((float *)(messages[i].getData() + 4));
                logger->log("Sender.Received.Orientation:");
                char buf[40];
                snprintf(buf, 40, "Pitch: %f Roll: %f", pitch, roll);
                logger->log(buf);
            }
            else if (messages[i].getMsg() == STATUS_POSITION)
            {
                float x = *((float *)messages[i].getData());
                float y = *((float *)(messages[i].getData() + 4));
                logger->log("Sender.Received.Position:");
                char buf[40];
                snprintf(buf, 40, "X: %f Y: %f", x, y);
                logger->log(buf);
            }
            else
            {
                logger->log("Sender.Received.Unknown:");
                char buf[20];
                snprintf(buf, 20, "Msg: %d Len: %d", messages[i].getMsg(), messages[i].getLength());
                logger->log(buf);
            }*/
        }
    }
    else
    {
        logger->log("Sender.Received 0");
    }
    return report;
}
