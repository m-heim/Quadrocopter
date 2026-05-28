#include "mcapp.hpp"
#include "WString.h"

int MCAppReceiver::handle()
{
    msg_n += 1;
    int msgs = -1;
    int l = remote.read();
    if (l > 0)
    { // is there a payload? get the pipe number that recieved it
        uint8_t *o = remote.getBuf();
        /*for (int i = 0; i < l; i++)
        {
            logger.log(String(o[i], HEX).c_str());
        }*/
        logger.log("R.R");
        if (o[0] == PACKAGE)
        {
            msgs = messageHandler.parsePackage(o, messages, MESSAGES);
            if (msgs >= 1)
            {
                logger.log(FLASH_STRING("R.R 1"));
            }
            else
            {
                logger.log(FLASH_STRING("R.R Err"));
            }
        }
        else
        {
            logger.log(FLASH_STRING("R.R Err"));
        }
    }
    if (!recentMessage())
    {
        noPackageAction();
        logger.log(FLASH_STRING("R.R.R 0"));
    }
    else
    {
        logger.log(FLASH_STRING("R.R.R 1"));
    }
    if ((msg_n % 18) == 0)
    {
        Message voltageMessage;
        float voltage = voltageHandler.getVoltage();
        voltageMessage.init(STATUS_VOLTAGE, sizeof(float), (uint8_t *)&voltage);
        Message orientationMessage;
        float orientation[2] = {0.0, 0.0};
        orientationMessage.init(STATUS_ORIENTATION, sizeof(orientation), (uint8_t *)orientation);
        Message eventMessage;
        uint8_t event[1] = {0};
        eventMessage.init(STATUS_EVENT, sizeof(event), event);
        Message msgs[] = {voltageMessage, orientationMessage, eventMessage};
        int payloadLength = messageHandler.buildPackage(msgs, sizeof(msgs) / sizeof(msgs[0]), msgBuf);
        getRemote().disableReceive();
        bool report = remote.write(msgBuf, payloadLength);
        getRemote().enableReceive();
        if (report)
        {
            logger.log(FLASH_STRING("R.S 1"));
        }
        else
        {
            logger.log("R.S 0");
        }
    }
    return msgs;
}

int MCAppSender::handle(InputPayload &p)
{
    logger.log(FLASH_STRING("S.H"));
    msg_n += 1;
    getRemote().disableReceive();
    for (int i = 0; i < p.getLen(); i++)
    {
        logger.log(String(p.getBuf()[i], HEX).c_str());
    }
    bool report = getRemote().write(p.getBuf(), p.getLen());
    getRemote().enableReceive();
    if (report)
    {
        logger.log(FLASH_STRING("S.S 1"));
        timer.start();
    }
    else
    {
        logger.log(FLASH_STRING("S.S 0"));
    }
    if (!recentMessage())
    {
        noPackageAction();
        logger.log(FLASH_STRING("S.S.Err 1"));
    }
    if (getRemote().read() > 0)
    { // is there a payload? get the pipe number that recieved it
        logger.log(FLASH_STRING("S.R 1"));
        uint8_t *o = getRemote().getBuf();
        int msgs = messageHandler.parsePackage(o, messages, MESSAGES);
        for (int i = 0; i < msgs; i++)
        {
            char buf[32];
            snprintf(buf, sizeof(buf), "%d %d %s", messages[i].getMsg(), messages[i].getLength(), messages[i].getData());
            logger.log(buf);
            if (messages[i].getMsg() == STATUS_EVENT)
            {
                if (!(*messages[i].getData()))
                {
                    logger.log(FLASH_STRING("S.R.E 1"));
                }
                else
                {
                    logger.log(FLASH_STRING("S.R.E 0"));
                }
            }
            if (messages[i].getMsg() == STATUS_VOLTAGE)
            {
                float voltage = *((float *)messages[i].getData());
                logger.log(FLASH_STRING("S.R.V:"));
                logger.log(String(voltage).c_str());
            }
            /*else if (messages[i].getMsg() == STATUS_SPEEDS)
            {
                logger->log(FLASH_STRING("S.R.Speeds:"));
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
                logger->log(FLASH_STRING("Sender.Received.Height:"));
                logger->log(String(height).c_str());
            }
            else if (messages[i].getMsg() == STATUS_ORIENTATION)
            {
                float pitch = *((float *)messages[i].getData());
                float roll = *((float *)(messages[i].getData() + 4));
                logger->log(FLASH_STRING("Sender.Received.Orientation:"));
                char buf[40];
                snprintf(buf, 40, "Pitch: %f Roll: %f", pitch, roll);
                logger->log(buf);
            }
            else if (messages[i].getMsg() == STATUS_POSITION)
            {
                float x = *((float *)messages[i].getData());
                float y = *((float *)(messages[i].getData() + 4));
                logger->log(FLASH_STRING("Sender.Received.Position:"));
                char buf[40];
                snprintf(buf, 40, "X: %f Y: %f", x, y);
                logger->log(buf);
            }
            else
            {
                logger->log(FLASH_STRING("S.R.Unknown:"));
                char buf[20];
                snprintf(buf, 20, "Msg: %d Len: %d", messages[i].getMsg(), messages[i].getLength());
                logger->log(buf);
            }*/
        }
    }
    else
    {
        logger.log(FLASH_STRING("S.R 0"));
    }
    return report;
}
