
#ifndef COMMUNICATION_PROVIDER_HPP_
#define COMMUNICATION_PROVIDER_HPP_
#define PAYLOAD_LENGTH 35

typedef enum
{
  PACKAGE = 0x04,
  GYRO_SETUP = 0x05,
  CONTROL = 0x06,
  HOVER = 0x07,
  STATUS_SPEEDS = 0x80,
  STATUS_ORIENTATION = 0x81,
  STATUS_POSITION = 0x82,
  STATUS_HEIGHT = 0x83,
  STATUS_VOLTAGE = 0x84,
  ERROR = 0xE0,
  IDLE = 0xE1
} MessageType;

class Message
{
public:
  Message() {}
  Message(uint8_t msg, uint8_t length, uint8_t *data) : msg(msg), length(length)
  {
    memcpy(this->data, data, length);
  }
  void init(uint8_t msg, uint8_t length, uint8_t *data)
  {
    if (length > PAYLOAD_LENGTH - 2)
    {
      // handle error
      length = PAYLOAD_LENGTH - 2;
    }
    this->msg = msg;
    this->length = length;
    memcpy(this->data, data, length);
  }
  int buildBuf(uint8_t *buf)
  {
    buf[0] = msg;
    buf[1] = length;
    memcpy(buf + 2, data, length);
    return 2 + length;
  }
  uint8_t getMsg()
  {
    return msg;
  }
  uint8_t getLength()
  {
    return length;
  }
  uint8_t *getData()
  {
    return data;
  }

private:
  uint8_t msg;
  uint8_t length;
  uint8_t data[PAYLOAD_LENGTH - 2];
};

class MessageHandler
{
public:
  int buildPackage(Message *messages, int count, uint8_t *buf)
  {
    int index = 0;
    for (int i = 0; i < count; i++)
    {
      index += messages[i].buildBuf(buf + index);
    }
    buf[0] = PACKAGE;
    buf[1] = index;
    return index + 2;
  }

  void parseMessage(Message *message, uint8_t *buf)
  {
    message->init(buf[0], buf[1], buf + 2);
  }

  int parsePackage(uint8_t *buf, Message *message, int maxMessages)
  {
    uint8_t msg = buf[0];
    uint8_t length = buf[1];
    uint8_t *p = buf + 2;
    uint8_t *end = buf + 2 + length;
    int i = 0;
    if (end >= buf + PAYLOAD_LENGTH)
    {
      return -1;
    }
    if (msg == PACKAGE)
    {
      while (p < end)
      {
        uint8_t subMsg = p[0];
        uint8_t subLeng = p[1];
        uint8_t *data = p + 2;
        if (subLeng > PAYLOAD_LENGTH - 2)
        {
          // invalid message, handle error
          return -1;
        }
        if (i >= maxMessages)
        {
          // too many messages, handle error
          return -1;
        }
        if (p + 2 + subLeng > end)
        {
          // invalid message, handle error
          return -1;
        }
        message[i].init(subMsg, subLeng, data);
        p += 2 + subLeng;
        i++;
      }
    }
    return i;
  }

private:
};

// Communication provider class
class CommunicationProvider
{
public:
  virtual int read() = 0;
  virtual bool write(void *buf, uint8_t len) = 0;
  virtual int setEndpoint(const uint8_t *address) = 0;
  virtual int setRead(const uint8_t *address) = 0;
  virtual int enableReceive() = 0;
  virtual int disableReceive() = 0;
  uint8_t *getBuf()
  {
    return msgBuf;
  }

protected:
  uint8_t msgBuf[PAYLOAD_LENGTH];
};

// Mock provider for testing without hardware
class RemoteMock : public CommunicationProvider
{
  int read() override { return 0; }
};
#endif