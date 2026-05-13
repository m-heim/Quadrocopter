
#ifndef COMMUNICATION_PROVIDER_HPP_
#define COMMUNICATION_PROVIDER_HPP_
#define PAYLOAD_LENGTH 35

enum
{
    IDLE = 20,
    OK,
    ERROR,
    BATTERY
} Status;

enum
{
    HELLO = 10,
    BYE,
    CONFIG,
    CONFIG_ACK,
    CONTROL,
    STATUS_SENDER,
    STATUS_RECEIVER,
    GYRO_SETUP,
    MOTOR_SETUP
} MessageType;

struct Message
{
    uint8_t msg;
    uint8_t length;
    uint8_t data[PAYLOAD_LENGTH - 2];
};

struct ReceiverPayload
{
    int8_t speed;
    int8_t pitch;
    int8_t roll;
    int8_t yaw;
};

struct SenderPayload {
    int8_t speeds[4]; // speed, pitch, roll, yaw
    int32_t position1[3]; // x, y, z
    int8_t position2[3]; // rotation x, rotation y, rotation z
    uint8_t voltage; // voltage level volage * 10
};

// Communication provider class
class CommunicationProvider
{
    public:
    virtual int read() = 0;
    virtual bool write(void *buf, uint8_t len) = 0;
    virtual int enableReceive() = 0;
    virtual int disableReceive() = 0;
    const uint8_t *getBuf() {
        return msgBuf;
    }
    protected:
    uint8_t msgBuf[PAYLOAD_LENGTH];
};



// Mock provider for testing without hardware
class RemoteMock : public CommunicationProvider {
    int read() override { return 0;}
};
#endif