
#ifndef COMMUNICATION_PROVIDER_HPP_
#define COMMUNICATION_PROVIDER_HPP_
#define PAYLOAD_LENGTH 24

enum
{
    HELLO = 10,
    BYE,
    CONFIG,
    CONFIG_ACK,
    CONTROL,
    STATUS_SENDER,
    STATUS_RECEIVER,
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

class CommunicationProvider
{
};
#endif