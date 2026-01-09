
#ifndef COMMUNICATION_PROVIDER_HPP_
#define COMMUNICATION_PROVIDER_HPP_
#define PAYLOAD_LENGTH 24

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
    int8_t speeds[4];
    float position1[3];
    float position2[3];
    float voltage;

};

class CommunicationProvider
{
};
#endif