#include <Arduino.h>
#include <Servo.h>
#include "nrf24l01_provider.hpp"
#include "mcapp.hpp"
#include "utils.hpp"
#include <ADXL345_WE.h>

#define SENDER 0
#define PIEZO 4
#define LED 5
#define INVOLTAGE A7
#define RECEIVER_SLEEP 20
#define SENDER_SLEEP 40
#define NO_MSG 450

#if SENDER == 1
#define CE_PIN 7
#define CSN_PIN 8
#else
#define CE_PIN 7
#define CSN_PIN 8
#endif

#define FRONT 0x00
#define BACK 0x02
#define LEFT 0x00
#define RIGHT 0x01

#define FRONT_LEFT FRONT | LEFT
#define FRONT_RIGHT FRONT | RIGHT
#define BACK_LEFT BAcK | LEFT
#define BACK_RIGHT BACK | RIGHT


uint8_t msgBuf[PAYLOAD_LENGTH];
uint8_t payloadLength = 0;

int freq = FREQ_DEFAULT;

long msg_a = 0;

int servoPins[4] = {A0, A1, A2, A3};
Servo servos[4];

bool isConnected = false;

uint8_t address[][6] = {"Send1", "Recv1"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination
int radioNumber = SENDER; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

NRF24L01Provider radio = NRF24L01Provider(CE_PIN, CSN_PIN);
MCApp app = MCApp();

#if SENDER == 0
ADXL345_WE a = ADXL345_WE();
#endif

int8_t val = 0;

ReceiverPayload payload;

xyzFloat gyro;

void setSpeeds(ReceiverPayload p) {
  float speeds[4];
  double frontWeight = ((gyro.x - (-10)) * (127.0/90.0) / 4);
  double leftWeight = ((gyro.y - 4) * (127.0/90.0) / 4);
  if (abs(frontWeight) < 4) {
    frontWeight = 0;
  }
  if (abs(leftWeight) < 4) {
    leftWeight = 0;
  }
  for (int i = 0; i < 4; i++)
  {
    speeds[i] = p.speed;
    if (i & BACK) {
      speeds[i] -= frontWeight;
    } else {
      speeds[i] += frontWeight;
    }
    if (i & RIGHT) {
      speeds[i] -= leftWeight;
    }
    else {
      speeds[i] += leftWeight;
    }
  }
  for (int i = 0; i < 4; i++)
  {
    int v = 1000;
    float s = speeds[i];
    s = s / 127;
    v += 1000 * s;
    if (v > 2000) {
      v = 2000;
    }
    if (v < 1000) {
      v = 1000;
    }
    servos[i].writeMicroseconds(v);
  }
  char buf[189];
  sprintf(buf, "%d %d %d %d %d %d %d", (int) gyro.x, (int) gyro.y, (int) gyro.z, (int) speeds[0], (int) speeds[1], (int) speeds[2], (int) speeds[3]);
  app.log(buf);
}

void noPackageAction() {
  payload.speed = 0;
  app.buzz(freq, 10);
  app.setLed(1);
  freq += 400;
  if (freq > 4000)
  {
    freq = FREQ_BASE;
  }
}

int getXRotation() {

}

void setup()
{
  payload.speed = 0;
  payload.pitch = 0;
  payload.roll = 0;
  payload.yaw = 0;
  pinMode(LED_BUILTIN, OUTPUT);
  app.initLog(9600);
  // put your setup code here, to run once:
  if (!radio.init())
  {
    app.log("Device is not responding");
    app.ledError();
  }
  else
  {
    app.log("Nrf radio successfully connected");
  }

#if SENDER == 0
  app.initPiezo(PIEZO);
  app.setNoPiezo(true);
  app.initVoltage(INVOLTAGE, 3);
  app.initLed(LED);
  Wire.begin();
  if (!a.init()) {
    app.log("Accelerometer is not working properly");
    app.ledError();
  }
  a.measureAngleOffsets();
  a.setDataRate(ADXL343_DATA_RATE_800);
  for (int i = 0; i < 4; i++)
  {
    app.buzz(1000 + i * 100, 40);
    delay(40);
  }
  pinMode(10, OUTPUT); // set pin 10 for output, necessary for spi
  radio.getRadio().openReadingPipe(1, address[0]);
  radio.getRadio().startListening();
  delay(400);
  for (int i = 0; i < 4; i++)
  {
    servos[i].attach(servoPins[i]);
    servos[i].writeMicroseconds(1000);
  }
/*  for (int i = 0; i < 4; i++)
  {
    servos[i].writeMicroseconds(1000);
  }*/
 delay(1000);
#else
  app.log("Setting up radio for sender");
  pinMode(10, OUTPUT); // set pin 10 for output, necessary for spi
  radio.getRadio().openWritingPipe(address[0]);
  radio.getRadio().stopListening();
  app.log("Radio setup for sender");
#endif
}
void loop()
{
  bool hasPackage = false;
  int action = 1;
#if SENDER == 1
  int8_t uartData[4] = {0, 0, 0, 0};
  if (Serial) // read from uart
  {
    Serial.setTimeout(10);
    int s = Serial.readBytesUntil('\n', (char *)uartData, 4);
    if (s <= 1)
    {
      app.log("No action");
    }
    else if (s >= 2 && uartData[0] == 'c')
    {
      val = uartData[1];
      app.log("Got speed change");
    }
    else if (s >= 2 && uartData[0] == 's') {
      app.log("God steer change");
    } else
    {
      app.log("Unknown action");
    }
  }
  if (!isConnected)
  {
    msgBuf[0] = HELLO;
    msgBuf[1] = 0;
    payloadLength = 2;
    app.log("Sending hello");
  }
  else if (action == 1)
  {
    msgBuf[0] = CONTROL;
    msgBuf[1] = 4;
    payloadLength = 2 + sizeof(ReceiverPayload);
    ReceiverPayload p;
    p.speed = val;
    p.pitch = 0;
    p.yaw = 0;
    p.roll = 0;
    memcpy(msgBuf + 2, &p, sizeof(p));
    app.log("Sending control");
    app.log("Speed");
    char buf[45];
    itoa(p.speed, buf, 10);
    app.log(buf);
  }
  else
  {
    app.log("Unknown action");
  }
  bool report = radio.getRadio().write(&msgBuf, payloadLength);
  if (report)
  {
    app.log("Message was successfully transmitted");
    isConnected = true;
  }
  else
  {
    app.log("No ack from receiver");
    isConnected = false;
  }
  delay(SENDER_SLEEP);
#else
  app.setLed(0);
  if (!a.getAngles(&gyro)) {
    app.log("Accelerometer is not working");
    gyro.x = 0;
    gyro.y = 0;
    gyro.z = 0;
  }
  if (radio.getRadio().available())
  { // is there a payload? get the pipe number that recieved it
    hasPackage = true;
    uint8_t bytes = radio.getRadio().getPayloadSize(); // get the size of the payload
    radio.getRadio().read(&msgBuf, bytes);             // fetch payload from FIFO

    if (msgBuf[0] == HELLO)
    {
      isConnected = true;
      app.log("Hello from sender");
      for (int i = 0; i < 4; i++)
      {
        app.buzz((i + 1) * 400, 40);
        delay(40);
      }
    }
    else if (msgBuf[0] == BYE)
    {
      isConnected = false;
      app.log("Bye from sender");
      for (int i = 3; i >= 0; i--)
      {
        app.buzz((i + 1) * 400, 40);
        delay(40);
      }
    }
    else if (msgBuf[0] == CONTROL)
    {
      memcpy(&payload, msgBuf + 2, sizeof(payload));
      msg_a = millis();
      if (Serial)
      {
        app.log("Payload from sender: ");
        Serial.print(payload.speed);
        Serial.print(payload.yaw);
        Serial.print(payload.pitch);
        Serial.print(payload.roll);
        Serial.print("\n");
      }
    }
    else
    {
      app.log("Invalid message");
    }
  }
  else
  {
    app.log("Radio not available");
  }
/**  if (Serial)
  {
    Serial.print("Voltage: ");
    Serial.print((int)app.getVoltage(), DEC);
    Serial.print("V");
    Serial.print("\n");
  }**/
  if (((millis() - msg_a) > NO_MSG))
  {
    noPackageAction();
  }
  else
  {
  }
  setSpeeds(payload);
  delay(RECEIVER_SLEEP);
#endif
}
