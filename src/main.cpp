#include <Arduino.h>
#include <Servo.h>
#include "nrf24l01_provider.hpp"
#include "mcapp.hpp"
#define PIEZO 4
#define LED 5
#define INVOLTAGE A7
#define SENDER 1

#if SENDER == 1
#define CE_PIN 7
#define CSN_PIN 8
#else
#define CE_PIN 7
#define CSN_PIN 8
#endif

#define FREQ_BASE 400
#define FREQ_DEFAULT 1000
#define FREQ_HIGH 10000

uint8_t msgBuf[PAYLOAD_LENGTH];
uint8_t payloadLength = 0;

int freq = FREQ_DEFAULT;

int servoPins[4] = {A0, A1, A2, A3};
Servo servos[4];

bool isConnected = false;

uint8_t address[][6] = {"Send1", "Recv1"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination
int radioNumber = SENDER; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

NRF24L01Provider radio = NRF24L01Provider(CE_PIN, CSN_PIN);
MCApp app = MCApp();

int8_t val = 0;

void setup()
{
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
  app.initVoltage(INVOLTAGE, 3);
  app.initLed(LED);
  for (int i = 0; i < 4; i++)
  {
    tone(PIEZO, 1000 + i * 100, 40);
    delay(40);
  }
  pinMode(10, OUTPUT); // set pin 10 for output, necessary for spi
  radio.getRadio().openReadingPipe(1, address[0]);
  radio.getRadio().startListening();
  for (int i = 0; i < 4; i++)
  {
    servos[i].attach(servoPins[i]);
  }
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
  delay(100);
#else
  app.setLed(0);
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
        tone(PIEZO, (i + 1) * 400, 100);
        delay(100);
      }
    }
    else if (msgBuf[0] == BYE)
    {
      isConnected = false;
      app.log("Bye from sender");
      for (int i = 3; i >= 0; i--)
      {
        tone(PIEZO, (i + 1) * 400, 100);
        delay(100);
      }
    }
    else if (msgBuf[0] == CONTROL)
    {
      ReceiverPayload payload;
      memcpy(&payload, msgBuf + 2, sizeof(payload));
      uint8_t speeds[4];
      for (int i = 0; i < 4; i++)
      {
        speeds[i] = payload.speed;
      }
      for (int i = 0; i < 4; i++)
      {
        int v = 1000;
        v += 7 * speeds[i];
        if (v > 2000) {
          v = 2000;
        }
        servos[i].writeMicroseconds(v);
      }
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
      app.setLed(1);
    }
  }
  else
  {
    app.log("Radio not available");
    app.setLed(1);
  }
  if (Serial)
  {
    Serial.print("Voltage: ");
    Serial.print((int)app.getVoltage(), DEC);
    Serial.print("V");
    Serial.print("\n");
  }
  if (!hasPackage)
  {
    app.buzz(freq, 10);
    app.ledToggle();
    delay(100);
    freq += 1000;
    if (freq > 4000)
    {
      freq = FREQ_BASE;
    }
  }
  else
  {
  }
  delay(40);
#endif
}
