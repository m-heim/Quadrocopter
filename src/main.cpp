#include "init.hpp"
#include "mcapp.hpp"
#include "nrf24l01_provider.hpp"
#include "pid.hpp"
#include "utils.hpp"
#include "vehicle.hpp"
#include <Adafruit_MPU6050.h>
#include <Arduino.h>
#include <Servo.h>
#if SENDER == 0
#if VEHICLE == 1
#include "vehicle.hpp"
#endif
#if VEHICLE == 0
#include "quadrocopter.hpp"
#endif
#endif

// It is very helpful to think of an address as a path instead of as
// an identifying device destination
uint8_t radioNumber =
    SENDER; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
NRF24L01Provider radio = NRF24L01Provider(CE_PIN, CSN_PIN);
MCApp app = MCApp(&radio);
UartInputHandler uartInputHandler{};

void startupSender()
{
  radio.getRadio().openWritingPipe(address[0]);
  radio.getRadio().openReadingPipe(1, address[1]);
  radio.getRadio().startListening();
}

void setup()
{
  app.initLog(115200);
  app.log(F("Init"));
  delay(100);
  // put your setup code here, to run once:
  if (!radio.init())
  {
    app.log(F("Init.RF24 0"));
    app.ledError();
  }
  else
  {
    app.log(F("Init.RF24 1"));
  }
#if SENDER == 0
  startup();
#endif
#if SENDER == 1
  startupSender();
#endif
  app.log(F("Init 1"));
}

void handleSender()
{
  QuadrocopterMessage p;
  int msgs = uartInputHandler.handle(&app.messageHandler, app.messages);
  if (msgs > 0)
  {
    app.log(F("Sender.Uart.Received 1"));
    for (int i = 0; i < msgs; i++)
    {
      if (app.messages[i].getMsg() == CONTROL)
      {
        app.log(F("Sender.Control 1"));
        uint8_t *data = app.messages[i].getData();
        memcpy(p.speeds, data, 4);
      }
      else if (app.messages[i].getMsg() == GYRO_SETUP)
      {
        p.gyroSetup = app.messages[i].getData()[0] == '1';
      }
      else if (app.messages[i].getMsg() == HOVER)
      {
        p.hover = app.messages[i].getData()[0] == '1';
      }
      else
      {
        app.log(F("Sender.Uart.Received Err"));
      }
    }
  }
  else
  {
    app.log(F("Sender.Uart.Received 0"));
  }
  app.handle2(p, uartInputHandler.isRecent());
}
void loop()
{
#if SENDER == 1
  handleSender();
  delay(SENDER_SLEEP);
#else
  bool gyroApply = false;
  app.setLed(0);
  if (gyro)
  {
    a.getEvent(&am, &g, &temp);
    gyroApply = true;
  }
  SenderPayload senderPayload;
  senderPayload.orientation[0] = (int8_t)(alphaX / 180) * 127;
  senderPayload.orientation[1] = (int8_t)(alphaY / 180) * 127;
  senderPayload.orientation[2] = 0;
  senderPayload.voltage = app.getVoltage();
  int pkg = app.handle(senderPayload);
  int8_t *speedBuf = nullptr;
  int8_t speeds0[4] = {0, 0, 0, 0};
  for (int i = 0; i < pkg; i++)
  {
    if (app.messages[i].getMsg() == CONTROL)
    {
      speedBuf = (int8_t *)app.messages[i].getData();
    }
  }
  if (speedBuf == nullptr)
  {
    speedBuf = speeds0;
  }
  setSpeeds(speedBuf, app.recentMessage(), gyroApply);
  if (!app.verifyVoltage())
  {
    app.log(F("Receiver.LowVoltage 1"));
  }

  delay(RECEIVER_SLEEP);
#endif
}
