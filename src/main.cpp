#include "init.hpp"
#include "mcapp.hpp"
#include "nrf24l01_provider.hpp"
#include "pid.hpp"
#include "utils.hpp"
#include <Arduino.h>
#if SENDER == 0
#if VEHICLE == 1
#include "vehicle.hpp"
#endif
#if VEHICLE == 0
#include "quadrocopter.hpp"
#endif
#else

// It is very helpful to think of an address as a path instead of as
// an identifying device destination
uint8_t radioNumber =
    SENDER; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
NRF24L01Provider radio = NRF24L01Provider(CE_PIN, CSN_PIN);
ArduinoLogger logger(DEBUG, BAUD);
Buzzer buzzer = Buzzer(NO_PIN);
Led led = Led(NO_PIN);
VoltageHandler voltageHandler(NO_PIN, 0, 0);
MCAppSender app(&radio, &logger, &buzzer, &led, &voltageHandler);
UartInputHandler uartInputHandler{};

uint8_t defaultBuf[] = {0x04, 0x06, 0x06, 0x04, 0x00, 0x00, 0x00, 0x00};
InputPayload inputPayload = InputPayload((uint8_t *)defaultBuf, sizeof(defaultBuf));

void startupSender()
{
  radio.getRadio().openWritingPipe(address[0]);
  radio.getRadio().openReadingPipe(1, address[1]);
  radio.getRadio().startListening();
}

#endif

void setup()
{
  app.logger->log(FLASH_STRING("Init"));
  delay(100);
  // put your setup code here, to run once:
  if (!radio.init())
  {
    app.logger->log(FLASH_STRING("Init.RF24 0"));
    app.led->ledError();
  }
  else
  {
    app.logger->log(FLASH_STRING("Init.RF24 1"));
  }
#if SENDER == 0
  startup();
#endif
#if SENDER == 1
  startupSender();
#endif
  app.logger->log(FLASH_STRING("Init 1"));
}

void handleSender()
{
  bool valid = uartInputHandler.handle(inputPayload);
  if (valid)
  {
    app.logger->log(FLASH_STRING("Sender.Uart.Received 1"));
    app.logger->log(FLASH_STRING("Sender.Uart.Received:"));
    app.logger->log(String(inputPayload.getLen()).c_str());
    app.logger->log((char *)inputPayload.getBuf());
  }
  else
  {
    app.logger->log(FLASH_STRING("Sender.Uart.Received 0"));
  }
  app.handle(inputPayload);
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
    app.logger->log(FLASH_STRING("Receiver.LowVoltage 1"));
  }

  delay(RECEIVER_SLEEP);
#endif
}
