#include "mcapp.hpp"
#include "nrf24l01_provider.hpp"
#include "utils.hpp"
#include <Arduino.h>
#if SENDER == 0
#if VEHICLE == 1
#include "vehicle.hpp"
#endif
#if VEHICLE == 0
#include "quadrocopter.hpp"
#endif
#endif

NRF24L01Provider radio = NRF24L01Provider(CE_PIN, CSN_PIN);
Buzzer buzzer = Buzzer(BUZZER_PIN);
Led led = Led(LED_PIN);
VoltageHandler voltageHandler(VOLTAGE_PIN, VOLTAGE_FACTOR, VOLTAGE);
ArduinoLogger logger(DEBUG, BAUD);
// It is very helpful to think of an address as a path instead of as
// an identifying device destination
uint8_t radioNumber =
    SENDER; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
libmh::ArduinoTimeProvider arduinoProv;
libmh::TimeProvider<float> &prov = arduinoProv;

void startup();
#if SENDER == 0
MCAppReceiver app(radio, logger, buzzer, led, voltageHandler);
extern void startupVehicle();
void startup()
{
  app.logger.log(FLASH_STRING("SUP"));
  startupVehicle();
  radio.getRadio().openReadingPipe(1, address[0]);
  radio.getRadio().openWritingPipe(address[1]);
  radio.getRadio().startListening();
  delay(100);
  app.buzzer.output(1000, 1300, 100, 0.04); // buzz on startup to indicate it is on
  app.logger.log(FLASH_STRING("SUP 1"));
}
#else
uint8_t defaultBuf[] = {PACKAGE, 0x06, CONTROL, 0x04, 0x00, 0x00, 0x00, 0x00};
uint8_t defaultBufOutput[] = {PACKAGE, 0x02, STATUS_EVENT, 0x00};

MCAppSender app(radio, logger, buzzer, led, voltageHandler);
UartInputHandler uartInputHandler{};

InputPayload inputPayload = InputPayload((uint8_t *)defaultBuf, sizeof(defaultBuf));

void startup()
{
  app.logger.log(FLASH_STRING("S.RF.Init"));
  radio.getRadio().openWritingPipe(address[0]);
  radio.getRadio().openReadingPipe(1, address[1]);
  radio.getRadio().startListening();
  app.logger.log(FLASH_STRING("S.RF.Init 1"));
}

void handleSender()
{
  bool valid = uartInputHandler.handle(inputPayload);
  if (valid)
  {
    app.logger.log(FLASH_STRING("S.U.R 1"));
    app.logger.log(FLASH_STRING("S.U.R:"));
    app.logger.log(String(inputPayload.getLen()).c_str());
    app.logger.log((char *)inputPayload.getBuf());
  }
  else
  {
    app.logger.log(FLASH_STRING("S.U.R 0"));
  }
  app.handle(inputPayload);
}

#endif

void setup()
{
  logger.init();

  app.logger.log(FLASH_STRING("I"));
  delay(100);
  // put your setup code here, to run once:
  if (!radio.init())
  {
    app.logger.log(FLASH_STRING("I.RF24 0"));
    app.led.ledError();
  }
  else
  {
    app.logger.log(FLASH_STRING("I.RF24 1"));
  }
  startup();
  app.logger.log(FLASH_STRING("I 1"));
}

int8_t speedBuf[4];
void loop()
{
#if SENDER == 1
  handleSender();
  delay(SENDER_SLEEP);
#endif
#if SENDER == 0
  app.led.setLed(0);
  int pkg = app.handle();
  for (int i = 0; i < pkg; i++)
  {
    if (app.messages[i].getMsg() == CONTROL)
    {
      char buf[32];
      snprintf(buf, 32, "%d %d %d %d", app.messages[i].getData()[0], app.messages[i].getData()[1], app.messages[i].getData()[2], app.messages[i].getData()[3]);
      app.logger.log("Received control");
      app.logger.log(buf);
      memcpy(speedBuf, app.messages[i].getData(), 4);
      app.timer.start();
    }
  }
  setSpeeds(speedBuf, app.recentMessage());
  if (!app.voltageHandler.verifyVoltage())
  {
    app.logger.log(FLASH_STRING("R.V.Low"));
  }

  delay(RECEIVER_SLEEP);
#endif
}
