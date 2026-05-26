#include <Arduino.h>
#include <Servo.h>
#include "nrf24l01_provider.hpp"
#include "mcapp.hpp"
#include "utils.hpp"
#include "init.hpp"
#include "pid.hpp"
#if VEHICLE == 1
#include "vehicle.hpp"
#endif
#if VEHICLE == 0
#include "quadrocopter.hpp"
#endif

// It is very helpful to think of an address as a path instead of as
// an identifying device destination
uint8_t radioNumber = SENDER; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
SenderPayload payload2 = {0};

void setup()
{
  app.initLog(115200);
  // put your setup code here, to run once:
  if (!radio.init())
  {
    app.log("RF24 0");
    app.ledError();
  }
  else
  {
    app.log("RF24 1");
  }
#if SENDER == 0
  startup();
#endif
#if SENDER == 1
startupSender();
#endif
}
void loop()
{
  #if SENDER == 1
  handleSender();
#else
  bool gyroApply = false;
  app.setLed(0);
  if (gyro)
  {
    a.getEvent(&am, &g, &temp);
    gyroApply = true;
  }
  SenderPayload senderPayload;
  senderPayload.position2[0] = (int8_t) (alphaX / 180) * 127;
  senderPayload.position2[1] = (int8_t) (alphaY / 180) * 127;
  senderPayload.position2[2] = 0;
  senderPayload.voltage = app.getVoltage();
  bool pkg = app.handle(senderPayload);
  /*printVoltage();*/
  if (!app.verifyVoltage())
  {
    app.log("Voltage");
    /*motorsApply = false;*/
    //app.noPackageAction();
  }
  setSpeeds(app.getPayload(), app.recentMessage(), gyroApply);

  delay(RECEIVER_SLEEP);
#endif
}
