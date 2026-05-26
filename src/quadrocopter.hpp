#include "mcapp.hpp"
#include "servo.h"
#include "nrf24l01_provider.hpp"
#include <Adafruit_MPU6050.h>
#include <pid.hpp>

#if SENDER == 0
#define SERVOS 4
#define PIDS 2
#define PIEZO 4
#define LED 5
#define INVOLTAGE A7
#define GRAVITY 4000

#define FRONT 0x00
#define BACK 0x02
#define LEFT 0x00
#define RIGHT 0x01

#define FRONT_LEFT FRONT | LEFT
#define FRONT_RIGHT FRONT | RIGHT
#define BACK_LEFT BACK | LEFT
#define BACK_RIGHT BACK | RIGHT

#define EXPONENTIAL_FACTOR 0.45
#define LP (1 - 0.45)

extern NRF24L01Provider radio;
extern MCApp app;
sensors_event_t am, g, temp;
Adafruit_MPU6050 a;
int servoPins[SERVOS] = {A0, A1, A2, A3};
Servo servos[SERVOS];
bool gyro = false; // gyro apply

float pitchVal = 0;
float rollVal = 0;
float alphaX = 0;
float alphaY = 0;
float xGyro = 0;
float yGyro = 0;
PID<float> pids[PIDS];
Filter<float> filters[PIDS];
uint64_t start = millis();
// what angle we are by default
float gravity[2];
int index = 0;

void initServos()
{
  for (int i = 0; i < 4; i++)
  {
    servos[i].attach(servoPins[i]);
  }
}

void setServos(int speed)
{
  for (int i = 0; i < 4; i++)
  {
    servos[i].writeMicroseconds(speed);
  }
}

void setMotors()
{
  setServos(2000);
  delay(4000);
  setServos(1000);
  delay(4000);
}

void setGravity()
{
  app.log("Setting gravity");
  if (gyro)
  {
    long m = millis();
    int index = 0;
    for (int i = 0; i < 2; i++)
    {
      gravity[i] = 0;
    }
    while ((millis() - m) < GRAVITY)
    {
      a.getEvent(&am, &g, &temp);
      float xAcc = am.acceleration.x;
      float yAcc = am.acceleration.y;
      float zAcc = am.acceleration.z;
      gravity[0] += atan((yAcc) / sqrt(pow((xAcc), 2) + pow((zAcc), 2))) * 57.29577951308232;
      gravity[1] += atan(-1 * (xAcc) / sqrt(pow((yAcc), 2) + pow((zAcc), 2))) * 57.29577951308232;
      index += 1;
    }
    if (index >= 1)
    {
      for (int i = 0; i < 2; i++)
      {
        gravity[i] /= index;
      }
    }
    else
    {
      for (int i = 0; i < 2; i++)
      {
        gravity[i] = 0;
      }
    }
  }
  app.log("Setting gravity ok");
}

void initGyro()
{
  bool a1 = a.begin();
  if (a1)
  {
    app.log(F("G 1"));
    a.setGyroRange(MPU6050_RANGE_500_DEG);
    a.setFilterBandwidth(MPU6050_BAND_184_HZ);
    a.setSampleRateDivisor(7);
    delay(100);
    gyro = true;
    // setGravity();
  }
  else
  {
    app.log(F("G 0"));
  }
}

void setAlpha()
{
  float xAcc = am.acceleration.x;
  float yAcc = am.acceleration.y;
  float zAcc = am.acceleration.z;
  float alphaX1 = atan((yAcc) / sqrt(pow((xAcc), 2) + pow((zAcc), 2))) * 57.29577951308232;
  float alphaY1 = atan(-1 * (xAcc) / sqrt(pow((yAcc), 2) + pow((zAcc), 2))) * 57.29577951308232;
  alphaX1 -= gravity[0];
  alphaY1 -= gravity[0];
  alphaX = (LP * alphaX) + ((1 - LP) * alphaX1);
  alphaY = (LP * alphaY) + ((1 - LP) * alphaY1);
  if (isnan(alphaX) || isnan(alphaY))
  {
    app.log("Nan");
    alphaX = 0;
    alphaY = 0;
    if (index == 18)
    { // if we have 18 readings, reset gyro to try to fix it
      initGyro();
      index = 0;
    }
    else
    {
      index += 1;
    }
  }
}

void setGyro()
{
  xGyro = g.gyro.x;
  yGyro = g.gyro.y;
}

void setValues()
{
  setAlpha();
  setGyro();
}

void setSpeeds(int8_t sVals[4], bool motorsApply, bool gyroApply)
{
  float speeds[4];
  setValues();
  pitchVal = filters[0].update(pids[0].update(alphaY, sVals[1] / 4.0, 1));
  rollVal = filters[1].update(pids[1].update(alphaX, sVals[2] / 4.0, 1));
  float pp = inRange<float>(pitchVal, -16, 16); // - (yGyro / 4);
  float rr = inRange<float>(rollVal, -16, 16);  // + (xGyro / 4);
  for (int i = 0; i < 4; i++)
  {
    speeds[i] = speeds[i];
    if (gyroApply)
    {
      if (i & BACK)
      {
        speeds[i] += pp;
      }
      else
      {
        speeds[i] -= pp;
      }
      if (i & RIGHT)
      {
        speeds[i] -= rr;
      }
      else
      {
        speeds[i] += rr;
      }
    }
  }
  for (int i = 0; i < 4; i++)
  {
    int v = 1000;
    float s = speeds[i];
    s = s / 127;
    v += 800 * s;
    if (v > 1800)
    {
      v = 1800;
    }
    if (v < 1000)
    {
      v = 1000;
    }
    if (!motorsApply)
    {
      v = 1000;
    }
    servos[i].writeMicroseconds(v);
  }
  char buf[40];
  snprintf(buf, 40, "%d %d %d %d %d %d %d %d %d %d", (int)(alphaX * 1000), (int)(alphaY * 1000), (int)(gravity[0] * 1000), (int)(gravity[1] * 1000), (int)(pitchVal * 1000), (int)(rollVal * 1000), (int)speeds[0], (int)speeds[1], (int)speeds[2], (int)speeds[3]);
  app.log(buf);
}

void printVoltage()
{
  if (Serial)
  {
    Serial.print("Voltage: ");
    Serial.print((int)app.getVoltage(), DEC);
    Serial.print("V");
    Serial.print("\n");
  }
}
void initAccelerometer()
{
  for (int i = 0; i < 10; i++)
  {
    app.log("I1");
    bool a1 = a.begin();
    delay(100);
    app.log("I1");
    if (a1)
    {
      delay(10);
      app.log("OM 1");
      delay(10);
      a.setGyroRange(MPU6050_RANGE_500_DEG);
      a.setFilterBandwidth(MPU6050_BAND_5_HZ);
      a.setSampleRateDivisor(7);
      delay(100);
      gyro = true;
      setGravity();
      break;
    }
    if (i == 9)
    {
      app.log("OM 0");
    }
    delay(10);
  }
}

void initPIDS()
{
  for (int i = 0; i < PIDS; i++)
  {
    pids[i] = PID<float>(0.18, 0.0, 0.18, -4, 4);
    filters[i] = Filter<float>(0.18);
  }
}

void startup()
{
  app.log("1");
  app.initLed(LED);
  app.initPiezo(PIEZO);
  app.initVoltage(INVOLTAGE, 3, 8);
  app.log("PID,GYRO");
  initPIDS();
  initAccelerometer();
  app.log("MOTORS");
  initServos();
  setServos(1000);
  app.log("Setting up radio for receiver");
  radio.getRadio().openReadingPipe(1, address[0]);
  radio.getRadio().openWritingPipe(address[1]);
  radio.getRadio().stopListening();
  app.log("Radio setup for receiver");
  delay(100);
  app.output(1000, 1300, 100, 0.04); // buzz on startup to indicate it is on
}
#endif