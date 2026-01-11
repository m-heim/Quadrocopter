#include <Arduino.h>
#include <Servo.h>
#include "nrf24l01_provider.hpp"
#include "mcapp.hpp"
#include "utils.hpp"
#include <Adafruit_MPU6050.h>
#include "init.hpp"
#include "pid.hpp"
#include "vehicle.hpp"

uint8_t payloadLength = 0;
uint8_t address[][6] = {"Send1", "Recv1"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination
int radioNumber = SENDER; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
NRF24L01Provider radio = NRF24L01Provider(CE_PIN, CSN_PIN);
MCApp app = MCApp(&radio);
SenderPayload payload2 = {0};
int freq = FREQ_DEFAULT;
double vals1[4];
int8_t vals[4];
long msg_b;

#if SENDER == 0
sensors_event_t am, g, temp;
Adafruit_MPU6050 a;
int servoPins[4] = {A0, A1, A2, A3};
Servo servos[4];
bool gyro = false;

double pitchVal = 0;
double rollVal = 0;
double alphaX = 0;
double alphaY = 0;
double xGyro = 0;
double yGyro = 0;
PID pids[4];
Filter filters[4];
uint64_t start = millis();
// what angle we are by default
double gravity[2];
int index = 0;
#if VEHICLE == 0
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

void initGyro() {
    bool a1 = a.begin();
    if (a1)
    {
      app.log("Accelerometer is working");
      a.setGyroRange(MPU6050_RANGE_500_DEG);
      a.setFilterBandwidth(MPU6050_BAND_184_HZ);
      a.setSampleRateDivisor(7);
      delay(100);
      gyro = true;
      //setGravity();
    }
    else {
      app.log("Accelerometer is not working");
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
  if (isnan(alphaX) || isnan(alphaY)) {
    app.log("Nan");
    alphaX = 0;
    alphaY = 0;
    if (index == 18) {
      initGyro();
      index = 0;
    } else {
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

void setSpeeds(ReceiverPayload p, bool motorsApply, bool gyroApply)
{
  float speeds[4];
  setValues();
  pitchVal = filters[1].update(pids[1].update(alphaY, app.getPayload().pitch / 4, 1));
  rollVal = filters[2].update(pids[2].update(alphaX, app.getPayload().roll / 4, 1));
  double pp = inRange(pitchVal, -16, 16); // - (yGyro / 4);
  double rr = inRange(rollVal, -16, 16);  // + (xGyro / 4);
  for (int i = 0; i < 4; i++)
  {
    speeds[i] = p.speed;
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
  char buf[189];
  sprintf(buf, "%d %d %d %d %d %d %d %d %d %d", (int)(alphaX * 1000), (int)(alphaY * 1000), (int)(gravity[0] * 1000), (int)(gravity[1] * 1000), (int)(pitchVal * 1000), (int)(rollVal * 1000), (int)speeds[0], (int)speeds[1], (int)speeds[2], (int)speeds[3]);
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
#endif
#if VEHICLE == 1
void setSpeeds(ReceiverPayload p, bool motorsApply, bool gyroApply) {
  if (motorsApply) {
    drive(p.speed * 0.8, p.roll * 0.45);
  } else {
    drive(0, 0);
  }
}
#endif
#endif

void setup()
{
  vals1[0] = 4;
  vals1[1] = 8;
  vals1[2] = 16;
  vals1[3] = 180;
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
  //app.initPiezo(PIEZO);
  app.initVoltage(INVOLTAGE, 3, 8);
  #if VEHICLE == 0
  app.initLed(LED);
  for (int i = 0; i < 4; i++)
  {
    pids[i] = PID(0.18, 0.0, 0.18, -4, 4);
    filters[i] = Filter(0.18);
  }
  for (int i = 0; i < 10; i++)
  {
    bool a1 = a.begin();
    if (a1)
    {
      app.log("Accelerometer is working");
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
      app.log("Accelerometer is not working");
    }
    delay(10);
  }
  app.output(1000, 1300, 100, 0.04);
  initServos();
  setServos(1000);
  //setMotors();
  #endif
  #if VEHICLE == 1
  motor_1_setup(1);
  motor_2_setup(1);
  #endif
  pinMode(10, OUTPUT); // set pin 10 for output, necessary for spi
  radio.getRadio().openReadingPipe(1, address[0]);
  radio.getRadio().startListening();
  delay(400);
  delay(100);
#else
  app.log("Setting up radio for sender");
  Serial.setTimeout(10);
  pinMode(10, OUTPUT); // set pin 10 for output, necessary for spi
  radio.getRadio().openWritingPipe(address[0]);
  radio.getRadio().stopListening();
  app.log("Radio setup for sender");
#endif
}
void loop()
{
  int action = 1;
  bool gyroSetup = false;
#if SENDER == 1
  int8_t uartData[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  if (Serial) // read from uart
  {
    int s = Serial.readBytesUntil('\n', (char *)uartData, sizeof(uartData) - 1);
    if (s < 1)
    {
      app.log("No action");
    }
    else if (s >= 5 && uartData[0] == 'c')
    {
      memcpy(vals, uartData + 1, 4);
      app.log("Got speed change");
      msg_b = millis();
    }
    else if (uartData[0] == 's')
    {
      app.log("Sending setup");
      gyroSetup = true;
    }
  }
  if ((millis() - msg_b) > NO_MSG) {
    vals[0] = 0;
    vals[1] = 0;
    vals[2] = 0;
    vals[3] = 0;
  }
  app.handle2(vals, gyroSetup);
  delay(SENDER_SLEEP);
#else
  bool gyroApply = false;
  app.setLed(0);
  if (gyro)
  {
    a.getEvent(&am, &g, &temp);
    gyroApply = true;
  }
  bool pkg = app.handle();
  /*printVoltage();*/
  if (!app.verifyVoltage())
  {
    app.log("Voltage");
    /*motorsApply = false;*/
  }
  setSpeeds(app.getPayload(), app.recentMessage(), gyroApply);
  delay(RECEIVER_SLEEP);
#endif
}
