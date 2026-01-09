#include <Arduino.h>
#include <Servo.h>
#include "nrf24l01_provider.hpp"
#include "mcapp.hpp"
#include "utils.hpp"
#include <Adafruit_MPU6050.h>
#include "init.hpp"
#include "pid.hpp"

uint8_t msgBuf[PAYLOAD_LENGTH];
uint8_t payloadLength = 0;
bool isConnected = false;
uint8_t address[][6] = {"Send1", "Recv1"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination
int radioNumber = SENDER; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
NRF24L01Provider radio = NRF24L01Provider(CE_PIN, CSN_PIN);
MCApp app = MCApp();
ReceiverPayload payload = {0, 0, 0, 0};
SenderPayload payload2 = {0};
double vals1[4];
int8_t vals[4];

#if SENDER == 0
sensors_event_t am, g, temp;
Adafruit_MPU6050 a;
int servoPins[4] = {A0, A1, A2, A3};
Servo servos[4];
int freq = FREQ_DEFAULT;
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
// previous message
long msg_a = 0;
#endif

#if SENDER == 0
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

void setAlpha()
{
  float xAcc = am.acceleration.x;
  float yAcc = am.acceleration.y;
  float zAcc = am.acceleration.z;
  alphaX = atan((yAcc) / sqrt(pow((xAcc), 2) + pow((zAcc), 2))) * 57.29577951308232;
  alphaY = atan(-1 * (xAcc) / sqrt(pow((yAcc), 2) + pow((zAcc), 2))) * 57.29577951308232;
  alphaX -= gravity[0];
  alphaY -= gravity[1];
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
  pitchVal = filters[1].update(pids[1].update(alphaY, p.pitch / 8, 1));
  rollVal = filters[2].update(pids[2].update(alphaX, p.roll / 8, 1));
  double pp = inRange(pitchVal, -8, 8); // - (yGyro / 4);
  double rr = inRange(rollVal, -8, 8);  // + (xGyro / 4);
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

void noPackageAction()
{
  app.setLed(1);
  app.buzz(freq, 10);
  freq += 400;
  if (freq > 4000)
  {
    freq = FREQ_BASE;
  }
}

void output(int start, int stop, int step, float seconds)
{
  int s = (int)((seconds / 1000));
  for (int i = start; i <= stop; i += step)
  {
    app.buzz(i, s);
    delay(s);
  }
}

void printPayload()
{
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

void setup()
{
  vals1[0] = 4;
  vals1[1] = 8;
  vals1[2] = 16;
  vals1[3] = 180;
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
#if VEHICLE == 1
  motor_1_setup(1);
  motor_2_setup(1);
#endif
#if SENDER == 0
  for (int i = 0; i < 4; i++)
  {
    pids[i] = PID(0.45, 0.00045, 0.45);
    filters[i] = Filter(0.45);
  }
  app.initPiezo(PIEZO);
  app.setNoPiezo(true);
  app.initVoltage(INVOLTAGE, 3, VOLTAGE);
  app.initLed(LED);
  for (int i = 0; i < 10; i++)
  {
    bool a1 = a.begin();
    if (a1)
    {
      app.log("Accelerometer is working");
      a.setGyroRange(MPU6050_RANGE_500_DEG);
      a.setFilterBandwidth(MPU6050_BAND_184_HZ);
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
  output(1000, 1300, 100, 0.04);
  pinMode(10, OUTPUT); // set pin 10 for output, necessary for spi
  pinMode(INVOLTAGE, INPUT);
  radio.getRadio().openReadingPipe(1, address[0]);
  radio.getRadio().startListening();
  delay(400);
  initServos();
  setServos(1000);
  // setMotors();
  delay(100);
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
  int8_t uartData[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  if (Serial) // read from uart
  {
    Serial.setTimeout(10);
    int s = Serial.readBytesUntil('\n', (char *)uartData, sizeof(uartData) - 1);
    if (s < 1)
    {
      app.log("No action");
    }
    else if (s >= 5 && uartData[0] == 'c')
    {
      memcpy(vals, uartData + 1, 4);
      app.log("Got speed change");
    }
    else if (uartData[0] == 's')
    {
      app.log("Sending setup");
      msgBuf[0] = GYRO_SETUP;
      msgBuf[1] = 0;
      payloadLength = 2;
      bool report = radio.getRadio().write(&msgBuf, payloadLength);
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
    p.speed = vals[0];
    p.pitch = vals[1];
    p.yaw = vals[2];
    p.roll = vals[3];
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
  bool motorsApply = false;
  bool gyroApply = false;
  app.setLed(0);
  if (gyro)
  {
    a.getEvent(&am, &g, &temp);
    gyroApply = true;
  }
  if (radio.getRadio().available())
  { // is there a payload? get the pipe number that recieved it
    hasPackage = true;
    payloadLength = radio.getRadio().getPayloadSize(); // get the size of the payload
    radio.getRadio().read(&msgBuf, payloadLength);     // fetch payload from FIFO

    if (msgBuf[0] == HELLO)
    {
      isConnected = true;
      app.log("Hello from sender");
      output(500, 900, 100, 0.04);
    }
    else if (msgBuf[0] == BYE)
    {
      isConnected = false;
      app.log("Bye from sender");
      output(900, 500, -100, 0.04);
    }
    else if (msgBuf[0] == GYRO_SETUP)
    {
      setGravity();
    }
    else if (msgBuf[0] == MOTOR_SETUP)
    {
    }
    else if (msgBuf[0] == CONTROL)
    {
      memcpy(&payload, msgBuf + 2, sizeof(payload));
      msg_a = millis();
      printPayload();
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
  /*printVoltage();*/
  if (((millis() - msg_a) > NO_MSG))
  {
    noPackageAction();
  }
  else
  {
    motorsApply = true;
  }
  if (!app.verifyVoltage())
  {
    app.log("Voltage");
    /*motorsApply = false;*/
  }
  setSpeeds(payload, motorsApply, gyroApply);
  delay(RECEIVER_SLEEP);
#endif
}
