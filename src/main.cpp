#include <Arduino.h>
#include <Servo.h>
#include "nrf24l01_provider.hpp"
#include "mcapp.hpp"
#include "utils.hpp"
#include <Adafruit_MPU6050.h>

#define SENDER 0
#define VEHICLE 0
#define PIEZO 4
#define LED 5
#define INVOLTAGE A7
#define RECEIVER_SLEEP 20
#define SENDER_SLEEP 40
#define NO_MSG 450
#define VOLTAGE 11.4

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
#define BACK_LEFT BACK | LEFT
#define BACK_RIGHT BACK | RIGHT

#define EXPONENTIAL_FACTOR 0.45


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
sensors_event_t am, g, temp;
Adafruit_MPU6050 a;
#endif

int8_t vals[4] = {0, 0, 0, 0};

ReceiverPayload payload = {0, 0, 0, 0};

bool motorsApply = false;
bool gyroApply = false;
bool gyro = false;

double frontWeight = 0;
double leftWeight = 0;

float setPoints[4];
float value[4];
float error[4];
float error_sum[4];
float deltaError[4];
float previousError[4];
float pid[4];

uint64_t v = millis();

void setValues() {
  float xAcc = am.acceleration.x;
  float yAcc = am.acceleration.y;
  float zAcc = am.acceleration.z;
  float alphaX1 = atan((yAcc)/sqrt(pow((xAcc),2) + pow((zAcc),2))) * 57.29577951308232;
  float alphaY1 = atan(-1*(xAcc)/sqrt(pow((yAcc),2) + pow((zAcc),2))) * 57.29577951308232;
  float alphaX = (alphaX1 * 0.45) + 0.58;
  float alphaY = (alphaY1 * 0.45) + 1;
  float xGyro = g.gyro.x * 4.5 + 0.25;
  float yGyro = g.gyro.y * 4.5 + 0.07;
  float leftWeightVal = xGyro * 0.3 + alphaX * 0.7;
  float frontWeightVal = -(yGyro * 0.3 + alphaY * 0.7);
  value[1] = frontWeightVal;
  value[3] = leftWeightVal;
  char buf[189];
  sprintf(buf, "%d %d %d %d", (int) (alphaX * 1000), (int) (alphaY * 1000), (int) (xGyro * 1000), (int) (yGyro * 1000));
  app.log(buf);
}

#if SENDER == 0
void setSpeeds(ReceiverPayload p) {
  float speeds[4];
  setValues();
  leftWeight = (EXPONENTIAL_FACTOR * value[3]) + ((1 - EXPONENTIAL_FACTOR) * leftWeight);
  frontWeight = (EXPONENTIAL_FACTOR * value[1]) + ((1 - EXPONENTIAL_FACTOR) * frontWeight);
  /*if (abs(frontWeight) < 0.4) {
    frontWeight = 0;
  } else */
  if (frontWeight < -8) {
    frontWeight = -8;
  } else if (frontWeight > 8) {
    frontWeight = 8;
  }
  /*if (abs(leftWeight) < 0.4) {
    leftWeight = 0;
  } else */
  if (leftWeight < -8) {
    leftWeight = -8;
  } else if (leftWeight > 8) {
    leftWeight = 8;
  }
  for (int i = 0; i < 4; i++)
  {
    speeds[i] = p.speed;
    if (i & BACK) {
      speeds[i] += p.pitch / 18;
    } else {
      speeds[i] -= p.pitch / 18;
    }
    if (i & RIGHT) {
      speeds[i] += p.roll / 18;
    } else {
      speeds[i] -= p.roll / 18;
    }
    if (gyroApply) {
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
  }
  for (int i = 0; i < 4; i++)
  {
    int v = 1000;
    float s = speeds[i];
    s = s / 127;
    v += 800 * s;
    if (v > 1800) {
      v = 1800;
    }
    if (v < 1000) {
      v = 1000;
    }
    if (!motorsApply) {
      v = 1000;
    }
    servos[i].writeMicroseconds(v);
  }
  char buf[189];
  sprintf(buf, "%d %d %d %d %d %d", (int) (frontWeight * 1000), (int) (leftWeight * 1000), (int) speeds[0], (int) speeds[1], (int) speeds[2], (int) speeds[3]);
  app.log(buf);
}

void noPackageAction() {
  app.setLed(1);
  app.buzz(freq, 10);
  freq += 400;
  if (freq > 4000)
  {
    freq = FREQ_BASE;
  }
}
#endif

#if VEHICLE == 1
#define MOTOR_1_EN_L 2
#define MOTOR_1_EN_R 4
#define MOTOR_1_PWM_L 5
#define MOTOR_1_PWM_R 6

#define MOTOR_2_PWM_R 10
#define MOTOR_2_PWM_L 9
#define MOTOR_2_EN_L A1
#define MOTOR_2_EN_R A2


void motor_1_drive(int8_t speed) {
  if (speed > 0) {
    analogWrite(MOTOR_1_PWM_L, speed * 8);
    analogWrite(MOTOR_1_PWM_R, 0);
  } if (speed < 0) {
    analogWrite(MOTOR_1_PWM_L, 0);
    analogWrite(MOTOR_1_PWM_R, speed * 8);
  } else {
    analogWrite(MOTOR_1_PWM_L, 0);
    analogWrite(MOTOR_1_PWM_R, 0);
  }
}

void motor_2_drive(int8_t speed) {
  if (speed > 0) {
    analogWrite(MOTOR_2_PWM_L, speed * 8);
    analogWrite(MOTOR_2_PWM_R, 0);
  } if (speed < 0) {
    analogWrite(MOTOR_2_PWM_L, 0);
    analogWrite(MOTOR_2_PWM_R, speed * 8);
  } else {
    analogWrite(MOTOR_2_PWM_L, 0);
    analogWrite(MOTOR_2_PWM_R, 0);
  }
}

void motor_1_setup(int status) {
  digitalWrite(MOTOR_1_EN_L, status);
  digitalWrite(MOTOR_1_EN_R, status);
}

void motor_2_setup(int status) {
  digitalWrite(MOTOR_2_EN_L, status);
  digitalWrite(MOTOR_2_EN_R, status);
}

void drive(int8_t speed, int8_t steering) {
  int16_t speed_left = speed + steering;
  int16_t speed_right = speed - steering;
  if (speed_left > 127) {
    speed_left = 127;
  } else if (speed_left < -127) {
    speed_left = -127;
  }

  if (speed_right > 127) {
    speed_right = 127;
  } else if (speed_right < -127) {
    speed_right = -127;
  }

  motor_1_drive(speed_left);
  motor_2_drive(speed_right);
}
#endif

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
#if VEHICLE == 1
  motor_1_setup(1);
  motor_2_setup(1);
#endif
#if SENDER == 0
  app.initPiezo(PIEZO);
  app.setNoPiezo(true);
  app.initVoltage(INVOLTAGE, 3, VOLTAGE);
  app.initLed(LED);
  for (int i = 0; i < 10; i++) {
    bool a1 = a.begin();
    if (a1) {
      app.log("Accelerometer is working");
      a.setGyroRange(MPU6050_RANGE_500_DEG);
      a.setFilterBandwidth(MPU6050_BAND_184_HZ);
      a.setSampleRateDivisor(7);
      gyro = true;
      break;
    }
    if (i == 9) {
      app.log("Accelerometer is not working");
    }
  }
  for (int i = 0; i < 4; i++)
  {
    app.buzz(1000 + i * 100, 40);
    delay(40);
  }
  pinMode(10, OUTPUT); // set pin 10 for output, necessary for spi
  pinMode(INVOLTAGE, INPUT);
  radio.getRadio().openReadingPipe(1, address[0]);
  radio.getRadio().startListening();
  delay(400);
  for (int i = 0; i < 4; i++)
  {
    servos[i].attach(servoPins[i]);
    servos[i].writeMicroseconds(1000);
  }
  /*delay(4000);
  for (int i = 0; i < 4; i++)
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
  int8_t uartData[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  if (Serial) // read from uart
  {
    Serial.setTimeout(10);
    int s = Serial.readBytesUntil('\n', (char *)uartData, sizeof(uartData) - 1);
    if (s <= 1)
    {
      app.log("No action");
    }
    else if (s >= 5 && uartData[0] == 'c')
    {
      memcpy(vals, uartData + 1, 4);
      app.log("Got speed change");
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
  app.setLed(0);
  motorsApply = false;
  gyroApply = false;
  if (gyro) {
    a.getEvent(&am, &g, &temp);
    gyroApply = true;
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
    motorsApply = true;
  }
  if (!app.verifyVoltage()) {
    app.log("Voltage");
    /*motorsApply = false;*/
  }
  setSpeeds(payload);
  delay(RECEIVER_SLEEP);
#endif
}
