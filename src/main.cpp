#include "init.hpp"
#include "mcapp.hpp"
#include "nrf24l01_provider.hpp"
#include "pid.hpp"
#include "utils.hpp"
#include "vehicle.hpp"
#include <Adafruit_MPU6050.h>
#include <Arduino.h>
#include <Servo.h>

#define SERVOS 4
#define PIDS 2

uint8_t address[][6] = {"Send1", "Recv1"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination
uint8_t radioNumber =
    SENDER; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
NRF24L01Provider radio = NRF24L01Provider(CE_PIN, CSN_PIN);
UartIOHandler uartIOHandler = UartIOHandler();
MCApp app = MCApp(&radio, uartIOHandler);

#if SENDER == 0
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
PID pids[PIDS];
Filter filters[PIDS];
uint64_t start = millis();
// what angle we are by default
float gravity[2];
int index = 0;
#if VEHICLE == 0
void initServos() {
  for (int i = 0; i < 4; i++) {
    servos[i].attach(servoPins[i]);
  }
}

void setServos(int speed) {
  for (int i = 0; i < 4; i++) {
    servos[i].writeMicroseconds(speed);
  }
}

void setMotors() {
  setServos(2000);
  delay(4000);
  setServos(1000);
  delay(4000);
}

void setGravity() {
  app.log("Setting gravity");
  if (gyro) {
    long m = millis();
    int index = 0;
    for (int i = 0; i < 2; i++) {
      gravity[i] = 0;
    }
    while ((millis() - m) < GRAVITY) {
      a.getEvent(&am, &g, &temp);
      float xAcc = am.acceleration.x;
      float yAcc = am.acceleration.y;
      float zAcc = am.acceleration.z;
      gravity[0] += atan((yAcc) / sqrt(pow((xAcc), 2) + pow((zAcc), 2))) *
                    57.29577951308232;
      gravity[1] += atan(-1 * (xAcc) / sqrt(pow((yAcc), 2) + pow((zAcc), 2))) *
                    57.29577951308232;
      index += 1;
    }
    if (index >= 1) {
      for (int i = 0; i < 2; i++) {
        gravity[i] /= index;
      }
    } else {
      for (int i = 0; i < 2; i++) {
        gravity[i] = 0;
      }
    }
  }
  app.log("Setting gravity ok");
}

void initGyro() {
  bool a1 = a.begin();
  if (a1) {
    app.log(F("G 1"));
    a.setGyroRange(MPU6050_RANGE_500_DEG);
    a.setFilterBandwidth(MPU6050_BAND_184_HZ);
    a.setSampleRateDivisor(7);
    delay(100);
    gyro = true;
    // setGravity();
  } else {
    app.log(F("G 0"));
  }
}

void setAlpha() {
  float xAcc = am.acceleration.x;
  float yAcc = am.acceleration.y;
  float zAcc = am.acceleration.z;
  float alphaX1 =
      atan((yAcc) / sqrt(pow((xAcc), 2) + pow((zAcc), 2))) * 57.29577951308232;
  float alphaY1 = atan(-1 * (xAcc) / sqrt(pow((yAcc), 2) + pow((zAcc), 2))) *
                  57.29577951308232;
  alphaX1 -= gravity[0];
  alphaY1 -= gravity[0];
  alphaX = (LP * alphaX) + ((1 - LP) * alphaX1);
  alphaY = (LP * alphaY) + ((1 - LP) * alphaY1);
  if (isnan(alphaX) || isnan(alphaY)) {
    app.log("Nan");
    alphaX = 0;
    alphaY = 0;
    if (index == 18) { // if we have 18 readings, reset gyro to try to fix it
      initGyro();
      index = 0;
    } else {
      index += 1;
    }
  }
}

void setGyro() {
  xGyro = g.gyro.x;
  yGyro = g.gyro.y;
}

void setValues() {
  setAlpha();
  setGyro();
}

void setSpeeds(ReceiverPayload p, bool motorsApply, bool gyroApply) {
  float speeds[4];
  setValues();
  pitchVal =
      filters[0].update(pids[0].update(alphaY, app.getPayload().pitch / 4, 1));
  rollVal =
      filters[1].update(pids[1].update(alphaX, app.getPayload().roll / 4, 1));
  float pp = inRange(pitchVal, -16, 16); // - (yGyro / 4);
  float rr = inRange(rollVal, -16, 16);  // + (xGyro / 4);
  for (int i = 0; i < 4; i++) {
    speeds[i] = p.speed;
    if (gyroApply) {
      if (i & BACK) {
        speeds[i] += pp;
      } else {
        speeds[i] -= pp;
      }
      if (i & RIGHT) {
        speeds[i] -= rr;
      } else {
        speeds[i] += rr;
      }
    }
  }
  for (int i = 0; i < 4; i++) {
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
  char buf[40];
  snprintf(buf, 40, "%d %d %d %d %d %d %d %d %d %d", (int)(alphaX * 1000),
           (int)(alphaY * 1000), (int)(gravity[0] * 1000),
           (int)(gravity[1] * 1000), (int)(pitchVal * 1000),
           (int)(rollVal * 1000), (int)speeds[0], (int)speeds[1],
           (int)speeds[2], (int)speeds[3]);
  app.log(buf);
}

void printVoltage() {
  if (Serial) {
    Serial.print("Voltage: ");
    Serial.print((int)app.getVoltage(), DEC);
    Serial.print("V");
    Serial.print("\n");
  }
}
void initAccelerometer() {
  for (int i = 0; i < 10; i++) {
    app.log("I1");
    bool a1 = a.begin();
    delay(100);
    app.log("I1");
    if (a1) {
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
    if (i == 9) {
      app.log("OM 0");
    }
    delay(10);
  }
}

void initPIDS() {
  for (int i = 0; i < PIDS; i++) {
    pids[i] = PID(0.18, 0.0, 0.18, -4, 4);
    filters[i] = Filter(0.18);
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

void setup() {
  app.initLog(115200);
  // put your setup code here, to run once:
  if (!radio.init()) {
    app.log("RF24 0");
    app.ledError();
  } else {
    app.log("RF24 1");
  }
#if SENDER == 0
#if VEHICLE == 0
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
#endif
#if VEHICLE == 1
  motor_1_setup(1);
  motor_2_setup(1);
#endif
  app.log("Setting up radio for receiver");
  radio.getRadio().openReadingPipe(1, address[0]);
  radio.getRadio().openWritingPipe(address[1]);
  radio.getRadio().stopListening();
  app.log("Radio setup for receiver");
  delay(100);
  app.output(1000, 1300, 100, 0.04); // buzz on startup to indicate it is on
#else
  app.log("Setting up radio for sender");
  Serial.setTimeout(100); // 100 ms timeout for serial read, adjust if needed
  radio.getRadio().openWritingPipe(address[0]);
  radio.getRadio().openReadingPipe(1, address[1]);
  radio.getRadio().stopListening();
  app.log("Radio setup for sender");
#endif
}
SenderPayload senderPayload;
QuadrocopterMessage p;
InputHandler inputHandler;
void loop() {
  bool gyroSetup = false;
#if SENDER == 1
  int8_t uartData[35] = {0};
  bool valid = false;
  if (Serial) // read from uart
  {
    String s = Serial.readStringUntil('\n');
    valid = inputHandler.handle(s, p);
  }
  app.handle2(p, valid && inputHandler.isRecent());
  char buf[35];
  sprintf(buf, "%d %d", (int)senderPayload.position2[0] / 127 * 180,
          (int)senderPayload.position2[1] / 127 * 180);
  app.log(buf);
  delay(SENDER_SLEEP);
#else
  bool gyroApply = false;
  app.setLed(0);
  if (gyro) {
    a.getEvent(&am, &g, &temp);
    gyroApply = true;
  }
  SenderPayload senderPayload;
  senderPayload.position2[0] = (int8_t)(alphaX / 180) * 127;
  senderPayload.position2[1] = (int8_t)(alphaY / 180) * 127;
  senderPayload.position2[2] = 0;
  senderPayload.voltage = app.getVoltage();
  bool pkg = app.handle(senderPayload);
  /*printVoltage();*/
  if (!app.verifyVoltage()) {
    app.log("Voltage");
    /*motorsApply = false;*/
    // app.noPackageAction();
  }
  setSpeeds(app.getPayload(), app.recentMessage(), gyroApply);

  delay(RECEIVER_SLEEP);
#endif
}
