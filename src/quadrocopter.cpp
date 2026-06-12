#include <quadrocopter.hpp>
#if VEHICLE == 0 && SENDER == 0
sensors_event_t am, g, temp;
Adafruit_MPU6050 a;
int servoPins[SERVOS] = {A0, A1, A2, A3};
float servoMultiplier[SERVOS] = {1.06, 1.0, 1.04, 1.0};
Servo servos[SERVOS];
// Per-motor yaw direction multiplier: adjust if yaw correction is inverted
int yawDir[SERVOS] = {1, -1, -1, 1};
float pitchVal = 0;
float rollVal = 0;
float alphaXVal = 0;
float alphaYVal = 0;
float alphaX = 0;
float alphaY = 0;
PID<float> pids[PIDS] = {PID<float>(4.5, 0.0, 1, -1.8, 1.8), PID<float>(4.5, 0.0, 1, -1.8, 1.8)};
Filter<float> filters[PIDS] = {Filter<float>(0.45), Filter<float>(0.45)};
Filter<float> filters2[2] = {Filter<float>(0.18), Filter<float>(0.18)};
Filter<float> filters3[3] = {Filter<float>(0.35), Filter<float>(0.35), Filter<float>(0.35)};
uint64_t start = millis();
// what angle we are by default
float gravity[2];
float gravity2[3];
int gyroMsgs = 0;

bool gyroOn = false;
bool gravitySet = false;
bool gyroValid = false;

void startupVehicle()
{
    // app.logger.log(FLASH_STRING("M"));
    initServos();
    setServos(1000);
    // app.logger.log(FLASH_STRING("P.G"));
    initAccelerometer();
    // app.logger.log(FLASH_STRING("SR"));
}

void initServos()
{
    for (int i = 0; i < 4; i++)
    {
        if (servos[i].attach(servoPins[i]) == INVALID_SERVO)
        {
            while (1)
            {
            }
        }
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

bool setAlphaVals()
{
    a.getEvent(&am, &g, &temp);
    float xAcc = am.acceleration.x;
    float yAcc = am.acceleration.y;
    float zAcc = am.acceleration.z;
    if (isnan(xAcc) || isnan(yAcc) || isnan(zAcc))
    {
        gyroValid = false;
    }
    else
    {
        alphaYVal = atan((yAcc) / sqrt(pow((xAcc), 2) + pow((zAcc), 2))) * 57.29577951308232;
        alphaXVal = atan(-1 * (xAcc) / sqrt(pow((yAcc), 2) + pow((zAcc), 2))) * 57.29577951308232;
        gyroValid = true;
    }
    return gyroValid;
}

void setGravity()
{
    app.logger.log(FLASH_STRING("G"));
    if (gyroOn)
    {
        for (int i = 0; i < 2; i++)
        {
            gravity[i] = 0;
        }
        for (int i = 0; i < 3; i++)
        {
            gravity2[i] = 0;
        }
        long m = millis();
        int vals = 0;
        while ((millis() - m) < GRAVITY)
        {
            setAlphaVals();
            gravity[0] += alphaXVal;
            gravity[1] += alphaYVal;
            gravity2[0] += g.gyro.x;
            gravity2[1] += g.gyro.y;
            gravity2[2] += g.gyro.z;
            vals += 1;
        }
        if (vals >= 1)
        {
            for (int i = 0; i < 2; i++)
            {
                gravity[i] /= vals;
            }
            for (int i = 0; i < 3; i++)
            {
                gravity2[i] /= vals;
            }
            app.logger.log(FLASH_STRING("G 1"));
            gravitySet = true;
            return;
        }
    }
    gravitySet = false;
}

bool initGyro()
{
    gyroOn = a.begin();
    if (gyroOn)
    {
        app.logger.log(F("G 1"));
        a.setGyroRange(MPU6050_RANGE_500_DEG);
        // a.setAccelerometerRange(MPU6050_RANGE_4_G);
        a.setFilterBandwidth(MPU6050_BAND_5_HZ);
        a.setSampleRateDivisor(1);
    }
    else
    {
        app.logger.log(FLASH_STRING("G 0"));
    }
    return gyroOn;
}

bool setAlpha()
{
    bool v = setAlphaVals();
    if (v)
    {
        if (gravitySet)
        {
            alphaXVal -= gravity[0];
            alphaYVal -= gravity[1];
            g.gyro.x -= gravity2[0];
            g.gyro.y -= gravity2[1];
            g.gyro.z -= gravity2[2];
        }
        alphaX = filters2[0].update(alphaXVal);
        alphaY = filters2[1].update(alphaYVal);
    }
    else
    {
        app.logger.log(FLASH_STRING("Nan"));
        gyroMsgs += 1;
        if ((gyroMsgs % 18) == 0)
        {
            initGyro();
        }
    }
    return v;
}

void setValues()
{
    setAlpha();
}

void setSpeeds(int8_t sVals[4], bool motorsApply)
{
    float speeds[4];
    setValues();
    pitchVal = filters[0].update(pids[0].update(alphaX / 45, sVals[1] / (127 * 2), 1));
    rollVal = filters[1].update(pids[1].update(alphaY / 45, sVals[2] / (127 * 2), 1));
    int elapsed = millis() - start;
    start = millis();
    float gyroPp = filters3[1].update(g.gyro.y * 57.29577951308232 * elapsed * (0.004));
    float gyroRr = filters3[0].update(g.gyro.x * 57.29577951308232 * elapsed * (0.004));
    float gyroYy = filters3[2].update(g.gyro.z * 57.29577951308232 * elapsed * (0.001));
    float pp = inRange<float>(pitchVal, -8, 8) + inRange<float>(gyroPp, -8, 8);
    float rr = inRange<float>(rollVal, -8, 8) + inRange<float>(gyroRr, -8, 8);
    float yy = 0; // inRange<float>(gyroYy, -8, 8);
    for (int i = 0; i < 4; i++)
    {
        speeds[i] = sVals[0];
        if (gyroOn && gyroValid)
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
            // apply yaw correction (use yawDir to match motor rotation directions)
            speeds[i] += yawDir[i] * yy;
        }
    }
    for (int i = 0; i < 4; i++)
    {
        int v = 1000 + ((int)(800 * (speeds[i] * servoMultiplier[i] / 127)));
        v = inRange<int>(v, 1000, 1800);
        if (!motorsApply)
        {
            v = 1000;
        }
        servos[i].writeMicroseconds(v);
    }
    // char buf[45];
    // snprintf(buf, sizeof(buf), "%d %d - %d %d - %d %d - %d %d %d %d", (int)(alphaX), (int)(alphaY), (int)(gravity[0]), (int)(gravity[1]), (int)(pitchVal), (int)(rollVal), (int)speeds[0], (int)speeds[1], (int)speeds[2], (int)speeds[3]);
    // app.logger.log(buf);
}

void initAccelerometer()
{
    int i = 0;
    while (i < 10)
    {
        if (initGyro())
        {
            setGravity();
            return;
        }
        // delay(10);
    }
}

#endif