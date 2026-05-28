#include <quadrocopter.hpp>
#if VEHICLE == 0 && SENDER == 0
void startupVehicle()
{
    app.logger.log(FLASH_STRING("PID,GYRO"));
    initPIDS();
    initAccelerometer();
    app.logger.log(FLASH_STRING("MOTORS"));
    initServos();
    setServos(1000);
    app.logger.log(FLASH_STRING("SR"));
}
sensors_event_t am, g, temp;
Adafruit_MPU6050 a;
int servoPins[SERVOS] = {A0, A1, A2, A3};
Servo servos[SERVOS];
float pitchVal = 0;
float rollVal = 0;
float alphaXVal = 0;
float alphaYVal = 0;
float alphaX = 0;
float alphaY = 0;
float xGyro = 0;
float yGyro = 0;
PID<float> pids[PIDS];
Filter<float> filters[PIDS];

Filter<float> filters2[2];
uint64_t start = millis();
// what angle we are by default
float gravity[2];
int gyroMsgs = 0;

bool gyroOn = false;
bool gravitySet = false;
bool gyroValid = false;

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

bool setAlphaVals()
{
    a.getEvent(&am, &g, &temp);
    float xAcc = am.acceleration.x;
    float yAcc = am.acceleration.y;
    float zAcc = am.acceleration.z;
    if (isnan(xAcc) || isnan(yAcc) || isnan(zAcc))
    {
        alphaXVal = 0;
        alphaYVal = 0;
        gyroValid = false;
        return false;
    }
    alphaYVal = atan((yAcc) / sqrt(pow((xAcc), 2) + pow((zAcc), 2))) * 57.29577951308232;
    alphaXVal = atan(-1 * (xAcc) / sqrt(pow((yAcc), 2) + pow((zAcc), 2))) * 57.29577951308232;
    gyroValid = true;
    return true;
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
        long m = millis();
        int vals = 0;
        while ((millis() - m) < GRAVITY)
        {
            setAlphaVals();
            gravity[0] += alphaXVal;
            gravity[1] += alphaYVal;
            vals += 1;
        }
        if (vals >= 1)
        {
            for (int i = 0; i < 2; i++)
            {
                gravity[i] /= vals;
            }
            app.logger.log(FLASH_STRING("G 1"));
            gravitySet = true;
            return;
        }
    }
    gravitySet = false;
    gravity[0] = 0;
    gravity[1] = 0;
}

void initGyro()
{
    bool a1 = a.begin();
    if (a1)
    {
        app.logger.log(F("G 1"));
        a.setGyroRange(MPU6050_RANGE_500_DEG);
        a.setFilterBandwidth(MPU6050_BAND_184_HZ);
        a.setSampleRateDivisor(7);
        delay(100);
        gyroOn = true;
    }
    else
    {
        app.logger.log(F("G 0"));
        gyroOn = false;
    }
}

void setAlpha()
{
    bool v = setAlphaVals();
    if (v)
    {
        if (gravitySet)
        {
            alphaXVal -= gravity[0];
            alphaYVal -= gravity[1];
        }
        alphaX = filters[0].update(alphaXVal);
        alphaY = filters[1].update(alphaYVal);
    }
    else
    {
        app.logger.log("Nan");
        gyroMsgs += 1;
        alphaX = 0;
        alphaY = 0;
        if ((gyroMsgs % 18) == 0)
        {
            initGyro();
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

void setSpeeds(int8_t sVals[4], bool motorsApply)
{
    float speeds[4];
    setValues();
    pitchVal = filters[0].update(pids[0].update(alphaY, sVals[1] / 8.0, 1)) / 1.8;
    rollVal = filters[1].update(pids[1].update(alphaX, sVals[2] / 8.0, 1)) / 1.8;
    float pp = inRange<float>(pitchVal, -16, 16); // - (yGyro / 4);
    float rr = inRange<float>(rollVal, -16, 16);  // + (xGyro / 4);
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
        if (v < 1010)
        {
            v = 1010;
        }
        if (!motorsApply)
        {
            v = 1000;
        }
        servos[i].writeMicroseconds(v);
    }
    char buf[78];
    snprintf(buf, 40, "%d %d - %d %d - %d %d - %d %d %d %d", (int)(alphaX), (int)(alphaY), (int)(gravity[0]), (int)(gravity[1]), (int)(pitchVal), (int)(rollVal), (int)speeds[0], (int)speeds[1], (int)speeds[2], (int)speeds[3]);
    app.logger.log(buf);
}

void printVoltage()
{
    if (Serial)
    {
        Serial.print("Voltage: ");
        Serial.print((int)app.voltageHandler.getVoltage(), DEC);
        Serial.print("V");
        Serial.print("\n");
    }
}
void initAccelerometer()
{
    int i = 0;
    while (i < 10)
    {
        initGyro();
        if (gyroOn)
        {
            setGravity();
            return;
        }
    }
}

void initPIDS()
{
    for (int i = 0; i < PIDS; i++)
    {
        pids[i] = PID<float>(0.04, 0.0, 0.01, -1.8, 1.8);
        filters[i] = Filter<float>(0.18);
    }
    for (int i = 0; i < 2; i++)
    {
        filters2[i] = Filter<float>(0.04);
    }
}
#endif