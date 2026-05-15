#ifndef PID_HPP
#define PID_HPP
#include <stdlib.h>

class PID
{
public:
    PID() {}
    PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}
    PID(float kp, float ki, float kd, float integralRange1, float integralRange2) : kp(kp), ki(ki), kd(kd), integralRange1(integralRange1), integralRange2(integralRange2), useIntegralRange(true) {}
    ~PID() {}
    float update(float val, float set, float step)
    {
        error = val - set;
        error_derivative = error - prev_err;
        if (!useIntegralRange || (val > integralRange1 && val < integralRange2))
        {
            error_sum += error;
        }
        prev_err = error;
        return getValue();
    }
    float getValue()
    {
        val = 0;
        val += error * kp;
        val += error_sum * ki;
        val += error_derivative * kd;
        return val;
    }

private:
    float integralRange1;
    float integralRange2;
    float kp;
    float ki;
    float kd;
    bool useIntegralRange = false;
    float prev_err = 0;
    float val = 0;
    float error = 0;
    float error_sum = 0;
    float error_derivative = 0;
};

class Filter
{
public:
    Filter() {}
    Filter(float factor) : factor(factor)
    {
        if (factor < 0 || factor > 1)
        {
            exit(1);
        }
    }
    ~Filter() {}
    float update(float value)
    {
        val = (factor * value) + ((1 - factor) * val);
        return getValue();
    }
    float getValue() {
        return val;
    }

private:
    float factor = 0;
    float val = 0;
};
float inRange(float val, float r1, float r2)
{
    if (val < r1)
    {
        val = r1;
    }
    if (val > r2)
    {
        val = r2;
    }
    return val;
}
#endif