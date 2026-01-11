#ifndef PID_HPP
#define PID_HPP
#include <stdlib.h>

class PID
{
public:
    PID() {}
    PID(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd) {}
    PID(double kp, double ki, double kd, double integralRange1, double integralRange2) : kp(kp), ki(ki), kd(kd), integralRange1(integralRange1), integralRange2(integralRange2), useIntegralRange(true) {}
    ~PID() {}
    double update(double val, double set, double step)
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
    double getValue()
    {
        val = 0;
        val += error * kp;
        val += error_sum * ki;
        val += error_derivative * kd;
        return val;
    }

private:
    double integralRange1;
    double integralRange2;
    double kp;
    double ki;
    double kd;
    bool useIntegralRange = false;
    double prev_err = 0;
    double val = 0;
    double error = 0;
    double error_sum = 0;
    double error_derivative = 0;
};

class Filter
{
public:
    Filter() {}
    Filter(double factor) : factor(factor)
    {
        if (factor < 0 || factor > 1)
        {
            exit(1);
        }
    }
    ~Filter() {}
    double update(double value)
    {
        val = (factor * value) + ((1 - factor) * val);
        return getValue();
    }
    double getValue() {
        return val;
    }

private:
    double factor = 0;
    double val = 0;
};
double inRange(double val, double r1, double r2)
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