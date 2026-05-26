#ifndef PID_HPP
#define PID_HPP
#include <stdlib.h>

template <class T>
class PID
{
public:
    PID() {}
    PID(T kp, T ki, T kd) : kp(kp), ki(ki), kd(kd) {}
    PID(T kp, T ki, T kd, T integralRange1, T integralRange2) : kp(kp), ki(ki), kd(kd), integralRange1(integralRange1), integralRange2(integralRange2), useIntegralRange(true) {}
    ~PID() {}
    T update(T val, T set, T step)
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
    T getValue()
    {
        val = 0;
        val += error * kp;
        val += error_sum * ki;
        val += error_derivative * kd;
        return val;
    }

private:
    T kp;
    T ki;
    T kd;
    bool useIntegralRange = false;
    T integralRange1;
    T integralRange2;

    T prev_err = 0;
    T val = 0;

    T error = 0;
    T error_sum = 0;
    T error_derivative = 0;
};

template <class T>
class Filter
{
public:
    Filter() {}
    Filter(T factor) : factor(factor)
    {
        if (factor < 0 || factor > 1)
        {
            exit(1);
        }
    }
    ~Filter() {}
    T update(T value)
    {
        val = (factor * value) + ((1 - factor) * val);
        return getValue();
    }
    T getValue() {
        return val;
    }

private:
    T factor = 0;
    T val = 0;
};

template <class T>
T inRange(T val, T r1, T r2)
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