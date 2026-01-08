#ifndef FLIGHT_HPP
#define FLIGHT_HPP
#include <vector>
#include <Servo.h>
class Flight
{
public:
    Flight(std::vector<int> servoPins)
    {
        for (int i = 0; i < servoPins.size(); i++)
        {
            servos[i] = Servo(servoPins[0]);
        }
    }

private:
    bool motorsApply = false;
    bool gyroActive = false;
    bool hover;
    std::vector<Servo> servos;
};
#endif