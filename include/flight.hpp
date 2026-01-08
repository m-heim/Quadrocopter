#ifndef FLIGHT_HPP
#define FLIGHT_HPP
class Flight {
    public:
    Flight() {}
    private:
    bool motorsApply = false;
    bool gyroActive = false;
    bool hover;
};
#endif