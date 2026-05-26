#ifndef UTILS_HPP_
#define UTILS_HPP_
#define FREQ_BASE 400
#define FREQ_DEFAULT 1000
#define FREQ_HIGH 10000

namespace libmh {

template <typename MessageType> class Subscriber {
public:
  virtual void receive(const MessageType &msg) = 0;
};
template <typename MessageType> class Publisher {
public:
  virtual void publish(const MessageType &msg) = 0;
  virtual void subscribe(Subscriber<MessageType> &sub) = 0;
};

template <typename Fun> class Timer {
  Timer(float expiry) : expiry(expiry) {}

public:
  void start() { lastTime = fun(); }
  bool isExpired() { return (fun() - lastTime) >= expiry; }

private:
  Fun fun;
  float lastTime;
  float expiry;
};
}; // namespace libmh

class ArduinoTimer : public libmh::Timer<float (*)(void)> {
public:
  ArduinoTimer(float expiry) : libmh::Timer<float (*)(void)>(expiry) {}
  float operator()() { return millis / 1000.0; }
};
#endif