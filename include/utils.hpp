#ifndef UTILS_HPP_
#define UTILS_HPP_
#define FREQ_BASE 400
#define FREQ_DEFAULT 1000
#define FREQ_HIGH 10000

namespace libmh
{

  template <typename MessageType>
  class Subscriber
  {
  public:
    virtual void receive(const MessageType &msg) = 0;
  };
  template <typename MessageType>
  class Publisher
  {
  public:
    virtual void publish(const MessageType &msg) = 0;
    virtual void subscribe(Subscriber<MessageType> &sub) = 0;
  };

  class Timer
  {
  public:
    Timer(float expiry, float (*fun)()) : expiry(expiry), fun(fun)
    {
      lastTime = this->fun() - expiry;
    }

    void start() { lastTime = fun(); }
    bool isExpired() { return (fun() - lastTime) >= expiry; }

  protected:
    float expiry;
    float (*fun)();
    float lastTime;
  };
  inline float fun1()
  {
    return millis() / 1000.0;
  }

  class ArduinoTimer : public libmh::Timer
  {
  public:
    ArduinoTimer(float expiry) : libmh::Timer(expiry, fun1) {}
  };

  inline uint8_t getNibble(uint8_t v)
  {
    if (v >= '0' && v <= '9')
    {
      return v - '0';
    }
    else if (v >= 'A' && v <= 'F')
    {
      return v - 'A' + 10;
    }
    else if (v >= 'a' && v <= 'f')
    {
      return v - 'a' + 10;
    }
    else
    {
      return 0;
    }
  }
};

#ifdef ARDUINO
#define FLASH_STRING(x) (F(x))
#elif defined(__GNUC__) // STM32, ESP32, etc.
#define FLASH_STRING(x) (__extension__({                           \
  static const char __c[] __attribute__((section(".rodata"))) = x; \
  __c;                                                             \
}))
#else
#define FLASH_STRING(x) (x) // Fallback for unknown platforms
#endif
#endif