#ifndef NRF24L01_PROVIDER_HPP_
#define NRF24L01_PROVIDER_HPP_
#include "communication_provider.hpp"
#include "RF24.h"
class NRF24L01Provider : public CommunicationProvider
{
public:
    NRF24L01Provider() {}
    NRF24L01Provider(int ce, int csn) : cePin(ce), csnPin(csn), radio(ce, csn)
    {
    }
    bool init()
    {
        int r = this->radio.begin();
        if (r)
        {
            this->radio.setPALevel(RF24_PA_LOW);
            this->radio.setAutoAck(true);  // ensure auto-ack is enabled
            this->radio.setRetries(5, 15); // small retry/backoff (adjust if needed)
            this->radio.setPayloadSize(PAYLOAD_LENGTH);
        }
        return r != 0;
    }
    RF24 &getRadio()
    {
        return this->radio;
    }

private:
    int cePin;
    int csnPin;
    RF24 radio;
};
#endif