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
    int read() override {
        int payload_size = 0;
        if (this->read_available()) {
            payload_size = this->get_payload_size();
            if (payload_size > 0) {
                this->radio.read(msgBuf, PAYLOAD_LENGTH);
            }
        }
        return payload_size;
    }
    bool write(void *buf, uint8_t len) override {
        return this->radio.write(buf, len);
    }
    RF24 &getRadio()
    {
        return this->radio;
    }

private:
    int get_payload_size() {
        return this->radio.getPayloadSize();
    }
    bool read_available() {
        return this->radio.available();
    }
    int cePin;
    int csnPin;
    RF24 radio;
};
#endif