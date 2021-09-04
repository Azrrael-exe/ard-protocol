#ifndef DATALINK
#define DATALINK

#include <stdint.h>
#include <Arduino.h>
#include "exception.h"

namespace datalink_error {
    static const int EMPTY = 2;
    static const int INVALID__CHECKSUM = 1;
};

class Datalink {
    public:
        Datalink(uint8_t header, uint8_t size, bool strict=false);
        void read(Stream &uart);
        uint8_t* getPayload();
        void send(uint8_t* payload, uint8_t size, Stream &uart);
    private:
        uint8_t header;
        uint8_t* buffer;
        uint8_t size;
        uint8_t index;
        bool available;
        bool strict;
};

#endif