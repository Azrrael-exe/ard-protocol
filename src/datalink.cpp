#include <stdint.h>
#include <Arduino.h>
#include "exception.h"
#include "datalink.h"

Datalink::Datalink(uint8_t header, uint8_t size, bool strict) {
    this->header = header;
    this->size = size;
    this->buffer = new uint8_t[this->size];
    this->strict = strict;
    this->index = 0;
}

void Datalink::read(Stream &uart) {
    if(uart.available()){
        if (uint8_t(uart.read()) == this->header) {
            uint8_t length = uart.read() + 1;
            uint8_t* buffer = new uint8_t[length];
            uart.readBytes(buffer, length);
            int local_checksum = 0;
            for(int i=0; i<length-1; i++){
                local_checksum+=buffer[i];
            }
            if(byte(0xFF - local_checksum) == buffer[length]){
                for(int i=0; i<length-1; i++) {
                    this->buffer[this->index+i] = buffer[i];
                }
                this->index += length - 1;
                this->available = true;
            }
            if(this->strict) {
                throw Exception(datalink_error::INVALID__CHECKSUM);
            }
        } 
    }
}

uint8_t* Datalink::getPayload(){
    uint8_t* payload = new uint8_t[this->index];
    for(int i=0; i<this->index; i++){
        payload[i] = this->buffer[i];
    }
    this->index=0;
    return payload;
}

void Datalink::send(uint8_t* payload, uint8_t size, Stream &uart) {
    uart.write(this->header);
    uart.write(this->size);
    uint8_t checksum = 0;
    for(int i=0; i<size; i++) {
        uart.write(payload[i]);
        checksum += payload[i];
    }
    uart.write(byte(0xFF - checksum));
}
