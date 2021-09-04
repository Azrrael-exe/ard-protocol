#include <Arduino.h>
#include "datalink.h"

Datalink datalink = Datalink(0x7E, 90);

void setup() {
    Serial.begin(115200);
}

void loop() {
    Serial.println("Hello from example/main.c");
    datalink.read(Serial);
    delay(10);
}