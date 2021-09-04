#include <Arduino.h>
#include "datalink.h"
#include "package.h"

#define PACKAGE_SIZE 9

Datalink datalink = Datalink(0x7E, 90);
Package package = Package(PACKAGE_SIZE);

void setup() {
    Serial.begin(115200);
}

void loop() {
    Serial.println("Hello from example/main.c");
    datalink.read(Serial);
    delay(100);    
}