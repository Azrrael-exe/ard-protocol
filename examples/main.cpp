#include <Arduino.h>
#include "datalink.h"
#include "package.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define PACKAGE_SIZE 12

#define I2C_SDA         21
#define I2C_SCL         22

#define SEALEVELPRESSURE_HPA (1013.25)

#define T_KEY 0xA4
#define H_KEY 0xA5
#define P_KEY 0xA6

Adafruit_BME280 bme; 

Datalink datalink = Datalink(0x7E, 90);

void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}

void setup() {
    Serial.begin(115200);

    pinMode(21, INPUT_PULLUP);
    pinMode(22, INPUT_PULLUP);

    bool status = bme.begin(0x76);  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
}

void sendData() {
    Package package = Package(PACKAGE_SIZE);
    package.addData(T_KEY,(uint16_t)round(bme.readTemperature()));
    package.addData(H_KEY,(uint16_t)round(bme.readHumidity()));
    package.addData(P_KEY,(uint16_t)round(bme.readTemperature()));
    uint8_t* payload = package.dump();
    datalink.send(payload,PACKAGE_SIZE, Serial);
    Serial.println("");
}

void loop() {
    //Serial.println("Hello from example/main.c");
    //printValues();
    datalink.read(Serial);
    sendData();

    delay(1000);    
}