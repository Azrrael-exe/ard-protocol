#include <Arduino.h>
#include "datalink.h"
#include "package.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define PACKAGE_SIZE 9

#define I2C_SDA 21
#define I2C_SCL 22

#define SEALEVELPRESSURE_HPA (1013.25)

#define HEADER 0x7E

#define T_KEY 0xA4
#define H_KEY 0xA5
#define P_KEY 0xA6
#define REQUEST_KEY 0xB1

Adafruit_BME280 bme;
Datalink datalink = Datalink(HEADER, PACKAGE_SIZE);
Package incoming_package(PACKAGE_SIZE);

void printValues()
{
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

void sendData(uint8_t key)
{
    static uint16_t value;
    Package package(PACKAGE_SIZE);
    try
    {
        switch (key)
        {
        case T_KEY:
            value = (uint16_t)round(bme.readTemperature());
            break;
            case H_KEY:
            value = (uint16_t)round(bme.readTemperature());
            break;
            case P_KEY:
            value = (uint16_t)round(bme.readTemperature());
            break;        
        default:
            return;
        }
        package.addData(key, value);
    }
    catch (Exception &err)
    {
        Serial.print("Codigo de error en funcion sendData()");
        Serial.println(err.getCode());
    }

    datalink.send(package.dump(), PACKAGE_SIZE, Serial);
    Serial.println("");
}

void bmeSetup()
{
    pinMode(21, INPUT_PULLUP);
    pinMode(22, INPUT_PULLUP);
    bool status = bme.begin(0x76);
    if (!status)
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x");
        Serial.println(bme.sensorID(), 16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1)
            delay(10);
    }
}

void setup()
{
    Serial.begin(115200);
    bmeSetup();
}

void loop()
{
    //Serial.println("Hello from example/main.c");
    //printValues();
    if (Serial.available())
    {
        try
        {
            datalink.read(Serial);
            incoming_package.loads(datalink.getPayload(), PACKAGE_SIZE);  
            if (incoming_package.hasValue(REQUEST_KEY)) {
                sendData(uint8_t(incoming_package.getValue(REQUEST_KEY)));
            }
        }
        catch (Exception &err)
        {
            Serial.print("Codigo de error en funcion loop()");
            Serial.println(err.getCode());
        }
    }
    delay(1000);
}