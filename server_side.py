# -*- coding: utf-8 -*-
# Para leer por serial
import time
import serial

# Para la comunicación HTTP
import uvicorn 
from random import randint
from pydantic import BaseModel
 
# Abrimos la conexión con Arduino
arduino = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1.0)
with arduino:
    x = bytes({0x7E,0x5a,0xB1,0xA4,0x00})
    checksum = sum(x)
    checksum = checksum // 0xFF
    checksum = 0xFF - checksum
    x += bytes({checksum})
    bytes(0xFF - checksum)
    arduino.write(x)
    print(x)
    while True: 
        try:
            line = arduino.readline()            
            print(line)

        except KeyboardInterrupt:
            print("Exiting")
            break

        time.sleep(0.1)

