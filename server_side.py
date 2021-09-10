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
    while True:
        try:
            line = arduino.readline()            
            print(line)

        except KeyboardInterrupt:
            print("Exiting")
            break

