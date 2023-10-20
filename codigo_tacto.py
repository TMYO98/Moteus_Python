import sys
import glob
import struct
from enum import Enum
import socket
import serial
import sys





MAMES = {
        "name"          : "MAMADAS",       # Variable sin razon alguna
        "ip"            : "10.115.2.79",   # Cambiar IP
        "port"          : 49221            # Puerto de escucha
    }


try:
    name = MAMES["name"]
    ip = MAMES["ip"]
    port = MAMES["port"]
    print("IP: {0}, PUERTO: {1}".format(ip, port))
    socketUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (ip, port)
    socketUDP.bind(server_address)
    serialCOM = serial.Serial(port = "COM13",
                            baudrate=9600,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            timeout=None)

    if not(serialCOM.isOpen()):
        serialCOM.open()
        print("ESP32@{0} abierto".format(port), serialCOM.isOpen())
    else:
        print("ESP32@{0} no se pudo abrir o ya lo estaba".format(port), serialCOM.isOpen())

    while True:
        print("####### Escuchando pura mamada #######")
        data, address = socketUDP.recvfrom(port)
        print("\n\n 2. Server received: ", data, "\n\n")
        serialCOM.write(data)
        serialCOM.write(b';')
        
except (KeyboardInterrupt,OSError):
    print("Hubo un pedo bien cabron")
    print("Huye")
    serialCOM.flushInput()
    serialCOM.close()