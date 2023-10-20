## daemon Test

import threading
import asyncio
import math
import moteus
import numpy as np
from ast import Pow
from SenseOne_Lib import SenseOne
from time import sleep
import serial
import sys
import math as Math

def Plataforma():
    Arreglo =[]
    Array_P =[10,10,10,10,10,10]
    Array_XM = [0,0,0,0,0,0]
    R = .2
    Q = .006
    Magnitud = 0
    Angulo = 0
    cell = SenseOne()
    print(cell.available_ports())
    cell.open_device(COM = "COM3", baudrate = "460800",
                     parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)
    try:
        while(True):
            Arreglo = cell.payload_parsed(printPayload = False)
            #Fx, Fy, Fz, Mx, My, Mz
            #0   1   2   3   4   5
            
            for i in range(6):
                K = Array_P[i]*(1/(Array_P[i]+R))
                Array_XM[i] = Array_XM[i]+K*(Arreglo[i] - Array_XM[i])
                Array_P[i] = ((1-K)*Array_P[i])+Q   
            Magnitud = Math.sqrt(pow(Array_XM[3],2)+pow(Array_XM[4],2))   
            Angulo = Math.atan2(Array_XM[3],Array_XM[4])*180/3.141516 
            # print(Array_XM[4])
            # print("\n")
            # print(Array_XM[3])
            # print("\n")
            # print(Angulo)
            # print("\r\n\r\n\r\n")

            ##print(Angulo)
            #print(Magnitud)
            if(Magnitud<1):
                print("Center")
            elif(Angulo>= -45 and Angulo<= 45):
                print("Left")
            elif(Angulo>= 45 and Angulo<= 135):
                print("UP")
            elif(Angulo>= -135 and Angulo<= -45):
                print("Down")  
            elif(Angulo>= 135 or Angulo<= -135):
                print("Right")   
            sleep(0.01)
    except(KeyboardInterrupt):
        cell.flush_buffer()
        cell.close_device()
        sys.exit()

import asyncio
import math
import moteus
import numpy as np


async def Lisa():
    # By default, Controller connects to id 1, and picks an arbitrary
    # CAN-FD transport, prefering an attached fdcanusb if available.
    qr = moteus.QueryResolution()
    # qr._extra = {0x001: moteus.INT32,0x050: moteus.INT32, 0x043: moteus.F32, 
    # 0x044: moteus.F32,0x031: moteus.F32, 0x000: moteus.INT8, 0x031: moteus.F32, 0x030: moteus.F32} ## Config para leer n cantidad de parametros
    qr._extra = {0x001: moteus.INT32}
    c = moteus.Controller(id = 1, query_resolution = qr)

    Torque = 0 # Valor que sera el resultado del control PD
    Position = 0 # Posicion normalizada
    Error = 0  # Variable para el control PD
    Velocity = 0 #Usar este valor para generar friccion en el lisa
    KP = 1
    KD = 1
    Des_Pos = 0
    Lidar_Data = 0 # Este dato vendra del lidar (Lectura por socket) y se agregara al error, para intentar prevenir al operador
    
    
    await c.set_stop()
    #c.Mode(10)
    #state= c.make_position(position=math.nan,
    #                  velocity=1, 
    #                  feedforward_torque=3,
    #                  kp_scale=4,
    #                  kd_scale=0.20000000298023224,
    #                  maximum_torque=3,
    #                  stop_position=-1,
    #                  watchdog_timeout=100,
    #                  query=True)

    state = await c.set_position(position=math.nan, velocity = .3, 
                                    feedforward_torque = 0, stop_position = 0, maximum_torque = 0, query=True)

    
    #print("RawP:", (state.values[moteus.Register.ENCODER_0_POSITION]))
    Des_Pos = state.values[moteus.Register.POSITION] ## Trabajar con la cantidad de vueltas
    #print("POS:", int(state.values[moteus.Register.ENCODER_0_POSITION]))
    while True:
        # `set_position` accepts an optional keyword argument for each
        # possible position mode register as described in the moteus
        # reference manual.  If a given register is omitted, then that
        # register is omitted from the command itself, with semantics
        # as described in the reference manual.
        #
        # The return type of 'set_position' is a moteus.Result type.
        # It has a __repr__ method, and has a 'values' field which can
        # be used to examine individual result registers.


        # Print out everything.
        #print(state)

        # Print out just the position register.

  
            
        state = await c.set_position (position=math.nan, velocity = .3, 
                                    feedforward_torque = .1, stop_position = Des_Pos, maximum_torque = Torque,
                                    kp_scale =1, kd_scale =1 ,query=True)                                     
        Position = abs(state.values[moteus.Register.POSITION]) ## Trabajar con la cantidad de vueltas
        Velocity = abs(state.values[moteus.Register.VELOCITY]) ## Velocidad del motor
        Error = abs(Des_Pos-Position)+Lidar_Data  
        Torque = (Error*KP)+(Velocity*KD*Error)
        #print (state)
        print(Torque)

        

       
        #print("POS:", int(state.values[moteus.Register.ENCODER_0_POSITION]))

        # And a blank line so we can separate one iteration from the
        # next.

        # Wait 20ms between iterations.  By default, when commanded
        # over CAN, there is a watchdog which requires commands to be
        # sent at least every 100ms or the controller will enter a
        # latched fault state.
        await asyncio.sleep(0.02)

def main():
    Lisa = threading.Thread(target=Lisa)
    Plataforma = threading.Thread(target=Plataforma)

    Lisa.start()
    Plataforma.start()
    Plataforma.join()
    Lisa.join()
    


