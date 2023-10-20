"""
 Inbiodroid MCU UDP socket teleoperation state machine:
 This program receives UDP datagrams, containing either SM (state machine) commands
 or (position) control input data, and acts accordingly based upon current state and
 of course, the nature of the module to be controlled.
 created 19 Oct 2022
 by Inbiodroid technical team.
 This code is property of Inbiodroid.
"""

from asyncio.windows_events import NULL
from contextlib import nullcontext
from operator import truediv
#from email.message import _PayloadType
from pickle import TRUE
from pickletools import uint8
from re import S, T
from sqlite3 import connect
import time
import socket
import struct
from wsgiref import validate
from statemachine import StateMachine, State
from time import sleep
from concurrent.futures import thread
from distutils.log import error
from xmlrpc.client import Boolean
import serial
import sys
import glob
import struct
import threading
import asyncio
import math
import moteus
import numpy as np
import string
import asyncio

################ USER VARIABLES ################
offset_roll = 0
offset_Deltax = 0
offset_Ty = 0
offset_Tx = 0
offset_Fz = 0
offset_yaw = 0
offset_pich = 0 
roll=0
pitch=0
yaw=0
Fz  = 0
Tx  = 0
Ty  = 0
deltax = 0
Offset_Error = 0
Desplazamiento_x = 0
Orientacion = 1
payload = []
s = NULL
ser = NULL
Bandera_Conected = False
Bandera_Validated = False
Bandera_Calibrated = False
Bandera_Home = False
Bandera_Conected = False
Bandera_Engaged = False
Bandera_Disengaged = False
Bandera_Disconected = False
Bandera_PowerOff = False
Bandera_Bail = False
Lisa2D = 0
Lisa1D = 0
LisaX_f = 0
LisaY_f = 0
Coche_x = 0
Coche_y = 0
Coche_z = 0
Acumulador_Plataforma = []
Acumulador_Plataforma_x = [0 for i in range(15)] 
Acumulador_Plataforma_y = [0 for i in range(15)] 


lock = threading.Lock()

################ END USER VARIABLES #############


class MasterController(StateMachine):
    disconnected  = State('Disconnected', initial = True)
    connected     = State('Connected')
    validated     = State('Validated')
    calibrated    = State('Calibrated')
    home          = State('Home')
    engaged       = State('Engaged')
    disengaged    = State('Disengaged')
    powered_off   = State('Powered Off')
    
    to_connect     = disconnected.to(connected)
    to_validate    = connected.to(validated)
    to_calibrate   = validated.to(calibrated)
    goto_home      = calibrated.to(home)
    to_engage      = home.to(engaged)
    to_disengage   = engaged.to(disengaged)
    to_disconnect  = home.to(disconnected)
    gobackto_home  = disengaged.to(home)
    to_power_off   = disconnected.to(powered_off)
    
    to_abortfcn    = connected.to(disconnected)
    to_abortfva    = validated.to(disconnected)
    to_abortfca    = calibrated.to(disconnected)
    to_abortfho    = home.to(disconnected)
    to_abortfen    = engaged.to(disconnected)
    to_abortfdn    = disengaged.to(disconnected)

controller = MasterController()

hostIP         = '0.0.0.0'          #OUR OWN ADDRESS
peerIP         = '10.115.1.111'     #OMNIWHEELS SYSTEM'S  IP ADDRESS
masterIP       = '10.115.2.51'      #MASTER'S             IP ADDRESS

localPort      = 0xC04F             #OWN (LISA) PORT
peerPort       = 0xC06F             #OMNIWHEEL'S PORT
masterPort     = 0xC033             #MASTER'S PORT

localDeviceId  = 0x4F4F             #LISA'S DEVICE ID
peerDeviceId   = 0x416F             #OMNIWHEEL'S DEVICE ID
masterDeviceId = 0x4D33             #MASTER'S DEVICE ID

#### testing config
# hostIP         = '0.0.0.0'     
# peerIP         = '16.221.69.123'
# masterIP       = '16.221.69.17'

# localPort      = 0xC010
# peerPort       = 0xC07B
# masterPort     = 0xC011

# localDeviceId  = 0x4110
# peerDeviceId   = 0x4F7B
# masterDeviceId = 0x4D11


udpsocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udpsocket.bind((hostIP,localPort))
udpsocket.settimeout(0.000001)

################ GLOBALS ################

move_along_y = True
rotate_on_z = False

move_torso = False
move_omniwheels = True

m_gapWideningCount = 0

delta = 0
lastDelta = 0

rx_DatagramSize = 24
tx_DatagramSize = 36

txDatagram = bytearray(tx_DatagramSize)
rxDatagram = bytearray(rx_DatagramSize)

data_is_ready    =     False
extern_device    =      0x33
extern_IPaddress =  (0,0,0,0)
extern_deviceID  =      (0,0)

opc_l = 0x00
tag_l = 0x01
dID_l = 0x06
cmd_l = 0x08
rcd_l = 0x09
tst_l = 0x0A
pld_l = 0x0E
rx_crc_l = rx_DatagramSize-2
tx_crc_l = tx_DatagramSize-2

tx_pld_q1_l = 0x0E
tx_pld_q2_l = 0x12
tx_pld_q3_l = 0x16
tx_pld_q4_l = 0x1A
tx_pld_q5_l = 0x1E

txQ1 = bytearray(4)
txQ2 = bytearray(4)
txQ3 = bytearray(4)
txQ4 = bytearray(4)
txQ5 = bytearray(4)

rx_pld_q1_l = 0x0E
rx_pld_q2_l = 0x12

rxQ1 = bytearray(4)
rxQ2 = bytearray(4)

outputData1 = 0.0
outputData2 = 0.0
outputData3 = 0.0
outputData4 = 0.0
outputData5 = 0.0

inputData1 = 0.0
inputData2 = 0.0

start_ref_time = time.perf_counter()
lastTimeStamp = 0
old_udps = 0
timeStamp_int  = 0
timeStamp_tx = 0
tx_timeStamp_byte = bytearray(4)
rx_timeStamp_byte = bytearray(4)

rx_CRC_byte = bytearray(2)
tx_CRC_byte = bytearray(2)

################ OTHER FUNCTIONS ################

def timestamp_processing():
    actualtime = (time.perf_counter() - start_ref_time) * 10**6
    elapsedTime = bytearray(struct.pack("f",actualtime))
    return elapsedTime

def message_counting():
    global tx_timeStamp_byte
    global timeStamp_tx
    if(controller.is_engaged):    
        timeStamp_tx += 1
        # print("tx_TimeStamp: ", timeStamp_tx)
        tx_timeStamp_byte = timeStamp_tx.to_bytes(4,'little',signed=False)
        # print("tx_timeStamp_byte: ", tx_timeStamp_byte[0],tx_timeStamp_byte[1],tx_timeStamp_byte[2],tx_timeStamp_byte[3])
    return tx_timeStamp_byte

def calculateCRC(datagram, datagramsize):
    crc = 0xFFFF
    pos = 0
    while pos < datagramsize-2:
        val = datagram[pos] & (0xFF)
        # print("Val[", pos,"]: ",hex(val)," - buffer: ", datagram[pos])
        crc ^= val
        # print("crc: ", hex(crc))
        i = 8
        while i !=0:
            if (crc & 0x0001) !=0:
                crc >>= 1
                crc ^= 0x4557
                # print("crc[",i,"] - crc & 0x0001 != 0:", hex(crc))
            else:
                crc >>= 1
                # print("crc[",i,"] - crc & 0x0001 == 0:", hex(crc))
            i -= 1
        pos +=1
    # print("crc final: ",hex(crc))
    return crc

################ UDP MESSAGE - RECEIVE & SNED ################

def receiveDatagram():
    global rxDatagram
    global timeStamp_int
    global rx_timeStamp_byte
    while True:
        try:
            data = udpsocket.recvfrom(rx_DatagramSize)
            rxDatagram = data[0]
            if data:
                tag = (data[0][tag_l],data[0][tag_l+1],data[0][tag_l+2],data[0][tag_l+3],data[0][tag_l+4])
                # rx_timeStamp_byte[0] = rxDatagram[tst_l]
                # rx_timeStamp_byte[1] = rxDatagram[tst_l+1]
                # rx_timeStamp_byte[2] = rxDatagram[tst_l+2]
                # rx_timeStamp_byte[3] = rxDatagram[tst_l+3]
                # timeStamp_int = int.from_bytes(rx_timeStamp_byte, "little", signed=False)
                rx_CRC_byte[0] = rxDatagram[rx_crc_l]
                rx_CRC_byte[1] = rxDatagram[rx_crc_l+1]
                rx_CRC_int = int.from_bytes(rx_CRC_byte, "little", signed=False)
                #print("Received timestamp", timeStamp_int)
                if(validateCRC(rx_CRC_int) == False):
                    
                    return (False, data[0], 0)
                elif(validateTag(tag) == False):
                    return (False, data[0], 0)
                # elif(rxDatagram[cmd_l] == 0x40 and validateTimestamp(timeStamp_int) == False):
                #     global old_udps
                #     old_udps += 1
                #     return (False, data[0], 0)
                elif(validateCommand(data[0][cmd_l]) == False):
                    return (False, data[0], 0)
                else:
                    extern_deviceID = (data[0][dID_l],data[0][dID_l+1])
                    extern_device = data[1][0].split(".")[3]
                    extern_IPaddress = data[1][0]
                    extern_data = [extern_deviceID, extern_device, extern_IPaddress, data[0][cmd_l]]
                    return (True, data[0], extern_data)
        except socket.timeout:
            sleep(0.0001)
            #print("................................................................")
            if(controller.is_engaged):
                txDatagram[rcd_l] = control()
                global data_is_ready
                if(data_is_ready == True):
                    updateOutgoingData()
                    sendDatagram(0x00, data_for_reply)
                    data_is_ready = False
            continue
    
def sendDatagram(MessageType, dataforextern):
    global portToSend
    global addressToSend
    txDatagram[tag_l+0] = 0x6F
    txDatagram[tag_l+1] = 0x52
    txDatagram[tag_l+2] = 0x63
    txDatagram[tag_l+3] = 0x69
    txDatagram[tag_l+4] = 0x6E
    
    if(MessageType==0x00):      #TELEOPERATION_DATA
        txDatagram[opc_l]   = 0x00    #OPC
        
        txDatagram[dID_l]   = 0x41    #Device ID - 'A' for Avatar - peer to connect
        txDatagram[dID_l+1] = 0x7B    #Device ID - 'o' for Omniwheels - peer to connect
        
        txDatagram[cmd_l]   = 0x40
        
        portToSend = peerPort
        addressToSend = peerIP
        
    elif(MessageType==0x01):    #EXCEPTION
        txDatagram[opc_l]   = 0x00    #OPC
        
        txDatagram[dID_l]   = 0x41    #Device ID - 'O' for Operator - master to connect
        txDatagram[dID_l+1] = 0x33    #Device ID - '3' for Master - master to connect
        
        txDatagram[cmd_l]   = 0x5E
        
        portToSend = masterPort
        addressToSend = masterIP
        
    elif(MessageType==0x02):    #STATEMACHINE_ACK
        txDatagram[opc_l]   = 0x10    #OPC
        
        txDatagram[dID_l]   = dataforextern[0][0]
        txDatagram[dID_l+1] = dataforextern[0][1]
        
        txDatagram[cmd_l]   = dataforextern[3]
        
        addressToSend = dataforextern[2]
        portToSend = 0xC000 + int(dataforextern[1])
    
    timestamp = message_counting()
    
    txDatagram[tst_l+0] = timestamp[0]
    txDatagram[tst_l+1] = timestamp[1]
    txDatagram[tst_l+2] = timestamp[2]
    txDatagram[tst_l+3] = timestamp[3]
    
    intCRC = calculateCRC(txDatagram,tx_DatagramSize)
    byteCRC = intCRC.to_bytes(2,'little')
    
    txDatagram[tx_crc_l+0] = byteCRC[0]
    txDatagram[tx_crc_l+1] = byteCRC[1]
    # print("Sending to: ", (addressToSend, portToSend))
    udpsocket.sendto(txDatagram,(addressToSend, portToSend))

################ UDP MESSAGE VALIDATION ################ 

    
def validateTag(rxTag):
    tag=(ord('o'), ord('R'), ord('c'), ord('i'), ord('n'))
    if(rxTag == tag):
        return True
    else:
        print("Invalid Tag: ", chr(rxTag[0]), chr(rxTag[1]), chr(rxTag[2]), chr(rxTag[3]), chr(rxTag[4]))
        return False
    
# # # def validateTimestamp(rxTstmp):
# # #     global lastTimeStamp
# # #     if(rxTstmp > lastTimeStamp or rxTstmp == 0):
# # #         lastTimeStamp = rxTstmp
# # #         return True
# # #     else:
# # #         print("Invalid TimeStamp")
# # #         return False

def validateTimestamp(rxTstmp):
    global lastTimeStamp
    global m_gapWideningCount
    delta = rxTstmp - lastTimeStamp
    if(delta < 0x7FFFFFFF):
        lastTimeStamp = rxTstmp
        return True
    
    else:
        m_gapWideningCount += 1
        return False

def validateCommand(rxCmd):
    if((rxCmd == 0x06) or (rxCmd == 0x21) or (rxCmd == 0x22) or (rxCmd == 0x23) or (rxCmd == 0x24) or (rxCmd == 0x25) or (rxCmd == 0x26) or (rxCmd == 0x27) or (rxCmd == 0x28) or (rxCmd == 0x40) or (rxCmd == 0x11) or (rxCmd == 0x2a) or (rxCmd == 0x30) or (rxCmd == 0x5E)):
        return True
    else:
        print("Invalid Command: ", hex(rxCmd))
        return False
    
def validateCRC(rxCRC):
    if rxCRC == calculateCRC(rxDatagram,rx_DatagramSize):
        return True
    else:
        return False

################ USER DEF ################
def map( x,  in_min,  in_max,  out_min,  out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def Calculo_XY(enc1,enc2,enc3,enc4):
    global LisaX_f 
    global LisaY_f
    global Lisa1D
    global Lisa2D
    Lisa1X = Lisa1D*math.cos(math.radians(enc1))*math.cos(math.radians(enc3))
    Lisa1Y = Lisa1D*math.cos(math.radians(enc3))*math.sin(math.radians(enc1))
    Lisa2X = Lisa2D*math.cos(math.radians(enc2))*math.cos(math.radians(enc4))
    Lisa2Y = Lisa2D*math.cos(math.radians(enc4))*math.sin(math.radians(enc2))
    LisaX_f = (Lisa1X+Lisa2X)/2
    LisaY_f = (Lisa1Y+Lisa2Y)/2

def read_IMU():
    global roll
    global pitch
    global yaw  
    global deltax 
    global ser
    global Bandera_Disconected
    global Bandera_PowerOff
    
    PosLisa=[]
    roll = 0
    roll1 = 0
    yaw1 = 0
    enc1 = 0
    enc2 = 0
    enc3 = 0
    enc4 = 0
    while(1):
        if(Bandera_Disconected or Bandera_PowerOff):
            ser.flushInput()
            ser.close()

            break
        else:
            line = ser.readline() #Obtiene dato del serial
        
        decoded_bytes = line.decode('utf-8','ignore') #Decodifica los datos del serial
        data = (decoded_bytes.replace('\r','')).replace('\n','') #Elimina los retorno de carro
        words = data.split(",") # Agrega coma entre cada dato
            # Particion del mensaje
        if len(words) >= 6:
            try:
                roll = float(words[0]) #Rotacion en x Agacharte
                pitch = float(words[1]) # Rotacion en Y
                yaw1 = float(words[2]) #Rotacion en Z 
                enc1=float(words[3])
                enc2=float(words[4])
                enc3=float(words[5])
                        ###
                enc4=float(words[6])
                        ###
                        # enc4=float(words[6])  #este es el de verdad :V !!!
                        # Array_Torso_Carrito[3]=yaw
                        # Array_Torso_Carrito[4]=roll
            except:
                print ("Invalid line")

            # lisa1 = 0.70 # Longitud del lisa derecho (Posicion de brazo derecho de operador)
            # lisa2 = 0.70 # Longitud del lisa izquierdo (Posicion de brazo izquierdo de operador)
            ## Convertir a radianesl
            # Calculo_XY(enc1,enc2,enc3,enc4)

            
            #deltax = (-lisa1*math.sin(math.radians(enc1))*math.cos(math.radians(enc3)) - lisa2*math.sin(math.radians(enc2))*math.cos(math.radians(enc4)))/2 # Diferencial en x
                    # Array_Torso_Carrito[1]=deltax
                    #deltay = (-lisa1*math.cos(math.radians(enc1))*math.sin(math.radians(enc3)) - lisa2*math.cos(math.radians(enc2))*math.sin(math.radians(enc4)))/2 # Probar esta ecuacion (Diferencial en y)
            # if(yaw1<0):
            #     yaw1+=360 
            yaw = yaw1 
            
def read_Plataforma():
    global Coche_x
    global Coche_y
    global Coche_z
    print("aqui")
    while(1):
        line1 = s.readline() #Obtiene dato del serial
        print("line1")        
        decoded_bytes = line1.decode('utf-8','ignore') #Decodifica los datos del serial
        data = (decoded_bytes.replace('\r','')).replace('\n','') #Elimina los retorno de carro
        words = data.split(",") # Agrega coma entre cada dato
        # Particion del mensaje
        if len(words) >= 4:
            try:
                Up_Left = float(words[0]) #Rotacion en x Agacharte
                Up_Right = float(words[1]) # Rotacion en Y
                Down_Right = float(words[2]) #Rotacion en Z 
                Down_Left=float(words[3])
                if(Up_Left == 1 and Up_Right == 1 and Down_Right == 0 and Down_Left == 0):
                    print("adelante")
                    Coche_x = .55
                    Coche_y = 0
                    Coche_z = 0
                elif(Up_Left == 0 and Up_Right == 0 and Down_Right == 0 and Down_Left == 0):
                    print("Nada")
                    Coche_x = 0
                    Coche_y = 0
                    Coche_z = 0
                elif(Up_Left == 0 and Up_Right == 1 and Down_Right == 0 and Down_Left == 0):
                    print("Lateral_Derecha") 
                    Coche_x = 0
                    Coche_y = -.55
                    Coche_z = 0
                elif(Up_Left == 1 and Up_Right == 0 and Down_Right == 0 and Down_Left == 0):
                    print("Lateral_Izquierda")  
                    Coche_x = 0
                    Coche_y = .55
                    Coche_z = 0
                elif(Down_Left == 1 and Down_Right == 1):
                    print("Atras")  
                    Coche_x = -.7
                    Coche_y = 0
                    Coche_z = 0
                elif(Down_Left == 1 and Up_Right == 1):
                    print("Rotacion_Derecha") 
                    Coche_x = 0
                    Coche_y = 0
                    Coche_z = .5
                elif(Up_Left == 1 and Down_Right == 1):
                    print("Rotacion_Izquieda") 
                    Coche_x = 0
                    Coche_y = 0
                    Coche_z = -.5
                else:
                    print("Nada")
                    Coche_x = 0
                    Coche_y = 0
                    Coche_z = 0
            except:
                print ("Invalid line")

def map_Data(Inner_negative_limit, Inner_positive_limit, Out_negative_limit, Out_positive_limit, Data, Data_min, Data_max):

    data_to_return = 0
    Data = map(Data,Data_min,Data_max,Out_negative_limit,Out_positive_limit) #Se mapea para trabajar con los rangos predefinidos
    if(Data<Inner_positive_limit and Data>Inner_negative_limit): 
        ## "Rango de operacion natural del operador", hay lecturas de movimiento pero el robot no se debe de mover
        Data = 0
    elif(Data>Inner_positive_limit): 
        ## Al sobrepasar el rango natural del operador, el robot se mueve, se le agrega un ofset para compensar
        ## el rango que se le quito ("Rango de operacion natural del operador")
        Data += Inner_negative_limit
    elif(Data<Inner_negative_limit):
        ## Al sobrepasar el rango natural del operador, el robot se mueve, se le agrega un ofset para compensar
        ## el rango que se le quito ("Rango de operacion natural del operador")
        Data += Inner_positive_limit
    
    if(Data>Out_positive_limit):
        data_to_return=Out_positive_limit #Se satura el dato en caso de que sobrepase los limites asignados
    elif(Data<Out_negative_limit):
        data_to_return=Out_negative_limit #Se satura el dato en caso de que sobrepase los limites asignados
    else:
        data_to_return = Data
    return data_to_return


############## END USER DEF ##############

################ CONTROL ################

def control():
    global offset_Deltax
    global offset_Fz
    global offset_Tx 
    global offset_Ty
    global offset_yaw
    global offset_roll  
    global roll
    global pitch
    global yaw  
    global deltax 
    global Fz  
    global Tx
    global Ty 
    global Desplazamiento_x
    global rotate_on_z
    global move_along_y
    global move_torso
    global move_omniwheels
    global Offset_Error
    global LisaX_f
    global LisaY_f
    global Coche_y
    global Coche_x
    global Coche_z
    global Bandera_Home

    global outputData1
    global outputData2
    global outputData3
    global outputData4
    global outputData5
    
    global inputData1
    global inputData2
    global data_is_ready

    sleep(.01)

    Tx_Set = (Tx - offset_Tx)
    Ty_Set = (Ty - offset_Ty)
    Fz_Set = Fz - offset_Fz
    DeltaX = deltax - offset_Deltax
    Roll_set = roll - offset_roll
    Yaw_set = yaw - offset_yaw
    Torso_y = 0
    Torso_z = 0





    Torso_y = (map_Data(-1.4,2,-10,10,(Roll_set*-1),-18,18) * math.pi /180 ) ##agachado
    #     #outputData1 = Roll_set 
    Torso_z = (map_Data(-3.5,3.5,-30,30,Yaw_set,-20,20) * math.pi /180 ) ##Giro_torso
    #     #outputData2 = Yaw_set    #Giro_Torso






    outputData1 = Torso_y  ## agachado
    outputData2 = Torso_z
 
    outputData3 = Coche_x #Adelante
    outputData4 = Coche_y #Costados
    outputData5 = Coche_z

    # outputData1 = 0
    # outputData2 = 0
    # outputData3 = 0
    # outputData4 = 0
    # outputData5 = 0

    
    
    data_is_ready = True
    
    return 0x00

################ STATE MACHINE CONTROL ################

def connect_f():
    global Bandera_Disconected 
    global Bandera_Bail
    if(Bandera_Bail):
        return 0x00
    global s
    global ser
    global Bandera_Conected
    Bandera_Disconected = False
    # s = serial.Serial("COM12",
    #             baudrate=115200,
    #             parity=serial.PARITY_NONE,
    #             stopbits=serial.STOPBITS_ONE) ##sensOne
    ser = serial.Serial(port='COM10',baudrate=9600, timeout=1) ## IMU

    
    # try:
    #     if not s.isOpen():
    #         print("s")
    #         s.open()
    #         print("Puerto  Plataforma abierto")      
    # except:
    #     return 0x02 #transition_ok

    try:
        if not ser.isOpen():
            print("s")
            ser.open()
            print("Puerto  IMU abierto")      
    except:
        return 0x02 #transition_ok
    sleep(4)
    Bandera_Conected = True

    return 0x00 #transition_ok
    
def validate_f():
    # global Bandera_Bail
    # if(Bandera_Bail):
    #     return 0x00
    # global Bandera_Validated
    # Bandera_Validated = True
    Plataforma_thread = threading.Thread(target= read_Plataforma)
    IMU_thread = threading.Thread(target= read_IMU)
    # Plataforma_thread.start()
    IMU_thread.start()
    sleep(2)

    return 0x00
    
def calibrate_f():
    global Bandera_Bail
    if(Bandera_Bail):
        return 0x00
    iteraciones = 15
    global Bandera_Calibrated
    global offset_Deltax
    global offset_Fz
    global offset_Tx 
    global offset_Ty
    global offset_yaw
    global offset_roll  
    global roll
    global pitch
    global yaw  
    global deltax 
    global Fz  
    global Tx
    global Ty 


    i = 0
    for i in range (iteraciones):
        sleep(.05)
        offset_roll += roll
        offset_Deltax += deltax
        offset_yaw += yaw
        offset_Tx += Tx
        offset_Ty += Ty
        offset_Fz += Fz
    
    offset_roll = offset_roll/iteraciones
    offset_Deltax = offset_Deltax/iteraciones
    offset_yaw = offset_yaw/iteraciones
    offset_Fz = offset_Fz/iteraciones
    offset_Tx = offset_Tx/iteraciones
    offset_Ty = offset_Ty/iteraciones

    print(offset_Deltax)
    print(offset_yaw)
    print(offset_roll)
    print(offset_Fz)
    print(offset_Tx)
    print(offset_Ty)



    Bandera_Calibrated = True


    return 0x00
    
def goto_home_f():
    global Bandera_Bail
    if(Bandera_Bail):
        return 0x00
    global Bandera_Home
    Bandera_Home = True

    return 0x00

def gobackto_home_f():
    global Bandera_Bail
    if(Bandera_Bail):
        return 0x00
    global Bandera_Home
    Bandera_Home = True
    print("TEST HOME")
    return 0x00
    
def engage_f():
    global Bandera_Bail
    if(Bandera_Bail):
        return 0x00
    global Bandera_Engaged
    global Bandera_Home
    Bandera_Home = False
    Bandera_Engaged = True
    return 0x00
    
def disengage_f():
    global Bandera_Bail
    if(Bandera_Bail):
        return 0x00
    global Bandera_Engaged
    global Bandera_Home
    global Bandera_Disengaged
    Bandera_Home = False
    Bandera_Engaged = False
    Bandera_Disengaged = True

    return 0x00
    
def disconnect_f():
    global s
    global ser
    global Bandera_Conected 
    global Bandera_Validated 
    global Bandera_Calibrated 
    global Bandera_Home 
    global Bandera_Conected 
    global Bandera_Engaged 
    global Bandera_Disengaged
    global Bandera_Disconected

    Bandera_Conected = False
    Bandera_Validated = False
    Bandera_Calibrated = False
    Bandera_Home = False
    Bandera_Conected = False
    Bandera_Engaged = False
    Bandera_Disengaged = False


    Bandera_Disconected = True
    return 0x00

def poweroff_f():
    global Bandera_PowerOff
    global Bandera_Disconected
    Bandera_PowerOff = True 
    Bandera_Disconected = False

    sys.exit()
    return 0x00

def bail_f():
    global Bandera_Engaged
    global Bandera_Home
    global Bandera_Disengaged
    global s
    global ser
    Bandera_Home = False
    Bandera_Engaged = False
    Bandera_Disengaged = True
    global Bandera_Bail
    Bandera_Bail = True
    s.flushInput()
    s.close()
    ser.flushInput()
    ser.close()
    if(controller.is_disconnected):
        print("Already DISCONNECTED")
    elif(controller.is_connected):
        controller.to_abortfcn()
    elif(controller.is_validated):
        controller.to_abortfva()
    elif(controller.is_calibrated):
        controller.to_abortfca()
    elif(controller.is_home):
        controller.to_abortfho()
    elif(controller.is_engaged):
        controller.to_abortfen()
    elif(controller.is_disengaged):
        controller.to_abortfdn()
    ##
    return 0x00

def updateIncomingData():
    global inputData1
    global inputData2
    
    rxQ1[0] = rxDatagram[rx_pld_q1_l+0]
    rxQ1[1] = rxDatagram[rx_pld_q1_l+1]
    rxQ1[2] = rxDatagram[rx_pld_q1_l+2]
    rxQ1[3] = rxDatagram[rx_pld_q1_l+3]
    
    rxQ2[0] = rxDatagram[rx_pld_q2_l+0]
    rxQ2[1] = rxDatagram[rx_pld_q2_l+1]
    rxQ2[2] = rxDatagram[rx_pld_q2_l+2]
    rxQ2[3] = rxDatagram[rx_pld_q2_l+3]
    
    i1 = struct.unpack('<f', rxQ1)
    i2 = struct.unpack('<f', rxQ2)
    inputData1 = i1[0]
    inputData2 = i2[0]
    
    print("i1: ", inputData1,"i2: ", inputData2)
    
    return

def updateOutgoingData():
    global outputData1
    global outputData2
    global outputData3
    global outputData4
    global outputData5
    
    txQ1 = bytearray(struct.pack("f", outputData1))
    txQ2 = bytearray(struct.pack("f", outputData2))
    txQ3 = bytearray(struct.pack("f", outputData3))
    txQ4 = bytearray(struct.pack("f", outputData4))
    txQ5 = bytearray(struct.pack("f", outputData5))
    
    txDatagram[tx_pld_q1_l+0] = txQ1[0]
    txDatagram[tx_pld_q1_l+1] = txQ1[1]
    txDatagram[tx_pld_q1_l+2] = txQ1[2]
    txDatagram[tx_pld_q1_l+3] = txQ1[3]
    
    txDatagram[tx_pld_q2_l+0] = txQ2[0]
    txDatagram[tx_pld_q2_l+1] = txQ2[1]
    txDatagram[tx_pld_q2_l+2] = txQ2[2]
    txDatagram[tx_pld_q2_l+3] = txQ2[3]
    
    txDatagram[tx_pld_q3_l+0] = txQ3[0]
    txDatagram[tx_pld_q3_l+1] = txQ3[1]
    txDatagram[tx_pld_q3_l+2] = txQ3[2]
    txDatagram[tx_pld_q3_l+3] = txQ3[3]
    
    txDatagram[tx_pld_q4_l+0] = txQ4[0]
    txDatagram[tx_pld_q4_l+1] = txQ4[1]
    txDatagram[tx_pld_q4_l+2] = txQ4[2]
    txDatagram[tx_pld_q4_l+3] = txQ4[3]
    
    txDatagram[tx_pld_q5_l+0] = txQ5[0]
    txDatagram[tx_pld_q5_l+1] = txQ5[1]
    txDatagram[tx_pld_q5_l+2] = txQ5[2]
    txDatagram[tx_pld_q5_l+3] = txQ5[3]
    
    print("o1: ",outputData1,"o2: ",outputData2,"o3: ",outputData3,"o4: ",outputData4,"o5: ",outputData5)
    
    return

################ STATE MACHINE TRANSITIONS ################

def Maquina_De_Estados():

    global Bandera_Conected 
    global Bandera_Validated 
    global Bandera_Calibrated 
    global Bandera_Home 
    global Bandera_Engaged 
    global Bandera_Disengaged
    global Bandera_Disconected


    while True:
        global data_for_reply
        global move_along_y
        global rotate_on_z

        global move_torso
        global move_omniwheels
        #timestamp_processing()
        receivedDatagram = receiveDatagram()
        isreceived = receivedDatagram[0]
        data = receivedDatagram[1]
        data_for_reply = receivedDatagram[2]
        if(isreceived):
            if(data[cmd_l] == 0x06):
                sendDatagram(0x02,data_for_reply)
                print("Message sent")
                
            elif(data[cmd_l] == 0x21):
                if(controller.is_disconnected):
                    print("Changing to CONNECT")
                    if(connect_f() == 0x00):
                        txDatagram[rcd_l]=0x00
                        controller.to_connect()
                        print("CONNECTED")
                    else:
                        txDatagram[rcd_l]=0x03
                        print("Failed to CONNECT")
                elif(controller.is_connected):
                    print("Already CONNECTED")
                    txDatagram[rcd_l] = 0x01
                else:
                    print("Not a valid command for this state")
                    txDatagram[rcd_l] = 0x02
                sendDatagram(0x02,data_for_reply)
                    
            elif(data[cmd_l] == 0x22):
                if(controller.is_connected):
                    print("Changing to VALIDATED")
                    if(validate_f() == 0x00):
                        txDatagram[rcd_l]=0x00
                        controller.to_validate()
                        print("VALIDATED")
                    else:
                        txDatagram[rcd_l]=0x03
                        print("Failed to VALIDATE")
                elif(controller.is_validated):
                    print("Already VALIDATED")
                    txDatagram[rcd_l] = 0x01
                else:
                    print("Not a valid command for this state")
                    txDatagram[rcd_l] = 0x02
                sendDatagram(0x02,data_for_reply)
                    
            elif(data[cmd_l] == 0x23):
                if(controller.is_validated):
                    print("Changing to CALIBRATED")
                    if(calibrate_f() == 0x00):
                        txDatagram[rcd_l]=0x00
                        controller.to_calibrate()
                        print("CALIBRATED")
                    else:
                        txDatagram[rcd_l]=0x03
                        print("Failed to CALIBRATE")
                elif(controller.is_calibrated):
                    print("Already CALIBRATED")
                    txDatagram[rcd_l] = 0x01
                else:
                    print("Not a valid command for this state")
                    txDatagram[rcd_l] = 0x02
                sendDatagram(0x02,data_for_reply)
                    
            elif(data[cmd_l] == 0x24):
                if(controller.is_calibrated):
                    print("Changing to HOME")
                    if(goto_home_f() == 0x00):
                        txDatagram[rcd_l]=0x00
                        controller.goto_home()
                        print("AT HOME")
                    else:
                        txDatagram[rcd_l]=0x03
                        print("Failed going HOME")        
                elif(controller.is_disengaged):
                    print("Changing to HOME")
                    if(gobackto_home_f() == 0x00):
                        txDatagram[rcd_l]=0x00
                        controller.gobackto_home()
                        print("AT HOME")
                    else:
                        txDatagram[rcd_l]=0x03
                        print("Failed returning HOME")
                elif(controller.is_home):
                    print("Already AT HOME")
                    txDatagram[rcd_l] = 0x01
                else:
                    print("Not a valid command for this state")
                    txDatagram[rcd_l] = 0x02
                sendDatagram(0x02,data_for_reply)
                
            elif(data[cmd_l] == 0x25):
                if(controller.is_home):
                    print("Changing to ENGAGED")
                    if(engage_f() == 0x00):
                        txDatagram[rcd_l]=0x00
                        controller.to_engage()
                        #lastTimeStamp = 0
                        print("ENGAGED")
                    else:
                        txDatagram[rcd_l]=0x03
                        print("Failed to ENGAGE")
                elif(controller.is_engaged):
                    print("Already ENGAGED")
                    txDatagram[rcd_l] = 0x01
                else:
                    print("Not a valid command for this state")
                    txDatagram[rcd_l] = 0x02
                move_along_y = True
                rotate_on_z = False
                move_torso = False
                move_omniwheels = True
                sendDatagram(0x02,data_for_reply)
                
            elif(data[cmd_l] == 0x26):
                if(controller.is_engaged):
                    print("Changing to DISENGAGED")
                    if(disengage_f() == 0x00):
                        txDatagram[rcd_l]=0x00
                        controller.to_disengage()
                        print("DISENGAGED")
                    else:
                        txDatagram[rcd_l]=0x03
                        print("Failed to DISENGAGE")
                elif(controller.is_disengaged):
                    print("Already DISENGAGED")
                    txDatagram[rcd_l] = 0x01
                else:
                    print("Not a valid command for this state")
                    txDatagram[rcd_l] = 0x02
                sendDatagram(0x02,data_for_reply)
                
            elif(data[cmd_l] == 0x27):
                if(controller.is_home):
                    print("Changing to DISCONNECTED")
                    if(disconnect_f() == 0x00):
                        txDatagram[rcd_l]=0x00
                        controller.to_disconnect()
                        print("DISCONNECTED")
                    else:
                        txDatagram[rcd_l]=0x03
                        print("Failed to DISCONNECT")
                elif(controller.is_disconnected):
                    print("Already DISCONNECTED")
                    txDatagram[rcd_l] = 0x01
                else:
                    print("Not a valid command for this state")
                    txDatagram[rcd_l] = 0x02
                sendDatagram(0x02,data_for_reply)
                
            elif(data[cmd_l] == 0x28):
                if(controller.is_disconnected):
                    print("Powering Off...")
                    if(poweroff_f() == 0x00):
                        txDatagram[rcd_l]=0x00
                        controller.to_power_off()
                        print("POWERED OFF")
                    else:
                        txDatagram[rcd_l]=0x03
                        print("Failed to POWER OFF")
                elif(controller.is_powered_off):
                    print("Already POWERED OFF")
                    txDatagram[rcd_l] = 0x01
                else:
                    print("Not a valid command for this state")
                    txDatagram[rcd_l] = 0x02
                sendDatagram(0x02,data_for_reply)
                    
            elif(data[cmd_l] == 0x40):
                if(controller.is_engaged):
                    rx_timeStamp_byte[0] = data[tst_l]
                    rx_timeStamp_byte[1] = data[tst_l+1]
                    rx_timeStamp_byte[2] = data[tst_l+2]
                    rx_timeStamp_byte[3] = data[tst_l+3]
                    timeStamp_int = int.from_bytes(rx_timeStamp_byte, "little", signed=False)
                    if(validateTimestamp(timeStamp_int) == True):
                        updateIncomingData()
                    elif(m_gapWideningCount > 1000):
                        lastTimeStamp = timeStamp_int
                else:
                    print("Not a valid command for this state")
                sendDatagram(0x00,data_for_reply)
                
            elif(data[cmd_l] == 0x11): 
                if(controller.is_engaged):
                    if(data[rcd_l] == 0x01):
                        if(rotate_on_z == True):
                            print("Already rotating on z")
                            txDatagram[rcd_l] = 0x01
                        else:
                            print("Rotating on z")
                            txDatagram[rcd_l] = 0x00
                        rotate_on_z = True
                        move_along_y = False
                        
                    elif(data[rcd_l] == 0x02):
                        if(move_along_y == True):
                            print("Already moving on y")
                            txDatagram[rcd_l] = 0x01
                        else:
                            print("Moving along y")
                            txDatagram[rcd_l] = 0x00
                        rotate_on_z = False
                        move_along_y = True
                        
                    # elif(data[rcd_l] == 0x03):
                    #     rotate_on_z == True
                    #     move_along_y == True
                    #     print("move all axis")
                    
                    elif(data[rcd_l] == 0x03):
                        if(move_torso == True):
                            print("Already moving torso")
                            txDatagram[rcd_l] = 0x01
                        else:
                            print("Moving torso")
                            txDatagram[rcd_l] = 0x00
                        move_torso = True
                        move_omniwheels = False
                        
                    elif(data[rcd_l] == 0x04):
                        if(move_omniwheels == True):
                            print("Already moving wheels")
                            txDatagram[rcd_l] = 0x01
                        else:
                            print("Moving wheels")
                            txDatagram[rcd_l] = 0x00
                        move_torso = False
                        move_omniwheels = True

                txDatagram[cmd_l] = 0x11
                sendDatagram(0x02,data_for_reply)
            
            elif(data[cmd_l] == 0x2A):
                #return 
                print("Changing to DISCONNECTED (ABORTING)")
                txDatagram[rcd_l] == bail_f()
                sendDatagram(0x02,data_for_reply)

                
            elif(data[cmd_l] == 0x29):
                #return
                print("")
                
            else:
                print("COMMAND UNKNOWN")

async def main():
    while(1):
        # Torque = 0 # Valor que sera el resultado del control PD
        # Position = 0 # Posicion normalizada
        # Error = 0  # Variable para el control PD
        # Velocity = 0 #Usar este valor para generar friccion en el lisa
        # KP = 10 
        # KD = 15
        # Des_Pos = 0
        # Lidar_Data = 0 # Este dato vendra del lidar (Lectura por socket) y se agregara al error, para intentar prevenir al operador
        # Estado = 0

        # Torque2 = 0 # Valor que sera el resultado del control PD
        # Position2 = 0 # Posicion normalizada
        # Error2 = 0  # Variable para el control PD
        # Velocity2 = 0 #Usar este valor para generar friccion en el lisa
        # KP2 = 10
        # KD2 = 15
        # Des_Pos2 = 1
        # Lidar_Data2 = 0 # Este dato vendra del lidar (Lectura por socket) y se agregara al error, para intentar prevenir al operador 

        # qr = moteus.QueryResolution()
        # # qr._extra = {0x001: moteus.INT32,0x050: moteus.INT32, 0x043: moteus.F32, 
        # # 0x044: moteus.F32,0x031: moteus.F32, 0x000: moteus.INT8, 0x031: moteus.F32, 0x030: moteus.F32} ## Config para leer n cantidad de parametros
        # qr._extra = {0x001: moteus.INT32}
        global Bandera_Conected 
        global Bandera_Validated 
        global Bandera_Calibrated 
        global Bandera_Home 
        global Bandera_Conected 
        global Bandera_Engaged 
        global Bandera_Disengaged
        global Bandera_Disconected
        global offset_Lisa_1
        global offset_Lisa_2
        global Desplazamiento_x
        global Bandera_Bail
        global Offset_Error
        global Lisa1D 
        global Lisa2D0
        global Coche_x
        global Coche_y
        global Coche_z
        
        Bandera_Disconected = False

        # offset_Lisa_1 = 0
        # offset_Lisa_2 = 0

        # i = 0

        print("ICI")
        Maquina_Estados = threading.Thread(target=Maquina_De_Estados)
        Maquina_Estados.start()
        


        s = serial.Serial("COM12",
                baudrate=9600,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE) ##sensOne
        while(1):
            line1 = s.readline() #Obtiene dato del serial
            # print("line1")        
            decoded_bytes = line1.decode('utf-8','ignore') #Decodifica los datos del serial
            data = (decoded_bytes.replace('\r','')).replace('\n','') #Elimina los retorno de carro
            words = data.split(",") # Agrega coma entre cada dato
            # Particion del mensaje
            if len(words) >= 4:
                try:
                    Up_Left = float(words[0]) #Rotacion en x Agacharte
                    Up_Right = float(words[1]) # Rotacion en Y
                    Down_Right = float(words[2]) #Rotacion en Z 
                    Down_Left=float(words[3])
                    if(Up_Left == 1 and Up_Right == 1 and Down_Right == 0 and Down_Left == 0):
                        print("adelante")
                        Coche_x = .55
                        Coche_y = 0
                        Coche_z = 0
                    elif(Up_Left == 0 and Up_Right == 0 and Down_Right == 0 and Down_Left == 0):
                        #print("Nada")
                        Coche_x = 0
                        Coche_y = 0
                        Coche_z = 0
                    elif(Up_Left == 0 and Up_Right == 1 and Down_Right == 0 and Down_Left == 0):
                        print("Lateral_Derecha") 
                        Coche_x = 0
                        Coche_y = -.55
                        Coche_z = 0
                    elif(Up_Left == 1 and Up_Right == 0 and Down_Right == 0 and Down_Left == 0):
                        print("Lateral_Izquierda")  
                        Coche_x = 0
                        Coche_y = .55
                        Coche_z = 0
                    elif(Down_Left == 1 and Down_Right == 1):
                        print("Atras")  
                        Coche_x = -.7
                        Coche_y = 0
                        Coche_z = 0
                    elif(Down_Left == 1 and Up_Right == 1):
                        print("Rotacion_Derecha") 
                        Coche_x = 0
                        Coche_y = 0
                        Coche_z = .5
                    elif(Up_Left == 1 and Down_Right == 1):
                        print("Rotacion_Izquieda") 
                        Coche_x = 0
                        Coche_y = 0
                        Coche_z = -.5
                    else:
                        #print("Nada")
                        Coche_x = 0
                        Coche_y = 0
                        Coche_z = 0
                except:
                    print ("Invalid line")
            if(Bandera_Disconected or Bandera_Bail):
                s.flush()
                s.close()

                break
        if(Bandera_PowerOff):
          sys.exit()    
        if(Bandera_Bail):
                Bandera_Bail = False  

        print("ALVVV")

    sys.exit() 

if __name__ == '__main__':
    asyncio.run(main())