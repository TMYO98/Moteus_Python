from concurrent.futures import thread
from distutils.log import error
from xmlrpc.client import Boolean
import serial
import sys
import glob
from time import sleep,time
import struct
import threading
import asyncio
import math
import moteus
import numpy as np
import string

roll=0
pitch=0
yaw=0
Fz  = 0
Tx  = 0
Ty  = 0
Maquina_Es=0
Enc_IMU_OK = False
SensOne_OK = False
deltax = 0
Orientacion = 1
offset_torque = 0 

class KeyboardThread(threading.Thread):

    def __init__(self, input_cbk = None, name='keyboard-input-thread'):
        self.input_cbk = input_cbk
        super(KeyboardThread, self).__init__(name=name)
        self.start()

    def run(self):
        while True:
            self.input_cbk(input()) #waits to get input + Return

def my_callback(inp):
    global Maquina_Es
    global Orientacion
    global offset_torque
    if(inp=="c"):
        Maquina_Es=1 ## calibrar
        print("callibrar")
    elif(inp=="d"):
        Maquina_Es=2 ## Disengaged
        print("disengaged")
    elif(inp=="e"):
        Maquina_Es=3 ## Engaged
        print("Engaged")
    elif(inp=="v"):
        Maquina_Es=4 ## Validar
        print("Validar")
    elif(inp=="k"):
        print("Kill")
        Maquina_Es=5 ## Kill
        sys.exit()
    elif(inp=="s"):
        print("Sideways")
        Orientacion=1 ## desplazamiento lateral
    elif(inp=="r"):
        print("Rotate z")
        Orientacion=2 ## Rotacion sobre z
    elif(inp=="a"):
        print("Regresaaa")
        offset_torque =10
    elif(inp=="o"):
        print("Cool")
        offset_torque =0

#Interrupcion de teclado
kthread = KeyboardThread(my_callback)

def ENC_IMU():
    global Enc_IMU_OK
    while(1):
        if(Maquina_Es == 4):
            try:    
                ser = serial.Serial(port='COM6',baudrate=115200, timeout=1)
                Enc_IMU_OK = True
                print("ENC_IMU_VALIDADO")
                break
            except:
                Enc_IMU_OK = False
                print("ENC IMU no conection")

    Array_Torso_Carrito=[5]

    global roll
    global pitch
    global yaw  
    global deltax  
    try:
        while(1):
            line = ser.readline() #Obtiene dato del serial
            decoded_bytes = line.decode('utf-8','ignore') #Decodifica los datos del serial
            data = (decoded_bytes.replace('\r','')).replace('\n','') #Elimina los retorno de carro
            words = data.split(",") # Agrega coma entre cada dato
        # Particion del mensaje
            if len(words) >= 6:
                try:
                    roll = float(words[0]) #Rotacion en x Agacharte
                    pitch = float(words[1]) # Rotacion en Y
                    yaw = float(words[2]) #Rotacion en Z 
                    enc1=float(words[3])
                    enc2=float(words[4])
                    enc3=float(words[5])
                    ###
                    enc4=float(words[5])
                    ###
                    # enc4=float(words[6])  #este es el de verdad :V !!!
                    # Array_Torso_Carrito[3]=yaw
                    # Array_Torso_Carrito[4]=roll
                except:
                    print ("Invalid line")

                lisa1 = 0.70 # Longitud del lisa derecho (Posicion de brazo derecho de operador)
                lisa2 = 0.70 # Longitud del lisa izquierdo (Posicion de brazo izquierdo de operador)
                deltax = (-lisa1*math.sin(math.radians(enc1))*math.cos(math.radians(enc3)) - lisa2*math.sin(math.radians(enc2))*math.cos(math.radians(enc4)))/2 # Diferencial en x
                # Array_Torso_Carrito[1]=deltax
                #deltay = (-lisa1*math.cos(math.radians(enc1))*math.sin(math.radians(enc3)) - lisa2*math.cos(math.radians(enc2))*math.sin(math.radians(enc4)))/2 # Probar esta ecuacion (Diferencial en y)
                if(roll<0):
                    roll+=360       
                #print(roll) #Impresion para debuggeo
            if(Maquina_Es==5):
                ser.close()
                print("IMU Deshabilitada")
                raise SystemExit
                break
    except(KeyboardInterrupt):
        ser.close()

def serial_ports():
    if(sys.platform.startswith('win')):
        ports = ["COM%s" % (i + 1) for i in range(256)]
    elif(sys.platform.startswith("linux") or sys.platform.startswith("cygwin")):
        # Esto excluye la terminal actual "/dev/tty"
        ports = glob.glob("/dev/tty[A-Za-z]*")
    elif(sys.platform.startswith("darwin")):
        ports = glob.glob("/dev/tty.*")
    else:
        raise EnvironmentError("Sistema operativo no soportado!")

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def SensOne():
    global Fz  
    global Tx
    global Ty 
    global SensOne_OK
    while(1):
        try:  
            if(Maquina_Es == 4):  
                s = serial.Serial("COM8",
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE)
                if not s.isOpen():
                    s.open()
                # print("SenseOne@COM10 abierto", s.isOpen())                
                payload = []
                sleep(4)
                print("SensOne_VALIDADO")
                SensOne_OK = True
                
                break
        except:
            SensOne_OK = False
            print("SensOne no conection")
    print("Comenzando envio de datos SensOne")    
    try:
        while True:

            byte_senseOne = s.read()
            if(byte_senseOne == b'\xaa'):
                if(payload):
                    if(len(payload) == 36):
                        [ForceZ] = struct.unpack('f', payload[10] + payload[11] + payload[12] + payload[13])
                        [TorqueX] = struct.unpack('f', payload[14] + payload[15] + payload[16] + payload[17])
                        [TorqueY] = struct.unpack('f', payload[18] + payload[19] + payload[20] + payload[21])

                        Fz= ForceZ
                        Tx= TorqueX
                        Ty= TorqueY
                    payload = []
                else:
                    payload = []
            else:
                payload.append(byte_senseOne)
            if(Maquina_Es==5):
                print("Plataforma Deshabilitada")
                s.flushInput()
                s.close()
                raise SystemExit
                break
                
    except KeyboardInterrupt:
        s.flushInput()
        s.close()

def map( x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def Calculo_Payload(Fz,oFz,Tx,Ty, oTx,oTy, Roll, oRoll, Yaw, oYaw, Des_x,DeltaX):
    global Orientacion
    stx = Tx-oTx
    sty = Ty-oTy
    cx = 0
    cy = 0
    cz = 0
    troll = 0
    tyaw = 0
    Magnitud = math.sqrt(pow(stx,2)+pow(sty,2))   ## Calculo de magnitud del angulo, esta variable se usa para ver si sale del poligono de soporte
    Angulo = math.atan2(stx,sty)*180/3.141516  ## Calcula el ángulo al que se esta moviendo 
    oFz = abs(Fz) - abs(oFz) 
    # print("mg")
    # print(Magnitud)
    # print("of")
    # print(oFz*.01)
    #print(Magnitud)
    if(Magnitud<(.5)): # Si sobrepasa un limite, detecta como movimiento, sino, esta en el centro.
        
        # print("Center")
        cx = 0
        cy = 0
        cz = 0    
    elif(Angulo>= -45 and Angulo<= 45):
        # print("Left")
        cx = Des_x * .4
        if(Orientacion == 1):
            cy = oFz*.003*Magnitud
            cz = 0
        elif(Orientacion == 2):
            cz = oFz*.003*Magnitud
            cy = 0
    elif(Angulo>= 45 and Angulo<= 135):
        # print("UP")
        cx = map(Magnitud,.35,2,.6, 3)
        if(Orientacion == 1):
            cy = DeltaX*.4
            cz = 0
        elif(Orientacion == 2):
            cz = DeltaX*.4
            cy = 0
        
    elif(Angulo>= -135 and Angulo<= -45):
        # print("Down")  
        cx = oFz*.003*Magnitud
        if(Orientacion == 1):
            cy = DeltaX*.4
            cz = 0
        elif(Orientacion == 2):
            cz = DeltaX*.4
            cy = 0
    elif(Angulo>= 135 or Angulo<= -135):
        # print("Right") 
        cx = Des_x * .4
        if(Orientacion == 1):
            cy = oFz*.003
            cz = 0
        elif(Orientacion == 2):
            cz = oFz*.1
            cy = 0
    troll = map((Roll-oRoll),-20,20,-10,10)
    tyaw = map ((Yaw - oYaw),-30,30,-15,15)
    return [cx,cy,cz,troll,tyaw,Magnitud,Angulo]

async def main():
    
    global Fz  
    global Tx
    global Ty 
    global roll
    global pitch
    global yaw  
    global deltax  
    global Enc_IMU_OK
    global SensOne_OK

    Plataforma = threading.Thread(target=SensOne)
    Imu_Enc = threading.Thread(target=ENC_IMU)
    Imu_Enc.start()
    Plataforma.start()
    
    # By default, Controller connects to id 1, and picks an arbitrary
    # CAN-FD transport, prefering an attached fdcanusb if available.
    qr = moteus.QueryResolution()
    # qr._extra = {0x001: moteus.INT32,0x050: moteus.INT32, 0x043: moteus.F32, 
    # 0x044: moteus.F32,0x031: moteus.F32, 0x000: moteus.INT8, 0x031: moteus.F32, 0x030: moteus.F32} ## Config para leer n cantidad de parametros
    qr._extra = {0x001: moteus.INT32}
    

    Torque = 0 # Valor que sera el resultado del control PD
    Position = 0 # Posicion normalizada
    Error = 0  # Variable para el control PD
    Velocity = 0 #Usar este valor para generar friccion en el lisa
    KP = 10 
    KD = 15
    Des_Pos = 1
    Lidar_Data = 0 # Este dato vendra del lidar (Lectura por socket) y se agregara al error, para intentar prevenir al operador
    Estado = 0

    Torque2 = 0 # Valor que sera el resultado del control PD
    Position2 = 0 # Posicion normalizada
    Error2 = 0  # Variable para el control PD
    Velocity2 = 0 #Usar este valor para generar friccion en el lisa
    KP2 = 10
    KD2 = 15
    Des_Pos2 = 1
    Lidar_Data2 = 0 # Este dato vendra del lidar (Lectura por socket) y se agregara al error, para intentar prevenir al operador   
    Estado2 = 0
    Lisa_OK = False
    offset_Lisa_1 = 0
    offset_Lisa_2 = 0
    offset_roll = 0
    offset_Deltax = 0
    offset_Ty = 0
    offset_Tx = 0
    offset_Fz = 0
    offset_yaw = 0
    Iteraciones = 15
    Array_Envio = 0 
    Array_Envio = [0,0,0,0,0]
    Desplazamiento_y = 0
    global offset_torque


    ### Inicializacion LISAS
    
    while(1):
        if(Maquina_Es==4):
            try:
                c = moteus.Controller(id = 1, query_resolution = qr)
                c2 = moteus.Controller(id = 2, query_resolution = qr)
                await c2.set_stop()
                await c.set_stop()
                state = await c.set_position(position=math.nan, velocity = .5, 
                                        feedforward_torque = 0, stop_position = 0, maximum_torque = 0, query=True)
                state2 = await c2.set_position(position=math.nan, velocity = .5, 
                        feedforward_torque = 0, stop_position = 0, maximum_torque = 0, query=True)
                Lisa_OK = True
                print("Lisa_Validado")
                break
            except:
                Lisa_OK = False

    
    ####
    while(1):
        if( Lisa_OK==True and Enc_IMU_OK == True and SensOne_OK):
            if(Maquina_Es==1):
                print("Calibrando")
                try:
                    state = await c.set_position(position=math.nan, velocity = .3, 
                    feedforward_torque = 0, stop_position = 0, maximum_torque = 0, query=True)
                    state2 = await c2.set_position(position=math.nan, velocity = .3, 
                    feedforward_torque = 0, stop_position = 0, maximum_torque = 0, query=True)
                    Des_Pos = state.values[moteus.Register.POSITION] ## Trabajar con la cantidad de vueltas
                    Des_Pos2 = state2.values[moteus.Register.POSITION] ## Trabajar con la cantidad de vueltas
                    for i in range(Iteraciones):
                        await c2.set_stop()
                        await c.set_stop()
                        state = await c.set_position(position=math.nan, velocity = .3, 
                                        feedforward_torque = 0, stop_position = 0, maximum_torque = 0, query=True)
                        state2 = await c2.set_position(position=math.nan, velocity = .3, 
                        feedforward_torque = 0, stop_position = 0, maximum_torque = 0, query=True)
                        #print("RawP:", (state.values[moteus.Register.ENCODER_0_POSITION]))
                        offset_Lisa_1 += state.values[moteus.Register.POSITION] ## Trabajar con la cantidad de vueltas
                        offset_Lisa_2 += state2.values[moteus.Register.POSITION] ## Trabajar con la cantidad de vueltas
                        offset_roll += roll
                        offset_Deltax += deltax
                        offset_Ty += Ty
                        offset_Tx += Tx
                        offset_Fz += Fz
                        offset_yaw += yaw
                        
                        sleep(.05)
                    print("Sistemas Calibrados")    
                    break
                except:
                    print("Error al calibrar")
    
    offset_Lisa_1 = offset_Lisa_1 /Iteraciones
    offset_Lisa_2 = offset_Lisa_2 /Iteraciones
    offset_roll = offset_roll/Iteraciones
    offset_Deltax = offset_Deltax / Iteraciones
    offset_Ty = offset_Ty/Iteraciones
    offset_Tx = offset_Tx/Iteraciones
    offset_Fz = offset_Fz/Iteraciones
    offset_yaw = offset_yaw/Iteraciones  

    print(offset_Lisa_1)
    print(offset_Lisa_2)
    print(offset_roll)
    print(offset_Deltax)
    print(offset_Ty)
    print(offset_Tx)
    print(offset_Fz)
    print(offset_yaw)


    while True:
        if(Maquina_Es==3):
            KD = 18
            KD2 = 18
            KP = 10
            KP2 = 10
           
            print(Array_Envio)
        else:
            KD = 0
            KD2 = 0
            KP = 0
            KP2 = 0 
        
        state = await c.set_position (position=math.nan, velocity = .3, 
                                            feedforward_torque = .1, stop_position = Des_Pos, maximum_torque = Torque,
                                            kp_scale =1, kd_scale =15 ,query=True)                                     
        Position = state.values[moteus.Register.POSITION] ## Trabajar con la cantidad de vueltas
        Velocity = abs(state.values[moteus.Register.VELOCITY]) ## Velocidad del motor
        Estado = state.values[moteus.Register.FAULT]
        Error = abs(Des_Pos-Position)+Lidar_Data 
        Torque = (Error*KP)+(Velocity*KD)+offset_torque
        if(Estado!=0):
            await c.set_stop() 
             
            ## Segundo lisa
        state2 = await c2.set_position (position=math.nan, velocity = .3, 
                                        feedforward_torque = .1, stop_position = Des_Pos2, maximum_torque = Torque2,
                                        kp_scale =1, kd_scale =15 ,query=True)                                     
        Position2 = state2.values[moteus.Register.POSITION] ## Trabajar con la cantidad de vueltas
        Velocity2 = abs(state2.values[moteus.Register.VELOCITY]) ## Velocidad del motor
        Estado2 = state2.values[moteus.Register.FAULT]
        Error2 = abs(Des_Pos2-Position2)+Lidar_Data2 
        Torque2 = (Error2*KP2)+(Velocity2*KD2)+offset_torque
        # print(Torque)
        # print(Torque2)
        if(Estado2!=0):
            await c2.set_stop() 

        Desplazamiento_x = ((Des_Pos-Position)+(Des_Pos2-Position2))/2
        Array_Envio = Calculo_Payload(Fz, offset_Fz, Tx,Ty,offset_Tx,offset_Ty,roll,offset_roll,yaw, offset_yaw, Desplazamiento_x,deltax)
        #print(Desplazamiento_x)
                ##
        
                    
        

                # print ("Error2")
                # print(Error2)
                #print ("tx")
                #print(Torque)
                # print("roll")
                # print(roll)
        Array_Envio = Calculo_Payload(Fz, offset_Fz, Tx,Ty,offset_Tx,offset_Ty,roll,offset_roll,yaw, offset_yaw, Desplazamiento_x,deltax)
        #print(Array_Envio)
        await asyncio.sleep(0.02)
        if(Maquina_Es==2):
            await c2.set_stop() 
            await c.set_stop() 
        elif(Maquina_Es==5):
            await c2.set_stop() 
            await c.set_stop() 
            sleep(.5)
            print("Lisa_Deshabilitada")
            raise SystemExit
            exit()
            break
            

# 0,1 b roll, z
#2,3,4  x,y,z carrito


    


    
    



if __name__ == '__main__':
    asyncio.run(main())
