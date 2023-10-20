import asyncio
import math
import moteus
import numpy as np





async def main():
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
    KP = 10
    KD = 15
    Des_Pos = 1
    Lidar_Data = 0 # Este dato vendra del lidar (Lectura por socket) y se agregara al error, para intentar prevenir al operador
    Error_Offset = 5
    Estado = 0

    ### Segundo LISA
    c2 = moteus.Controller(id = 2, query_resolution = qr)

    Torque2 = 0 # Valor que sera el resultado del control PD
    Position2 = 0 # Posicion normalizada
    Error2 = 0  # Variable para el control PD
    Velocity2 = 0 #Usar este valor para generar friccion en el lisa
    KP2 = 10
    KD2 = 15
    Des_Pos2 = 1
    Lidar_Data2 = 0 # Este dato vendra del lidar (Lectura por socket) y se agregara al error, para intentar prevenir al operador
    Error_Offset2 = 1
    Estado2 = 0
    ####
    
    
    await c2.set_stop()

    
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

    ## Segundo Lisa
    state2 = await c2.set_position(position=math.nan, velocity = .3, 
                                    feedforward_torque = 0, stop_position = 0, maximum_torque = 0, query=True)

    
    #print("RawP:", (state.values[moteus.Register.ENCODER_0_POSITION]))
    Des_Pos2 = state2.values[moteus.Register.POSITION] ## Trabajar con la cantidad de vueltas
    ####



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

        ## Segundo lisa
        state2 = await c2.set_position (position=math.nan, velocity = .3, 
                                    feedforward_torque = .1, stop_position = Des_Pos2, maximum_torque = Torque2,
                                    kp_scale =1, kd_scale =15 ,query=True)                                     
        Position2 = abs(state2.values[moteus.Register.POSITION]) ## Trabajar con la cantidad de vueltas
        Velocity2 = abs(state2.values[moteus.Register.VELOCITY]) ## Velocidad del motor
        Estado2 = state2.values[moteus.Register.FAULT]
        Error2 = abs(Des_Pos2-Position2)+Lidar_Data2 + Error_Offset2
        Torque2 = (Error2*KP2)+(Velocity2*KD2*Error2)
        if(Estado2!=0):
           await c2.set_stop() 

        #print (state)
        print(Error)
        ##
  
            
        state = await c.set_position (position=math.nan, velocity = .3, 
                                    feedforward_torque = .1, stop_position = Des_Pos, maximum_torque = Torque,
                                    kp_scale =1, kd_scale =15 ,query=True)                                     
        Position = abs(state.values[moteus.Register.POSITION]) ## Trabajar con la cantidad de vueltas
        Velocity = abs(state.values[moteus.Register.VELOCITY]) ## Velocidad del motor
        Estado = state.values[moteus.Register.FAULT]
        Error = abs(Des_Pos-Position)+Lidar_Data + Error_Offset
        Torque = (Error*KP)+(Velocity*KD*Error)
        if(Estado!=0):
           await c.set_stop() 

        #print (state)
        print(Error2)
        #print (Estado)

        

       
        #print("POS:", int(state.values[moteus.Register.ENCODER_0_POSITION]))

        # And a blank line so we can separate one iteration from the
        # next.

        # Wait 20ms between iterations.  By default, when commanded
        # over CAN, there is a watchdog which requires commands to be
        # sent at least every 100ms or the controller will enter a
        # latched fault state.
        await asyncio.sleep(0.02)

if __name__ == '__main__':
    asyncio.run(main())
