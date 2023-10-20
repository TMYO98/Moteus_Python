import time
import socket
import struct
from statemachine import StateMachine, State
from time import sleep

@@ -28,17 +30,30 @@ class MasterController(StateMachine):

controller = MasterController()

hostIP         = '0.0.0.0'          #OUR OWN ADDRESS
peerIP         = '16.221.4.111'     #OMNIWHEELS SYSTEM'S  IP ADDRESS
masterIP       = '16.221.2.51'      #MASTER'S             IP ADDRESS

localPort      = 0xC04F             #OWN (LISA) PORT
peerPort       = 0xC06F             #OMNIWHEEL'S PORT
masterPort     = 0xC033             #MASTER'S PORT

localDeviceId  = 0x4F4F             #LISA'S DEVICE ID
peerDeviceId   = 0x416F             #OMNIWHEEL'S DEVICE ID
masterDeviceId = 0x4D33             #MASTER'S DEVICE ID


udpsocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
@@ -97,6 +112,11 @@ class MasterController(StateMachine):

start_ref_time = time.perf_counter()
lastTimeStamp = 0

################ OTHER FUNCTIONS ################

@@ -106,7 +126,14 @@ def timestamp_processing():
    return elapsedTime

def message_counting():
    return

def calculateCRC(datagram, datagramsize):
    crc = 0xFFFF
@@ -129,15 +156,25 @@ def calculateCRC(datagram, datagramsize):

def receiveDatagram():
    global rxDatagram
    while True:
        try:
            data = udpsocket.recvfrom(rx_DatagramSize)
            rxDatagram = data[0]
            if data:
                tag = (data[0][tag_l],data[0][tag_l+1],data[0][tag_l+2],data[0][tag_l+3],data[0][tag_l+4])
                if(validateTag(tag) == False):
                    return (False, data[0], 0)
                elif(rxDatagram[cmd_l] == 0x40 and validateTimestamp(rxDatagram[cmd_l]) == False):
                    return (False, data[0], 0)
                elif(validateCommand(data[0][cmd_l]) == False):
                    return (False, data[0], 0)
@@ -148,7 +185,7 @@ def receiveDatagram():
                    extern_data = [extern_deviceID, extern_device, extern_IPaddress, data[0][cmd_l]]
                    return (True, data[0], extern_data)
        except socket.timeout:
            sleep(0.001)
            #print("................................................................")
            if(controller.is_engaged):
                txDatagram[rcd_l] = control()
@@ -170,7 +207,7 @@ def sendDatagram(MessageType, dataforextern):
        txDatagram[opc_l]   = 0x00    #OPC

        txDatagram[dID_l]   = 0x41    #Device ID - 'A' for Avatar - peer to connect
        txDatagram[dID_l+1] = 0x6F    #Device ID - 'o' for Omniwheels - peer to connect

        txDatagram[cmd_l]   = 0x40

@@ -199,7 +236,8 @@ def sendDatagram(MessageType, dataforextern):
        addressToSend = dataforextern[2]
        portToSend = 0xC000 + int(dataforextern[1])

    timestamp = timestamp_processing()
    txDatagram[tst_l+0] = timestamp[0]
    txDatagram[tst_l+1] = timestamp[1]
    txDatagram[tst_l+2] = timestamp[2]
@@ -210,7 +248,7 @@ def sendDatagram(MessageType, dataforextern):

    txDatagram[tx_crc_l+0] = byteCRC[0]
    txDatagram[tx_crc_l+1] = byteCRC[1]
    #print("Sending to: ", (addressToSend, portToSend))
    udpsocket.sendto(txDatagram,(addressToSend, portToSend))

################ UDP MESSAGE VALIDATION ################ 
@@ -224,19 +262,20 @@ def validateTag(rxTag):
        print("Invalid Tag: ", chr(rxTag[0]), chr(rxTag[1]), chr(rxTag[2]), chr(rxTag[3]), chr(rxTag[4]))
        return False

def validateCommand(rxCmd):
    if((rxCmd == 0x06) or (rxCmd == 0x21) or (rxCmd == 0x22) or (rxCmd == 0x23) or (rxCmd == 0x24) or (rxCmd == 0x25) or (rxCmd == 0x26) or (rxCmd == 0x27) or (rxCmd == 0x28) or (rxCmd == 0x40) or (rxCmd == 0x5E)):
        return True
    else:
        print("Invalid Command: ", hex(rxCmd))
        return False

def validateTimestamp(rxTstmp):
    global lastTimeStamp
    if(rxTstmp - lastTimeStamp >= 0):
        lastTimeStamp = rxTstmp
        return True
    else:
        return False

def validateCRC(rxCRC):
@@ -258,11 +297,11 @@ def control():
    global inputData1
    global inputData2

    outputData1 = inputData2 + inputData1
    outputData2 = inputData1 - inputData2
    outputData3 = (inputData1 + inputData2) / 2.7183
    outputData4 = inputData2 + inputData2
    outputData5 = inputData1 - inputData1

    global data_is_ready
    data_is_ready = True
@@ -358,6 +397,9 @@ def updateOutgoingData():
    txDatagram[tx_pld_q5_l+2] = txQ5[1]
    txDatagram[tx_pld_q5_l+1] = txQ5[2]
    txDatagram[tx_pld_q5_l+0] = txQ5[3]
    return

################ STATE MACHINE TRANSITIONS ################
@@ -379,7 +421,7 @@ def updateOutgoingData():
                controller.to_connect()
                txDatagram[rcd_l] = connect_f()
            elif(controller.is_connected):
                print("CONNECTED")
                txDatagram[rcd_l] = 0x01
            else:
                print("Not a valid command for this state")
@@ -391,7 +433,7 @@ def updateOutgoingData():
                controller.to_validate()
                txDatagram[rcd_l] = validate_f()
            elif(controller.is_validated):
                print("VALIDATED")
                txDatagram[rcd_l] = 0x01
            else:
                print("Not a valid command for this state")
@@ -403,7 +445,7 @@ def updateOutgoingData():
                controller.to_calibrate()
                txDatagram[rcd_l] = calibrate_f()
            elif(controller.is_calibrated):
                print("CALIBRATED")
                txDatagram[rcd_l] = 0x01
            else:
                print("Not a valid command for this state")
@@ -422,7 +464,7 @@ def updateOutgoingData():
                controller.gobackto_home()
                txDatagram[rcd_l] = gobackto_home_f()
            elif(controller.is_home):
                print("AT HOME")
                txDatagram[rcd_l] = 0x01
            else:
                print("Not a valid command for this state")
@@ -431,10 +473,13 @@ def updateOutgoingData():
        elif(data[cmd_l] == 0x25):
            if(controller.is_home):
                print("Changing to ENGAGED")
                controller.to_engage()
                txDatagram[rcd_l] = engage_f()
            elif(controller.is_engaged):
                print("ENGAGED")
                txDatagram[rcd_l] = 0x01
            else:
                print("Not a valid command for this state")
@@ -446,7 +491,7 @@ def updateOutgoingData():
                controller.to_disengage()
                txDatagram[rcd_l] = disengage_f()
            elif(controller.is_disengaged):
                print("DISENGAGED")
                txDatagram[rcd_l] = 0x01
            else:
                print("Not a valid command for this state")
@@ -458,7 +503,7 @@ def updateOutgoingData():
                controller.to_disconnect()
                txDatagram[rcd_l] = disconnect_f()
            elif(controller.is_disconnected):
                print("DISCONNECTED")
                txDatagram[rcd_l] = 0x01
            else:
                print("Not a valid command for this state")
@@ -470,7 +515,7 @@ def updateOutgoingData():
                controller.to_power_off()
                txDatagram[rcd_l] = poweroff_f()
            elif(controller.is_powered_off):
                print("POWERED OFF")
                txDatagram[rcd_l] = 0x01
            else:
                print("Not a valid command for this state")