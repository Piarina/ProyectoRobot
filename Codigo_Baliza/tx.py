#!/usr/bin/python3

from pycc1101.pycc1101 import TICC1101
from struct import pack
import time
import os
import smbus
import struct
import math
from procesa_datos import calcula_angulos
import Adafruit_ADS1x15
import serial
from bitarray import bitarray

#--------------------------------Temperatura------------------------------------
def Temperatura_baliza():
    bus = smbus.SMBus(1)
    # BMP280 address, 0x76(118)
    # Read data back from 0x88(136), 24 bytes
    b1 = bus.read_i2c_block_data(0x76, 0x88, 24)
    # Convert the data
    # Temp coefficents
    dig_T1 = b1[1] * 256 + b1[0]
    dig_T2 = b1[3] * 256 + b1[2]
    if dig_T2 > 32767 :
        dig_T2 -= 65536
    dig_T3 = b1[5] * 256 + b1[4]
    if dig_T3 > 32767 :
        dig_T3 -= 65536
# Pressure coefficents
    dig_P1 = b1[7] * 256 + b1[6]
    dig_P2 = b1[9] * 256 + b1[8]
    if dig_P2 > 32767 :
        dig_P2 -= 65536
    dig_P3 = b1[11] * 256 + b1[10]
    if dig_P3 > 32767 :
        dig_P3 -= 65536
    dig_P4 = b1[13] * 256 + b1[12]
    if dig_P4 > 32767 :
        dig_P4 -= 65536
    dig_P5 = b1[15] * 256 + b1[14]
    if dig_P5 > 32767 :
        dig_P5 -= 65536
    dig_P6 = b1[17] * 256 + b1[16]
    if dig_P6 > 32767 :
        dig_P6 -= 65536
    dig_P7 = b1[19] * 256 + b1[18]
    if dig_P7 > 32767 :
        dig_P7 -= 65536
    dig_P8 = b1[21] * 256 + b1[20]
    if dig_P8 > 32767 :
        dig_P8 -= 65536
    dig_P9 = b1[23] * 256 + b1[22]
    if dig_P9 > 32767 :
        dig_P9 -= 65536
    # BMP280 address, 0x76(118)
    # Select Control measurement register, 0xF4(244)
    # 0x27(39) Pressure and Temperature Oversampling rate = 1
    # Normal mode
    bus.write_byte_data(0x76, 0xF4, 0x27)
    # BMP280 address, 0x76(118)
    # Select Configuration register, 0xF5(245)
    # 0xA0(00) Stand_by time = 1000 ms
    bus.write_byte_data(0x76, 0xF5, 0xA0)
    time.sleep(0.5)
    # BMP280 address, 0x76(118)
    # Read data back from 0xF7(247), 8 bytes
    # Pressure MSB, Pressure LSB, Pressure xLSB, Temperature MSB, Temperature LSB
    # Temperature xLSB, Humidity MSB, Humidity LSB
    data = bus.read_i2c_block_data(0x76, 0xF7, 8)
    # Convert pressure and temperature data to 19-bits
    adc_p = ((data[0] * 65536) + (data[1] * 256) + (data[2] & 0xF0)) / 16
    adc_t = ((data[3] * 65536) + (data[4] * 256) + (data[5] & 0xF0)) / 16
    # Temperature offset calculations
    var1 = ((adc_t) / 16384.0 - (dig_T1) / 1024.0) * (dig_T2)
    var2 = (((adc_t) / 131072.0 - (dig_T1) / 8192.0) * ((adc_t)/131072.0 - (dig_T1)/8192.0)) * (dig_T3)
    t_fine = (var1 + var2)
    cTemp = (var1 + var2) / 5120.0
    fTemp = cTemp * 1.8 + 32
    # Pressure offset calculations
    var1 = (t_fine / 2.0) - 64000.0
    var2 = var1 * var1 * (dig_P6) / 32768.0
    var2 = var2 + var1 * (dig_P5) * 2.0
    var2 = (var2 / 4.0) + ((dig_P4) * 65536.0)
    var1 = ((dig_P3) * var1 * var1 / 524288.0 + ( dig_P2) * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * (dig_P1)
    p = 1048576.0 - adc_p
    p = (p - (var2 / 4096.0)) * 6250.0 / var1
    var1 = (dig_P9) * p * p / 2147483648.0
    var2 = p * (dig_P8) / 32768.0
    pressure = (p + (var1 + var2 + (dig_P7)) / 16.0) / 100
    # Output data to screen
    #print("Temperature in Celsius: ", float(cTemp))
    #print "Temperature in Fahrenheit : %.2f F" %fTemp
    #print "Pressure : %.2f hPa " %pressure
    return float(cTemp)
#--------------------------------Antena------------------------------------

ticc1101 = TICC1101()
ticc1101.reset()
ticc1101.selfTest()
ticc1101.setDefaultValues()
ticc1101.setFilteringAddress(0x0A)
ticc1101.setPacketMode("PKT_LEN_VARIABLE")
#ticc1101.setPacketMode("PKT_LEN_FIXED")
ticc1101.configureAddressFiltering("ENABLED_NO_BROADCAST")



#--------------------------------Acelerómetro------------------------------------

# Direccion del sensor
ACCEL_ADD = 0x6B

# Registro Who am I?
WHO_AM_I_REG= 0x0F

# Registros de control
CTRL1_XL = 0x10
CTRL10_C = 0x19
CTRL2_G =  0x11

# Registros de datos del acelerometro
OUT_X_L = 0x28
OUT_X_H = 0x29
OUT_Y_L = 0x2A
OUT_Y_H = 0x2B
OUT_Z_L = 0x2C
OUT_Z_H = 0x2D

# Registros de datos del giroscopo
OUT_X_L_G = 0x22
OUT_X_H_G = 0x23
OUT_Y_L_G = 0x24
OUT_Y_H_G = 0x25
OUT_Z_L_G = 0x26
OUT_Z_H_G = 0x27

# Para poder comunicarse con bus I2C
bus = smbus.SMBus(1)

# Configura acelerometro, escribiendo en el registro CTRL1_XL
# Escribe 10000000b = 0x80: modo HP, 1.66 kHz, +/-2g, filtro 400 Hz
bus.write_byte_data(ACCEL_ADD, CTRL1_XL, 0x80)
time.sleep(0.1)

#--------------------------------Sensor de ritmo cardiaco------------------------------------

rate = [0]*10
amp = 100
GAIN = 2/3  
curState = 0
stateChanged = 0


#--------------------------------Envio de datos------------------------------------

count = 0

def float_to_hex(f):
	return hex(struct.unpack('<I', struct.pack('<f', f))[0])
def read_pulse():
    firstBeat = True
    secondBeat = False
    sampleCounter = 0
    lastBeatTime = 0
    lastTime = int(time.time()*1000)
    th = 525
    P = 512
    T = 512
    IBI = 600
    Pulse = False
    #adc = Adafruit_ADS1x15.ADS1115()
    adc = Adafruit_ADS1x15.ADS1115(address=0x49)

    while True:
        
        Signal = adc.read_adc(0, gain=GAIN)   
        curTime = int(time.time()*1000)
        #send_to_prcessing("S",Signal)
        sampleCounter += curTime - lastTime
        lastTime = curTime
        N = sampleCounter - lastBeatTime

        if Signal > th and  Signal > P:          
            P = Signal
     
        if Signal < th and N > (IBI/5.0)*3.0 :  
            if Signal < T :                      
              T = Signal                                                 
        
        if N > 250 :                              
            if  (Signal > th) and  (Pulse == False) and  (N > (IBI/5.0)*3.0)  :       
              Pulse = 1;                       
              IBI = sampleCounter - lastBeatTime
              lastBeatTime = sampleCounter       

              if secondBeat :                     
                secondBeat = 0;               
                for i in range(0,10):             
                  rate[i] = IBI                      

              if firstBeat :                        
                firstBeat = 0                  
                secondBeat = 1                  
                continue                              

              runningTotal = 0;               
              for i in range(0,9):            
                rate[i] = rate[i+1]       
                runningTotal += rate[i]      

              rate[9] = IBI;                  
              runningTotal += rate[9]        
              runningTotal /= 10;             
              BPM = 60000/runningTotal       
              print("BPM:" + str(BPM))
              return BPM
              #send_to_prcessing("B", BPM)
              #send_to_prcessing("Q", IBI)

        if Signal < th and Pulse == 1 :                    
            amp = P - T                   
            th = amp/2 + T
            T = th
            P = th
            Pulse = 0 
            
        if N > 2500 :
            th = 512
            T = th                  
            P = th                                              
            lastBeatTime = sampleCounter
            firstBeat = 0                     
            secondBeat = 0                   
            print("no beats found")
            return 60

while True:
    BPM=read_pulse()
    # Leemos los datos de los 6 registros
    xH_raw = bus.read_byte_data(ACCEL_ADD, OUT_X_H)
    xL_raw = bus.read_byte_data(ACCEL_ADD, OUT_X_L)
    yH_raw = bus.read_byte_data(ACCEL_ADD, OUT_Y_H)
    yL_raw = bus.read_byte_data(ACCEL_ADD, OUT_Y_L)
    zH_raw = bus.read_byte_data(ACCEL_ADD, OUT_Z_H)
    zL_raw = bus.read_byte_data(ACCEL_ADD, OUT_Z_L)

    # Llamo a funcion que procesa las lecturas y devuelve los angulos
    ang_x, ang_y = calcula_angulos(xH_raw, xL_raw, yH_raw, yL_raw, zH_raw, zL_raw)
    
    cTemp=Temperatura_baliza()
    # Lista para enviar datos
    cTemp1 = round(cTemp,2)
    cTemp_int = int(cTemp1)
    cTemp_dec = round((abs(cTemp1)- abs(int(cTemp1))),2)*100 # Separo parte entera y decimal de la temperatura para enviarla
    
    signo1 = 0
    signo2 = 0
    
    if ang_x < 0:
	    ang_x = abs(ang_x)
	    signo1 = 1
    
    if ang_y < 0:
	    ang_y = abs(ang_y)
	    signo2 = 1
    

    
    tlist= [count,cTemp_int,int(cTemp_dec),signo1,abs(int(ang_x)),signo2,abs(int(ang_y)),int(BPM)]

    if(count == 255):
        count = 0

    print(tlist)
    list_byte=bytearray(tlist)
    #list_byte=bitarray(64)
    flag=ticc1101.sendData(list_byte)
    #while not flag:
    aa=0
    #while (aa<5):
    ticc1101.sendData(bytearray(1024))
        #time.sleep(0.1)
        #aa +=1
        
    count += 1
    time.sleep(1)
