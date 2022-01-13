
import time
import os 
from pycc1101.pycc1101 import TICC1101
from struct import pack
import binascii
import RPi.GPIO as GPIO
import numpy as np
import Adafruit_ADS1x15
import dht11
from struct import pack
import serial
import pynmea2
import sys
import string
import VL53L0X
import smbus
from MMA8452Q import MMA8452Q

# Parte del SC
import argparse
import json
import logging
import random
import signal
import string
import paho.mqtt.client as mqtt
import io
import picamera
import logging
import socketserver
from threading import Condition
from http import server

import board
import adafruit_mma8451

adc = Adafruit_ADS1x15.ADS1115(address=0x49)
adc_po = Adafruit_ADS1x15.ADS1115(address=0x48)
GPIO.setmode(GPIO.BCM)     # set up BCM GPIO numbering  
GAIN = 1
factorEscala=0.125
Datos_phase =[]
Datos_mag =[]
media_phase=0
media_mag=0
i=0
M=8          # Numero de datos promediados
# Motor
in1 = 12
in2 = 13
in3 = 5
in4 = 6
en = 26
pin_dht= 23

GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(en,GPIO.OUT)

#GPIO.setup(pin,GPIO.OUT)

GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)
#Prueba 
# GPIO.setup(13,GPIO.OUT)
# prueba=GPIO.PWM(13,10)
# prueba.start(0)

p1=GPIO.PWM(en,1000)
p1.start(98)

manua=False
entrada=None
led=0
led1=20
led2=19
led3=16
foco=18
GPIO.setup(led1,GPIO.OUT)
GPIO.setup(led2,GPIO.OUT)
GPIO.setup(led3,GPIO.OUT)
GPIO.setup(foco,GPIO.OUT)

GPIO.output(led1,GPIO.LOW)
GPIO.output(led2,GPIO.LOW)
GPIO.output(led3,GPIO.LOW)
GPIO.output(foco,GPIO.LOW)

def focos_on():
    
    GPIO.output(18,GPIO.HIGH)
def focos_off():
    
    GPIO.output(18,GPIO.LOW)

while True:
    focos_on()