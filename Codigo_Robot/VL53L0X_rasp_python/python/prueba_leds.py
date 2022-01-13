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

# sobran un monton de imports. Es un código de control de leds
GPIO.setmode(GPIO.BCM) # Define el modo de la numeración de los pines de la rpi, según está el numero no es el numero del pin, sino el GPIO <número>
led1=20
led2=16
led3=19
GPIO.setup(led1,GPIO.OUT) #Definición de salidas
GPIO.setup(led2,GPIO.OUT)
GPIO.setup(led3,GPIO.OUT)

GPIO.output(led1,GPIO.LOW) #Empiezan apagados
GPIO.output(led2,GPIO.LOW)
GPIO.output(led3,GPIO.LOW)


def gestion_leds(led1,led2,led3): # Dependiendo del valor de Estado se encienden unos leds u otros
    global Estado
    if Estado==1:
        GPIO.output(led1,GPIO.HIGH)
        GPIO.output(led2,GPIO.LOW)
        GPIO.output(led3,GPIO.LOW)
    elif Estado==2 or Estado==-1:
        GPIO.output(led2,GPIO.HIGH)
        GPIO.output(led1,GPIO.LOW)
        GPIO.output(led3,GPIO.LOW)
    elif Estado==3:
        GPIO.output(led3,GPIO.HIGH)
        GPIO.output(led1,GPIO.LOW)
        GPIO.output(led2,GPIO.LOW)
    elif Estado ==8:
        GPIO.output(led1,GPIO.HIGH)
        GPIO.output(led2,GPIO.HIGH)
        GPIO.output(led3,GPIO.HIGH)
    else:
        GPIO.output(led1,GPIO.LOW)
        GPIO.output(led2,GPIO.LOW)
        GPIO.output(led3,GPIO.LOW)
Estado=0       
while True:
    
    gestion_leds(led1,led2,led3)
    time.sleep(2)
    Estado=Estado+1