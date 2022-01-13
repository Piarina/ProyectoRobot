
import time
import RPi.GPIO as GPIO
import Adafruit_ADS1x15
from struct import pack
# Igual falta algun import :)
#IMPORTANTE PARA EL A/D             sudo pip3 install adafruit-circuitpython-ads1x15



GPIO.setmode(GPIO.BCM)     # set up BCM GPIO numbering  es por comodidad
adc = Adafruit_ADS1x15.ADS1115(address=0x49)        #Lectura del conversor A/D el adress estanso conectado a vcc (mirar la hoja del A/D)
adc_po = Adafruit_ADS1x15.ADS1115(address=0x48)     #Lectura del conversor A/D el adress estanso conectado a GND (mirar la hoja del A/D)
GAIN = 1
factorEscala=0.125                                  # necesario para que el valor se corresponda con la tensión de entrada analog
foco=18
GPIO.setup(foco,GPIO.OUT)                           # defines el pin como salida
GPIO.output(foco,GPIO.LOW)                          # pones el pin a 0 lógico, en RPi el 1 lógico es 3.3V

f=adc_po.read_adc(3, gain=GAIN) #mirar dnd soldar  el numero corresponde con a la entrada a la que esté conectado
fotoresistencia=(f*factorEscala)/1000
umbral=600
def focos():
    GPIO.output(foco,GPIO.HIGH)                     # pin a 1, enciendes algo
while True:
    #if fotoresistencia< umbral:
        #focos()
    f=adc_po.read_adc(3, gain=GAIN)         #lees todo el rato
    fotoresistencia=(f*factorEscala)/1000
    print(fotoresistencia)
    time.sleep(0.5)