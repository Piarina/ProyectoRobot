
# importante el :        sudo pip install dht11 
# Segurmente si falla sea que necesitas sudo pip3 install dht11
import RPi.GPIO as GPIO   
import dht11
import string
# código sencillo para obtención de temperatura con dht11
GPIO.setmode(GPIO.BCM)
GPIO.cleanup()
sensor = dht11.DHT11(pin=23)
while True:
    resultado=sensor.read()
    #print("Temperatura: %-3.1f C" %resultado.temperature)
    #temperatura = sensor.temperature
    if resultado.is_valid():
        print("humedad: %-3.1f %%" %resultado.humidity)
        print("Temperatura: %-3.1f C" %resultado.temperature)
