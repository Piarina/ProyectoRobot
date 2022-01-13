
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

def gestion_leds(led1,led2,led3,forzar):
    global Estado
    if forzar==1:
        GPIO.output(led1,GPIO.HIGH)
        GPIO.output(led2,GPIO.HIGH)
        GPIO.output(led3,GPIO.HIGH)
        focos_on()
    elif Estado==1:
        GPIO.output(led1,GPIO.HIGH)
        GPIO.output(led2,GPIO.LOW)
        GPIO.output(led3,GPIO.LOW)
        focos_off()
    elif Estado==2 or Estado==-1:
        GPIO.output(led2,GPIO.HIGH)
        GPIO.output(led1,GPIO.LOW)
        GPIO.output(led3,GPIO.LOW)
        focos_off()
    elif Estado==3:
        GPIO.output(led3,GPIO.HIGH)
        GPIO.output(led1,GPIO.LOW)
        GPIO.output(led2,GPIO.LOW)
        focos_off()
    elif Estado ==4:
        GPIO.output(led1,GPIO.HIGH)
        GPIO.output(led2,GPIO.HIGH)
        GPIO.output(led3,GPIO.HIGH)
        focos_on()
    else:
        GPIO.output(led1,GPIO.LOW)
        GPIO.output(led2,GPIO.LOW)
        GPIO.output(led3,GPIO.LOW)
        focos_off()

def control_remoto(entrada):
    if entrada=="w":
        mueve_recto()
    elif entrada=="d":
        mueve_derecha()
    elif entrada=="a":
        mueve_izquierda()
    else:
        quieto()
    

def obtener_GPS():
    port="/dev/ttyAMA0"
    ser=serial.Serial(port, baudrate=9600, timeout=0.5)
    newdata=ser.readline()
    if sys.version_info[0] == 3:
        newdata = newdata.decode("utf-8","ignore")              #necesario para python3
    if newdata[0:6] == "$GPRMC":
        newmsg=pynmea2.parse(newdata)
        lat=newmsg.latitude
        lng=newmsg.longitude
        gps_str = "Latitud=" + str(lat) + ", Longitud=" + str(lng)
        #print(lat)
        #print(lng)
        return float(lat),float(lng)

#Get CPU temperature using 'vcgencmd measure_temp'                                      
def measure_temp():
    temp = os.popen('vcgencmd measure_temp').readline()
    return(temp.replace("temp=","").replace("'C\n",""))

def AoA(x):
    
    #AoA=(x+180)/+180.*(1.7-0.18)+0.18;
    #AoA=(x-9/5)*1000/9
    #AoA=(x-1.5)/(-0.0262)
    AoA=(x-1.5)/(-0.017)
    return AoA

def mueve_derecha():
    # codigo para moverrse a la derecha
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    # p1.ChangeDutyCycle(50)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
def mueve_izquierda():
    # codigo para moverrse a la izquierda
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    # p2.ChangeDutyCycle(50)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)   
def mueve_recto():
    #código para moverse recto
    #print("mueve recto")
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
def quieto():
    #código para no moverse
    print("Quieto")
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
#recibir datos
ticc1101 = TICC1101()
ticc1101.reset()
ticc1101.selfTest()
ticc1101.setDefaultValues()
ticc1101.setFilteringAddress(0x0A)
ticc1101.setPacketMode("PKT_LEN_VARIABLE")
#ticc1101.setPacketMode("PKT_LEN_FIXED")
ticc1101.configureAddressFiltering("ENABLED_NO_BROADCAST")
ticc1101._setRXState()
# rssi_0=float(244.0)
# d_0=float(2.1)
# gamma=float(6.45)    

d= None
data=[]
Estado=0
timer=0 #-----------------------------igual da problemas

# Create a VL53L0X object
tof = VL53L0X.VL53L0X()





def buscar_recto():
    global Estado
    global d
    global timer
    global data
    ticc1101._setRXState()
    data=ticc1101.recvData()
    if data!= None:
        d=data[11]
    else:
        d=None
    if d!= None and d!=0:
        print("Distancia",d)
        Estado =2
        dist=float(d)
        timer=time.time()
        return dist
    mueve_recto()         #si no se detecta nada rota en sentido horario
    #print("no recibo")
def buscar():
    global Estado
    global d
    global timer
    global Vphase,Datos_phase
    global data
    ticc1101._setRXState()
    data=ticc1101.recvData()
    
    if time.time()>timer+25:
        Estado=4
    else:
        if data!= None:
            d=data[11]
        else:
            d=None
        #print(d)
    #     data=aux[1]
    #     d=aux[0]
        for a in range(5):
            phase=adc.read_adc(0, gain=GAIN)
            Vphase=(phase*factorEscala)/1000
            Datos_phase.append(Vphase)
        
        media_phase=np.mean(Datos_phase)
        Datos_phase =[]
        ang=AoA(media_phase)
        print(media_phase)
        print("angulo",abs(ang))
        if d!= None and d!=0 and abs(ang)<35:
            print("Distancia",d)
            Estado =2
            dist=float(d)
            timer=time.time()
            return dist
        mueve_izquierda()         #si no se detecta nada rota en sentido horario
        #time.sleep(2)
        print("no recibo")
        #Estado=2
    
def seguir(dist):   #añadir la sincronización para salir cada 15 segundps
    global i,M,timer,Vmag,Vphase,Datos_mag,Datos_phase
    global Estado
    if i < M:
        ticc1101.sidle()
        
        mag=adc.read_adc(1, gain=GAIN) #Leemos los valores de Vmag y Vphase desde el conversor A , comprobar que magnitud está en A1
        Vmag=(mag*factorEscala)/1000
        Datos_mag.append(Vmag)
            
        #print(prueba)
        phase=adc.read_adc(0, gain=GAIN)
        Vphase=(phase*factorEscala)/1000
        Datos_phase.append(Vphase)
        i=i+1
        time.sleep(0.05)
    
    else:
        if timer+5<time.time():
            Estado=3
            print(Estado)
        media_mag=np.mean(Datos_mag)
        media_phase=np.mean(Datos_phase)
            
        #print(Vmag)
        #print("Media magnitud: ",media_mag)
            
        Datos_mag =[]
        Datos_phase =[]
        i=0
        #print(i)
            
        #print("Media fase: ",media_phase)
            
        angulo=AoA(media_phase)
        if dist>0.7:
            #print(media_mag)
            if media_mag>0.95: # El centro es aprox 0.92
                mueve_izquierda()
                #print(int(angulo))
                time.sleep(0.5)             # Tiempo que se está moviendo hacia la izquierda
                #p1.ChangeDutyCycle(70)
                print("Mueve izquierda")
                
            elif  media_mag<0.89 :
                mueve_derecha()
                #angulo=AoA(media_phase)
                #(int(-angulo))
                time.sleep(0.5)             # Tiempo que se está moviendo hacia la derecha
                #p1.ChangeDutyCycle(70)  
                print("Mueve derecha")
                                                              #sleep(0.1)         # wait 0.1 seconds  
            else:                       # hace falta el RSSI para saber si se está recibiendo algo
                mueve_recto()
                #angulo=AoA(media_phase)
                #print(int(angulo))
                time.sleep(0.5)
        else:
            quieto()

def inicio():
    global Estado
    vol= volcado()
    #if #aceleracion()
    t_cpu=measure_temp()
    print("temperatura CPU",t_cpu)
    if vol==True or float(t_cpu)>70:
        Estado= 4
    else:
        Estado=1
    
def safe_rover():
    quieto()
    #Estado=8
    #sense.show_message("Safe Rover", scroll_speed=0.05,text_colour=[randint(0,255),randint(0,255),randint(0,255)])
    time.sleep(2)
    print("sale de safe")
    Estado=0
def safe_user(led1,led2,led3):
    quieto()
    gestion_leds(led1,led2,led3,1)
    
    
    
def distancia_lidar(tof):
    
    tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
    timing = tof.get_timing()
    if (timing < 20000):
        timing = 20000
    
    distance = tof.get_distance()
#     if (distance > 0):
#         print ("%d mm, %d cm" % (distance, (distance/10)))
    if (distance < 500):
        tof.stop_ranging()
        return True
    time.sleep(timing/100000.00)
    tof.stop_ranging()
    return False

def esquiva(tof,led1,led3):
    tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
    timing = tof.get_timing()
    GPIO.output(led1,GPIO.HIGH)
    GPIO.output(led3,GPIO.HIGH)
    print("esquivando")
    obstaculo=True
    if (timing < 20000):
        timing = 20000
    while obstaculo:
        distance = tof.get_distance()
        print(distance)
        if (distance > 500):
            tof.stop_ranging()
            GPIO.output(led1,GPIO.LOW)
            GPIO.output(led3,GPIO.LOW)
            obstaculo= False
        elif (distance < 500):
            mueve_derecha()
            time.sleep(timing/100000)
        time.sleep(timing/10000000.00)
    print("acabo")
        
sensor = dht11.DHT11(pin=pin_dht)
def dht11(sensor):
    
    while True:
        resultado=sensor.read()
        #temperatura = sensor.temperature
        if resultado.is_valid():
            #print("humedad: %-3.1f %%" %resultado.humidity)
            #print("Temperatura: %-3.1f C" %resultado.temperature)
            return float(resultado.humidity),float(resultado.temperature)

def aceleracion():
    bus = smbus.SMBus(1)
    i2c = board.I2C()  # uses board.SCL and board.SDA

    # Initialize MMA8451 module.
    sensor = adafruit_mma8451.MMA8451(i2c)
    # Optionally change the address if it's not the default:
    #sensor = adafruit_mma8451.MMA8451(i2c, address=0x1C)

    # Optionally change the range from its default of +/-4G:
    # sensor.range = adafruit_mma8451.RANGE_2G  # +/- 2G
    # sensor.range = adafruit_mma8451.RANGE_4G  # +/- 4G (default)
    # sensor.range = adafruit_mma8451.RANGE_8G  # +/- 8G

    # Optionally change the data rate from its default of 800hz:
    # sensor.data_rate = adafruit_mma8451.DATARATE_800HZ  #  800Hz (default)
    # sensor.data_rate = adafruit_mma8451.DATARATE_400HZ  #  400Hz
    # sensor.data_rate = adafruit_mma8451.DATARATE_200HZ  #  200Hz
    # sensor.data_rate = adafruit_mma8451.DATARATE_100HZ  #  100Hz
    # sensor.data_rate = adafruit_mma8451.DATARATE_50HZ   #   50Hz
    # sensor.data_rate = adafruit_mma8451.DATARATE_12_5HZ # 12.5Hz
    # sensor.data_rate = adafruit_mma8451.DATARATE_6_25HZ # 6.25Hz
    # sensor.data_rate = adafruit_mma8451.DATARATE_1_56HZ # 1.56Hz

    # Main loop to print the acceleration and orientation every second.
    #while True:
    x, y, z = sensor.acceleration
    return round(x,2),round(y,2),round(z,2)
    #print("Acceleration: x={0:0.3f}m/s^2 y={1:0.3f}m/s^2 z={2:0.3f}m/s^2".format(x, y, z))
#         orientation = sensor.orientation
        # Orientation is one of these values:
        #  - PL_PUF: Portrait, up, front
        #  - PL_PUB: Portrait, up, back
        #  - PL_PDF: Portrait, down, front
        #  - PL_PDB: Portrait, down, back
        #  - PL_LRF: Landscape, right, front
        #  - PL_LRB: Landscape, right, back
        #  - PL_LLF: Landscape, left, front
        #  - PL_LLB: Landscape, left, back
#         print("Orientation: ", end="")
#         if orientation == adafruit_mma8451.PL_PUF:
#             print("Portrait, up, front")
#         elif orientation == adafruit_mma8451.PL_PUB:
#             print("Portrait, up, back")
#         elif orientation == adafruit_mma8451.PL_PDF:
#             print("Portrait, down, front")
#         elif orientation == adafruit_mma8451.PL_PDB:
#             print("Portrait, down, back")
#         elif orientation == adafruit_mma8451.PL_LRF:
#             print("Landscape, right, front")
#         elif orientation == adafruit_mma8451.PL_LRB:
#             print("Landscape, right, back")
#         elif orientation == adafruit_mma8451.PL_LLF:
#             print("Landscape, left, front")
#         elif orientation == adafruit_mma8451.PL_LLB:
#             print("Landscape, left, back")
#         time.sleep(1.0)

def volcado():
    a=aceleracion()
    x=a[0]
    y=a[1]
    z=a[2]
    #print(a)
    if abs(x)>8 or abs(y)>8 or z>0:
        return True
        print("volcado")
    else:
        return False
    
    
def medir_intensidad():
    volt=adc_po.read_adc(0, gain=GAIN)#Lectura del sensor
    volt=(volt*factorEscala)/1000
    I=(volt-2.5)/0.185
    Consumo=I*5 #Consumo de la bateria en el momento
    descarga_media=5/I# 5Ah entre los A dados en cada momento da la descarga media
    return  round(Consumo,2),round(I,2)
    
    
# parte de SC -------------------------------------------------------


DEFAULT_KPC_HOST = os.getenv('DEFAULT_KPC_HOST', 'mqtt.cloud.kaaiot.com')
DEFAULT_KPC_PORT = os.getenv('DEFAULT_KPC_PORT', 1883)

KPC_HOST='mqtt.cloud.kaaiot.com'
KPC_PORT=1883

EPMX_INSTANCE_NAME = os.getenv('EPMX_INSTANCE_NAME', 'epmx')
DCX_INSTANCE_NAME = os.getenv('DCX_INSTANCE_NAME', 'dcx')

#CAMARA STREAM
camara=0
PAGE="""\
<html>
<head>
<title>picamera MJPEG streaming demo</title>
</head>
<body>
<h1>PiCamera MJPEG Streaming Demo</h1>
<img src="stream.mjpg" width="640" height="480" />
</body>
</html>
"""
class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True
#CLIENT.PY

def killhandle(signum, frame):
  logger.info("SIGTERM detected, shutting down")
  disconnectFromServer(client=client, host=host, port=port)
  sys.exit(0)


signal.signal(signal.SIGINT, killhandle)
signal.signal(signal.SIGTERM, killhandle)

# Configure logging
logger = logging.getLogger('mqtt-client')
logger.setLevel(logging.DEBUG)

hdl = logging.StreamHandler()
hdl.setLevel(logging.DEBUG)
hdl.setFormatter(logging.Formatter('%(levelname)s: %(message)s'))

logger.addHandler(hdl)

client_id = ''.join(random.choice(string.ascii_uppercase + string.digits) for _ in range(6))
#host = args.host
#port = args.port
#logger.info("Using EP token {0}, server at {1}:{2}".format(token, host, port))
appversion="c6534ktah5mmh3hrti2g-1"
token="QNSs5t2PIN"
host=KPC_HOST
port=KPC_PORT
def connectToServer(client, host, port):
  logger.info("Connecting to KPC instance at {0}:{1}...".format(host, port))
  client.connect(host, port, 60)
  logger.info("Successfully connected")


def disconnectFromServer(client, host, port):
  logger.info("Disconnecting from server at {0}:{1}.".format(host, port))
  time.sleep(4)  # wait
  client.loop_stop()  # stop the loop
  client.disconnect()
  logger.info("Successfully disconnected")


# METADATA section ------------------------------------
# Compose KP1 topic for metadata
metadata_request_id = random.randint(1, 99)
topic_metadata = "kp1/{application_version}/{service_instance}/{resource_path}".format(
  application_version=appversion,
  service_instance=EPMX_INSTANCE_NAME,
  resource_path="{token}/update/keys/{metadata_request_id}".format(token=token,
                                                                   metadata_request_id=metadata_request_id)
)
logger.debug("Composed metadata topic: {}".format(topic_metadata))


def composeMetadata(version):
  global Estado
  a=obtener_GPS()
  if a==None:
      a=[0,0]
  if Estado==1:
      meta_estado="Busqueda"
  elif Estado==2:
      meta_estado= "Seguimiento"
  elif Estado==3:
      meta_estado="Envio de datos"
  elif Estado==4:
      meta_estado="Safe User"
  else:
      meta_estado="No definido"
  return json.dumps(
    {
      "model": "Proyecto",
      "fwVersion": version,
      #"latitude": 43.5453,
      #"longitude": -5.66193
      "latitude":a[0], #poner aqui el gps
      "longitude":a[1],
      "estado": meta_estado
    }
  )


# TELEMETRY section --------------------------------------
# Compose KP1 topic for data collection
data_request_id = random.randint(1, 99)
topic_data_collection = "kp1/{application_version}/{service_instance}/{resource_path}".format(
  application_version=appversion,
  service_instance=DCX_INSTANCE_NAME,
  resource_path="{token}/json/{data_request_id}".format(token=token, data_request_id=data_request_id)
)
logger.debug("Composed data collection topic: {}".format(topic_data_collection))


def composeDataSample(sensor,data):
    #Datos sensehat
    #t = sense.get_temperature()
    #p = sense.get_pressure()
    #h = sense.get_humidity()
    a=dht11(sensor)
    b=medir_intensidad()
    if data[6]==0:
        acey=data[7]
    else:
        acey=-data[7]
    if data[4]==0:
        acex=data[5]
    else:
        acex=-data[5]
    if data[8]==0:
        acez=data[9] - 90
    else:
        acez=-data[9]
    
    c=aceleracion()
        
    payload = [
    {
      "timestamp": int(round(time.time() * 1000)),
      "temperature": a[1],
      "humidity": a[0],
      "bpm": data[10],#round(random.randint(40, 140)),
      "Intensidad_descarga": int(b[1]),
      "Potencia_descarga": int(b[0]),
      "Temperatura_User":data[2]+data[3]/100,
      "Aceler_User_x":int(acex),
      "Aceler_User_y":int(acey),
      "Aceler_User_z":int(acez),
      "Temperatura_CPU": float(measure_temp()),
      "Aceler_Rover_x":c[0],
      "Aceler_Rover_y":c[1],
      "Aceler_Rover_z":c[2]
      
    }
    
  ]
    return json.dumps(payload)


def on_connect(client, userdata, flags, rc):
  if rc == 0:
    client.connected_flag = True  # set flag
    logger.info("Successfully connected to MQTT server")
  else:
    logger.info("Failed to connect to MQTT code. Returned code=", rc)


def on_message(client, userdata, message):
  logger.info("Message received: topic [{}]\nbody [{}]".format(message.topic, str(message.payload.decode("utf-8"))))
  print(message.topic)
  cam="kp1/{}/cex/{}/command/camara/status".format(appversion,token)
  sos="kp1/{}/cex/{}/command/emergencia/status".format(appversion,token)
  reboot="kp1/{}/cex/{}/command/reboot/status".format(appversion,token)
  avanza="kp1/{}/cex/{}/command/avanza/status".format(appversion,token)
  manual="kp1/{}/cex/{}/command/manual/status".format(appversion,token)
  izquierda="kp1/{}/cex/{}/command/izquierda/status".format(appversion,token)
  derecha="kp1/{}/cex/{}/command/derecha/status".format(appversion,token)
  print("on message")
  global camara, led, manua ,entrada
  print(camara)
  if message.topic==cam and camara==0:
      camara=1
  elif message.topic==cam and camara==1:
      server.shutdown()
      #server.killhandle()
      camara=0
  if message.topic==sos and led==0:
      led=1
  elif message.topic==sos and led==1:
      led=0
  if message.topic==reboot:
      cmd='sudo reboot'
      os.system(cmd)
  if message.topic==manual and manua==False:
      manua=True
      print(manua)
  elif message.topic==manual and manua==True:
      manua=False
  if message.topic==avanza:
      entrada="w"
  if message.topic==izquierda:
      entrada="a"
  if message.topic==derecha:
      entrada="d"

  


client = mqtt.Client(client_id=client_id)
client.connected_flag = False  # create flag in class
client.on_connect = on_connect  # bind call back function

client.on_message = on_message
    # Start the loop
client.loop_start()

connectToServer(client=client, host=host, port=int(port))

while not client.connected_flag:  # wait in loop
    logger.info("Waiting for connection with MQTT server")
    time.sleep(1)


    # Send data sample in loop

def tx(sensor,data):
    t=time.time()
    metadata = composeMetadata(version="v0.0.1")
    client.publish(topic=topic_metadata, payload=metadata)
    logger.info("Sent metadata: {0}\n".format(metadata))
    payload = composeDataSample(sensor,data)
    result = client.publish(topic=topic_data_collection, payload=payload)

    if result.rc != 0:
        logger.info("Server connection lost, attempting to reconnect")
        connectToServer(client=client, host=host, port=port)
    else:
        logger.debug("{0}: Sent next data: {1}".format(token, payload))
#     if (camara==1):
#         with picamera.PiCamera(resolution='640x480', framerate=24) as camera:
#             output = StreamingOutput()
#             camera.start_recording(output, format='mjpeg')
#             address = ('', 8000)
#             server = StreamingServer(address, StreamingHandler)
#             server.serve_forever()
#             camera.stop_recording()
            #time.sleep(5)
              #camara=0
      #if led==1:
          #enceder led que estara en algun puerto
      #GPIO17=1
address = ('', 8000)  
server = StreamingServer(address, StreamingHandler)
    
try:
    time.sleep(1)
    t=time.time()
    timer=time.time()
    
    while True:
      if led==1:
        safe_user(led1,led2,led3)
      else:
          gestion_leds(led1,led2,led3,0)
          #distancia_lidar(tof)
          client.on_message = on_message
          if time.time()>t+ 1.5:
    #           print(distancia_lidar(tof))
              if distancia_lidar(tof):
                  print("obstaculo")
                  esquiva(tof,led1,led3)
                  time.sleep(1)
                  print(t)
                  t=time.time()
                  print('despues',t)
              t=time.time()
          if volcado():
              safe_rover()
              print("hola")
              Estado=4
          else:
              if manua:
        #           quieto()
        #           print(entrada)
                  control_remoto(entrada)
                  
              else:
                  print(Estado)
                  if Estado == 0:
                    
                      inicio()
                      #print(obtener_GPS())
                      #print(medir_intensidad())
                      #print(obtener_GPS())
                      #aceleracion()
                      #tx(sensor,data)
                      print("inicio")
                    
                  elif Estado ==-1:
                      dist=buscar_recto()
                  elif Estado ==1:
                      dist=buscar()
                  elif Estado==2:
                      seguir(dist)
                  elif Estado ==3:
                      tx(sensor,data)
                      Estado=-1
                  elif Estado==4:
                      safe_rover();
                      timer=time.time()
                  if (camara==1):
                    quieto()
                    with picamera.PiCamera(resolution='640x480', framerate=24) as camera:
                        camera.rotation=180
                        output = StreamingOutput()
                        camera.start_recording(output, format='mjpeg')
                        server.serve_forever()
                        camera.stop_recording()    
      #print(Estado)

finally:                   # this block will run no matter how the try block exits  
    GPIO.cleanup()         # clean up after yourself  