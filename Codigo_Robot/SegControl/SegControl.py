import argparse
import json
import logging
import os
import random
import signal
import string
import sys
import time
import paho.mqtt.client as mqtt
from sense_hat import SenseHat
import io
import picamera
import logging
import socketserver
from threading import Condition
from http import server
#Parte del sense hat, al final no se usa en el codigo completo por incompatibilidad con el CC1101
sense = SenseHat()
sense.set_rotation(270)
# configuracion de algunos elementos necesarios para conectarse con la plataforma elegida Kaaiot
DEFAULT_KPC_HOST = os.getenv('DEFAULT_KPC_HOST', 'mqtt.cloud.kaaiot.com')
DEFAULT_KPC_PORT = os.getenv('DEFAULT_KPC_PORT', 1883)

KPC_HOST='mqtt.cloud.kaaiot.com'
KPC_PORT=1883

EPMX_INSTANCE_NAME = os.getenv('EPMX_INSTANCE_NAME', 'epmx')
DCX_INSTANCE_NAME = os.getenv('DCX_INSTANCE_NAME', 'dcx')

#CAMARA STREAM
# página web en la que se mostará en streaming la imagen de la camara conectada a la raspberry
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

# Parse command line arguments and get device name
#parser = argparse.ArgumentParser(description="MQTT client for demo application")
#parser.add_argument("-d", "--deviceName", action="store", dest="deviceName", default="BME/BMP 280",
                    #required=False, help="Name of connected device")
#parser.add_argument("-a", "--appversion", action="store", dest="appversion", required=True,
                    #help="Application version")
#parser.add_argument("-t", "--token", action="store", dest="token", required=True,
                    #help="Device token")
#parser.add_argument("-s", "--host", action="store", dest="host", default=DEFAULT_KPC_HOST,
                    #help="Server host to connect to")
#parser.add_argument("-p", "--port", action="store", dest="port", default=DEFAULT_KPC_PORT,
                    #help="Server port to connect to")
#args = parser.parse_args()
#appversion = args.appversion
#token = args.token
client_id = ''.join(random.choice(string.ascii_uppercase + string.digits) for _ in range(6))
#host = args.host
#port = args.port
#logger.info("Using EP token {0}, server at {1}:{2}".format(token, host, port))
appversion="c6534ktah5mmh3hrti2g-1"
token="QNSs5t2PIN"
host=KPC_HOST
port=KPC_PORT
# Conexión al servidor 
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
# conformado de los mensajes, los cuales son propios de la plataforma utilizada
metadata_request_id = random.randint(1, 99)
topic_metadata = "kp1/{application_version}/{service_instance}/{resource_path}".format(
  application_version=appversion,
  service_instance=EPMX_INSTANCE_NAME,
  resource_path="{token}/update/keys/{metadata_request_id}".format(token=token,
                                                                   metadata_request_id=metadata_request_id)
)
logger.debug("Composed metadata topic: {}".format(topic_metadata))


def composeMetadata(version):
  return json.dumps(
    {
      "model": "Proyecto",
      "fwVersion": version,
      #"latitude": 43.5453,
      #"longitude": -5.66193
      "latitude":random.randint(40, 50), #se usa un numero aleatorio pq en interiores no funciona GPS
      "longitude":random.randint(-7, -5)
    }
  )


# TELEMETRY section --------------------------------------
# Compose KP1 topic for data collection 
# Má datos para el envío
data_request_id = random.randint(1, 99)
topic_data_collection = "kp1/{application_version}/{service_instance}/{resource_path}".format(
  application_version=appversion,
  service_instance=DCX_INSTANCE_NAME,
  resource_path="{token}/json/{data_request_id}".format(token=token, data_request_id=data_request_id)
)
logger.debug("Composed data collection topic: {}".format(topic_data_collection))


def composeDataSample():
    #Datos sensehat
    t = sense.get_temperature()
    p = sense.get_pressure()
    h = sense.get_humidity()
    
    payload = [
    {
      "timestamp": int(round(time.time() * 1000)),
      "temperature": round(t, 1),
      "humidity": round(h, 1),
      "pressure": round(p, 1),
      "bpm": round(random.randint(40, 140)),   #numero aleatorio para las pruebas
    }
    
  ]
    return json.dumps(payload)

#control de conexiones
def on_connect(client, userdata, flags, rc):
  if rc == 0:
    client.connected_flag = True  # set flag
    logger.info("Successfully connected to MQTT server")
  else:
    logger.info("Failed to connect to MQTT code. Returned code=", rc)

# Recepcion de mensajes desde la plataforma cada mensaje está destinado a ahcer alguna funcion en la versión unificada hay más :)
def on_message(client, userdata, message):
  logger.info("Message received: topic [{}]\nbody [{}]".format(message.topic, str(message.payload.decode("utf-8"))))
    #print(message.topic)
  cam="kp1/{}/cex/{}/command/camara/status".format(appversion,token)
  #nocam="kp1/{}/cex/{}/command/nocamara/status".format(appversion,token)
  sos="kp1/{}/cex/{}/command/emergencia/status".format(appversion,token)
  reboot="kp1/{}/cex/{}/command/reboot/status".format(appversion,token)

  global camara
  global led
  if message.topic==cam and camara==0:
      camara=1
  elif message.topic==cam and camara==1:
      server.shutdown()
      camara=0
  if message.topic==sos:
      led=1
  if messsage.topic==reboot:
      cmd='sudo reboot'
      os.system(cmd)


# Initiate server connection
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

# Send metadata once on the first connection
#metadata = composeMetadata(version="v0.0.1")
#client.publish(topic=topic_metadata, payload=metadata)
#logger.info("Sent metadata: {0}\n".format(metadata))

# Send data sample in loop
while 1:
  metadata = composeMetadata(version="v0.0.1")
  client.publish(topic=topic_metadata, payload=metadata)
  logger.info("Sent metadata: {0}\n".format(metadata))
  payload = composeDataSample()
  result = client.publish(topic=topic_data_collection, payload=payload)

  if result.rc != 0:
    logger.info("Server connection lost, attempting to reconnect")
    connectToServer(client=client, host=host, port=port)
  else:
    logger.debug("{0}: Sent next data: {1}".format(token, payload))
  if (camara==1):
      with picamera.PiCamera(resolution='640x480', framerate=24) as camera:
          output = StreamingOutput()
          camera.start_recording(output, format='mjpeg')
          address = ('', 8000)
          server = StreamingServer(address, StreamingHandler)
          server.serve_forever()
          camera.stop_recording()
          time.sleep(5)
          #camara=0
  #if led==1:
      #enceder led que estara en algun puerto
      #GPIO17=1
    
  time.sleep(8)