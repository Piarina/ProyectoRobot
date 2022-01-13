
# Código para detectar y comenzar un movimiento de esquiva con el lidar VL53L0X (se queman fácil) van por I2C y a 5V
# MUY IMPORTANTE MANTENER LA ESTRUCTURA DE CARPETAS, sino no funciona
import time
import VL53L0X

# Create a VL53L0X object
tof = VL53L0X.VL53L0X()

# Start ranging
tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

timing = tof.get_timing()           # Sincronización, importante pq sino puede funcionar raro
if (timing < 20000):
    timing = 20000
print ("Timing %d ms" % (timing/1000))
tiempo=time.time()                  # para no hacerlo constantemente
recto = 1                           #auxiliar
while True:
    if (recto == 1):                # Modo de funcionamiento normal, ej: ir recto 
        if tiempo+timing/10000000.00 < time.time():#timing/10000000.00 es cada cuanto mides distancia
            #print(tiempo)                          # cosas de debug
            distance = tof.get_distance()           #mides
            print ("modo normal")                   # cosas de debug
            if (distance < 500):                    # a que distancia consideras el obstáculo, el lidar mide bien hasta 80cm
                print("Obstaculo")                  # cosas de debug
                recto =0                            # hay obstaculo y pasas a modo de funcionamiento alternativo, esquiva
            print ("%d mm, %d cm" % (distance, (distance/10)))  # cosas de debug
            tiempo = time.time()
                
    else:                                           #modo de funcionamiento alternativo, esquiva
        if tiempo+timing/100000) <=  time.time(): # mismo de antes
            distance = tof.get_distance()           #mides
            if (distance < 500):                    # sigue habiendo obstáculo
                print("Gira")                       # cosas de debug
                recto = 0
            else:
                recto = 1
                print("Ya no hay Obstaculo")        # cosas de debug
            #print ("%d mm, %d cm" % (distance, (distance/10)))
            tiempo = time.time()

    #            time.sleep(timing/1000000.00)

    #            tof.stop_ranging()                 # SIRVE PARA PARAR EL LIDAR Y QUE NO ESTÉ FUNCIONANDO CONSTANTEMENTE, IMPORTANTE

    

                
               

