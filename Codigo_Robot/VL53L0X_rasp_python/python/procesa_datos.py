# -*- encoding: utf-8 -*-

import time
import math

def calcula_angulos(xH_raw, xL_raw, yH_raw, yL_raw, zH_raw, zL_raw):

    # Combino los dos bytes
    x_raw = xH_raw*256 + xL_raw
    y_raw = yH_raw*256 + yL_raw
    z_raw = zH_raw*256 + zL_raw

    # Deshago complemento a 2
    if x_raw > 32767:
        x_raw -= 65534

    if y_raw > 32767:
        y_raw -= 65534

    if z_raw > 32767:
        z_raw -= 65534


    # Calculo componentes del vector
    # En modo +/- 2g, sensibilidad = 0.000061 g/LSB
    x_accel = x_raw*0.000061
    y_accel = y_raw*0.000061
    z_accel = z_raw*0.000061

    # Calculo modulo vector fuerza
    R = math.sqrt(x_accel*x_accel + y_accel*y_accel + z_accel*z_accel)

    # Calculo angulos
    Ang_x = math.degrees(math.asin(x_accel/R))
    Ang_y = math.degrees(math.asin(y_accel/R))

    return Ang_x, Ang_y

def correccion_gyro(xHg_raw, xLg_raw, yHg_raw, yLg_raw):

    dt = 20  # ms que dura un bucle de gyro

    t_start = time.time()    # referencia de tiempo

    # Combino los dos bytes
    xg_raw = xHg_raw*256 + xLg_raw
    yg_raw = yHg_raw*256 + yLg_raw
    
    # Deshago complemento a 2
    if xg_raw > 32767:
        xg_raw -= 65534

    if yg_raw > 32767:
        yg_raw -= 65534

    # Calculo factores de correccion
    giro_x = xg_raw*0.004375*dt/1000
    giro_y = yg_raw*0.004375*dt/1000

    # Esperamos a que pasen 20 ms
    while ((time.time() - t_start)*1000000) < dt*1000:
        time.sleep(0.00001)

    return giro_x, giro_y

    
