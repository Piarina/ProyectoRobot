import os 
# Retorna la temperatura de la CPU de la RPi como float
def measure_temp():
    temp = os.popen('vcgencmd measure_temp').readline() # función que hace todo
    print(temp)
    return(temp.replace("temp=","").replace("'C\n",""))

print(float(measure_temp()))