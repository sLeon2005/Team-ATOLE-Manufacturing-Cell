import snap7
from snap7.util import get_int, get_bool
import paho.mqtt.client as mqtt
import time

# PLC CONFIG

PLC_IP = "192.168.0.3"

DB_NUMBER = 2
PLC_START_ADDRESS = 0
PLC_SIZE = 3   # bytes a leer

# MQTT CONFIG

MQTT_BROKER = "10.25.110.236"   # IP de la PC con Mosquitto
MQTT_PORT = 1883
TOPIC = "yopublico"

# INIT PLC

plc = snap7.client.Client()
plc.connect(PLC_IP, 0, 1)
print("Conectado al PLC")

# INIT MQTT

client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT, 60)
print("Conectado al broker MQTT")

# MAIN LOOP

while True:
    # Leer bytes del DB
    data = plc.db_read(DB_NUMBER, PLC_START_ADDRESS, PLC_SIZE)

    # -------- INT --------
    counterValue = get_int(data, 0)

    # -------- BOOLS --------
    greenBTN = get_bool(data, 2, 0)
    redBTN = get_bool(data, 2, 1)
    counterFinished = get_bool(data, 2, 2)

    # Mostrar valores
    print("-------------------")
    print("Counter:", counterValue)
    print("Green BTN:", greenBTN)
    print("Red BTN:", redBTN)
    print("Counter Finished:", counterFinished)

    # Enviar el valor del contador por MQTT
    client.publish(TOPIC, counterValue)
    print(f"Counter Value enviado a '{TOPIC}': {counterValue}")

    time.sleep(0.5)