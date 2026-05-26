from config import *

import snap7
from snap7.util import get_int, get_bool
import paho.mqtt.client as mqtt
import time
import sys

# PLC CONFIG
DB_NUMBER = 2
PLC_START_ADDRESS = 0
PLC_SIZE = 3   # bytes a leer

# MQTT CONFIG
TOPIC = "yopublico"

# INIT PLC
try:
    plc = snap7.client.Client()

    print(f"Attempting PLC connection to {PLC_IP}...")

    plc.connect(PLC_IP, 0, 1)

    if plc.get_connected():
        print("PLC connection established successfully.")
    else:
        print("PLC connection failed.")

        print("\nTroubleshooting:")
        print("- Verify PLC IP address")
        print("- Check Ethernet connection")
        print("- Ensure PLC and PC are on the same network")
        print("- Enable PUT/GET communication in TIA Portal")
        print("- Disable optimized block access in DB")

        sys.exit()

except Exception as e:

    print("\nPLC CONNECTION ERROR")
    print(f"Reason: {e}")

    print("\nTroubleshooting:")
    print("- Verify PLC IP address")
    print("- Check Ethernet cable")
    print("- Ping the PLC from your PC")
    print("- Ensure CPU is in RUN mode")
    print("- Verify Rack=0 and Slot=1")
    print("- Enable PUT/GET communication")

    sys.exit()

# INIT MQTT
try:
    client = mqtt.Client()

    print(f"\nAttempting MQTT connection to {MQTT_BROKER}:{MQTT_PORT}...")

    client.connect(MQTT_BROKER, MQTT_PORT, 60)

    print("MQTT broker connection established successfully.")

except Exception as e:

    print("\nMQTT CONNECTION ERROR")
    print(f"Reason: {e}")

    print("\nTroubleshooting:")
    print("- Verify broker IP address")
    print("- Ensure Mosquitto broker is running")
    print("- Verify port 1883 is open")
    print("- Check Windows firewall rules")
    print("- Ensure both devices are on the same network")

    try:
        plc.disconnect()
    except:
        pass

    sys.exit()

# MAIN LOOP
print("\nGateway started successfully.")

while True:

    try:
        # Read PLC DB
        data = plc.db_read(DB_NUMBER, PLC_START_ADDRESS, PLC_SIZE)

        # INT
        counterValue = get_int(data, 0)

        # BOOLS
        greenBTN = get_bool(data, 2, 0)
        redBTN = get_bool(data, 2, 1)
        counterFinished = get_bool(data, 2, 2)

        # Console output
        print("\n-------------------")
        print(f"Counter Value: {counterValue}")
        print(f"Green Button: {greenBTN}")
        print(f"Red Button: {redBTN}")
        print(f"Counter Finished: {counterFinished}")

        # MQTT Publish
        client.publish(TOPIC, counterValue)

        print(f"MQTT Publish Success -> Topic: '{TOPIC}' | Payload: {counterValue}")

        time.sleep(0.5)

    except Exception as e:
        print("\nRUNTIME ERROR")
        print(f"Reason: {e}")

        print("\nPossible Causes:")
        print("- PLC disconnected")
        print("- MQTT broker disconnected")
        print("- Network interruption")
        print("- Invalid DB address")
        print("- Incorrect DB size")

        time.sleep(2)

# CLEANUP
try:
    plc.disconnect()
    print("\nPLC disconnected.")

except:
    pass