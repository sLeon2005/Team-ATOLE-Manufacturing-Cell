from config import *

import snap7
from snap7.util import get_int, get_bool, get_real, get_string
import paho.mqtt.client as mqtt
import time
import sys

# PLC CONFIG
DB_NUMBER = 99
PLC_START_ADDRESS = 0
PLC_SIZE = 62   # bytes a leer

# MQTT TOPICS
TOPIC = "yopublico" #soon to be discarded

TOPIC_EMERGENCY_STOP_PNEU = "factory/safety/emergencyStopPneumatics"
TOPIC_EMERGENCY_STOP_MAC = "factory/safety/emergencyStopMachining"

TOPIC_COBOTS_MOVING = "factory/cobots/moving"

TOPIC_PNEUMATIC_POWER = "factory/pneumatics/power"
TOPIC_MACHINING_POWER = "factory/machining/power"

TOPIC_AIR_PRESSURE = "factory/pneumatics/airPressure"

TOPIC_CHOCOLATE_UNITS = "factory/production/chocolateUnitsFinished"
TOPIC_VANILLA_UNITS = "factory/production/vanillaUnitsFinished"
TOPIC_STRAWBERRY_UNITS = "factory/production/strawberryUnitsFinished"

TOPIC_ONLINE_USER = "factory/system/onlineUser"

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
# MAIN LOOP
print("\nGateway started successfully.")

while True:

    try:

        # Read PLC DB
        data = plc.db_read(DB_NUMBER, PLC_START_ADDRESS, PLC_SIZE)

        emergencyStop_Pneu = get_bool(data, 0, 0)
        emergencyStop_Mac = get_bool(data, 0, 1)
        cobotsMoving = get_bool(data, 0, 2)

        pneumaticPower = get_real(data, 2)
        machiningPower = get_real(data, 6)

        airPressure = get_real(data, 10)

        chocolateUnitsFinished = get_int(data, 14)
        vanillaUnitsFinished = get_int(data, 16)
        strawberryUnitsFinished = get_int(data, 18)

        onlineUser = get_string(data, 20)

        print("\n-------------------")

        print(f"Emergency Stop Pneumatics: {emergencyStop_Pneu}")
        print(f"Emergency Stop Machining: {emergencyStop_Mac}")
        print(f"Cobots Moving: {cobotsMoving}")

        print(f"Pneumatic Power: {pneumaticPower}")
        print(f"Machining Power: {machiningPower}")
        print(f"Air Pressure: {airPressure}")

        print(f"Chocolate Units Finished: {chocolateUnitsFinished}")
        print(f"Vanilla Units Finished: {vanillaUnitsFinished}")
        print(f"Strawberry Units Finished: {strawberryUnitsFinished}")

        print(f"Online User: {onlineUser}")

        # MQTT PUBLISH
        client.publish(TOPIC_EMERGENCY_STOP_PNEU, str(emergencyStop_Pneu))
        client.publish(TOPIC_EMERGENCY_STOP_MAC, str(emergencyStop_Mac))

        client.publish(TOPIC_COBOTS_MOVING, str(cobotsMoving))

        client.publish(TOPIC_PNEUMATIC_POWER, str(pneumaticPower))
        client.publish(TOPIC_MACHINING_POWER, str(machiningPower))

        client.publish(TOPIC_AIR_PRESSURE, str(airPressure))

        client.publish(TOPIC_CHOCOLATE_UNITS, str(chocolateUnitsFinished))
        client.publish(TOPIC_VANILLA_UNITS, str(vanillaUnitsFinished))
        client.publish(TOPIC_STRAWBERRY_UNITS, str(strawberryUnitsFinished))

        client.publish(TOPIC_ONLINE_USER, str(onlineUser))

        print("\nMQTT Publish Success")

        time.sleep(1)

    except Exception as e:

        print("\nRUNTIME ERROR")
        print(f"Reason: {e}")

        print("\nPossible Causes:")
        print("- PLC disconnected")
        print("- MQTT broker disconnected")
        print("- Network interruption")
        print("- Invalid DB address")
        print("- Incorrect DB size")
        print("- String size mismatch")

        time.sleep(2)
# CLEANUP
try:
    plc.disconnect()
    print("\nPLC disconnected.")

except:
    pass