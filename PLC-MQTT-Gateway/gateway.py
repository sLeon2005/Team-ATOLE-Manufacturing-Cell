#
# This application reads process data from a Siemens S7-1200
# PLC using Snap7 and publishes the data to an MQTT broker.
#
# Requirements:
#
# 1. Enable PUT/GET communication in the PLC.
#
# 2. Disable Optimized Block Access in the PLC Data Block.
#
# 3. Configure PLC and MQTT settings in:
#
#       config.py
#
# 4. Install required Python packages:
#
#       pip install python-snap7
#       pip install paho-mqtt
#
# Configuration:
#
# The project uses a local configuration file:
#
#       config.py
#
# Example:
#
#       PLC_IP = "192.168.0.3"
#
#       MQTT_BROKER = "192.168.0.100"
#       MQTT_PORT = 1883
#
# Author:
#
#       ATOLE S.A. de C.V.
#

from config import *

import snap7
from snap7.util import get_int, get_bool, get_real, get_string
import paho.mqtt.client as mqtt
import time
import sys

# PLC CONFIG
DB_NUMBER = 99
PLC_START_ADDRESS = 0
PLC_SIZE = 94   # bytes a leer

# MQTT TOPICS
TOPIC_EMERGENCY_STOP_PNEU = "pneumatic/emergency"
TOPIC_EMERGENCY_STOP_MAC = "machining/emergency"

TOPIC_COBOTS_MOVING = "cobot/moving"

TOPIC_PNEUMATIC_POWER = "pneumatic/power"
TOPIC_MACHINING_POWER = "machining/power"
TOPIC_TOTAL_POWER = "total/power"
TOPIC_TOTAL_ENERGY = "total/energy" # dos tópicos nuevos
TOPIC_TOTAL_COST = "total/cost"

TOPIC_AIR_PRESSURE = "pneumatic/pressure"

TOPIC_CHOCOLATE_UNITS = "chocolate/delivered"
TOPIC_VANILLA_UNITS = "vanilla/delivered"
TOPIC_STRAWBERRY_UNITS = "strawberry/delivered"
TOPIC_TOTAL_UNITS = "total/delivered"

TOPIC_ONLINE_USER = "user/logged"
TOPIC_ONLINE_ROLE = "role"

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

# CFE constant
CFE_PRICE_PER_KWH = 3.5 # 3.50 MXN/kWh

# Initialize energy consumption variable
energyWh = 0.0

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
        totalUnitsFinished = chocolateUnitsFinished + vanillaUnitsFinished + strawberryUnitsFinished

        onlineUser = get_string(data, 20)
        onlineRole = get_string(data, 62)

        # Power and consumptions metrics
        totalPower = pneumaticPower + machiningPower

        deltaTimeHours = 1 / 3600 # based on 1 second loop delay
        energyWh += totalPower * deltaTimeHours
        energyKWh = energyWh / 1000
        costMXN = energyKWh * CFE_PRICE_PER_KWH

        print("\n-------------------")

        print(f"Emergency Stop Pneumatics: {emergencyStop_Pneu}")
        print(f"Emergency Stop Machining: {emergencyStop_Mac}")
        print(f"Cobots Moving: {cobotsMoving}")

        print(f"Pneumatic Power: {pneumaticPower}")
        print(f"Machining Power: {machiningPower}")
        print(f"Total Power: {totalPower}")
        print(f"Energy Consumed (kWh): {energyKWh:.6f}")
        print(f"Estimated Cost (MXN): ${costMXN:.2f}")

        print(f"Air Pressure: {airPressure}")

        print(f"Chocolate Units Finished: {chocolateUnitsFinished}")
        print(f"Vanilla Units Finished: {vanillaUnitsFinished}")
        print(f"Strawberry Units Finished: {strawberryUnitsFinished}")
        print(f"Total Units Finished: {totalUnitsFinished}")

        print(f"Online User: {onlineUser}")
        print(f"Online Role: {onlineRole}")

        # MQTT PUBLISH
        client.publish(TOPIC_EMERGENCY_STOP_PNEU, str(emergencyStop_Pneu))
        client.publish(TOPIC_EMERGENCY_STOP_MAC, str(emergencyStop_Mac))

        client.publish(TOPIC_COBOTS_MOVING, str(cobotsMoving))

        client.publish(TOPIC_PNEUMATIC_POWER, str(pneumaticPower))
        client.publish(TOPIC_MACHINING_POWER, str(machiningPower))
        client.publish(TOPIC_TOTAL_POWER, str(totalPower))
        client.publish(TOPIC_TOTAL_ENERGY, str(energyKWh))
        client.publish(TOPIC_TOTAL_COST, str(costMXN))

        client.publish(TOPIC_AIR_PRESSURE, str(airPressure))

        client.publish(TOPIC_CHOCOLATE_UNITS, str(chocolateUnitsFinished))
        client.publish(TOPIC_VANILLA_UNITS, str(vanillaUnitsFinished))
        client.publish(TOPIC_STRAWBERRY_UNITS, str(strawberryUnitsFinished))
        client.publish(TOPIC_TOTAL_UNITS, str(totalUnitsFinished))

        client.publish(TOPIC_ONLINE_USER, str(onlineUser))
        client.publish(TOPIC_ONLINE_ROLE, str(onlineRole))

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
