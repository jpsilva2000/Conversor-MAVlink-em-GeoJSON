##########################################################################################################################################################################################
#Desenvolvido em parceria pela equipa do IST a participar no ARTEX e pelo Alferes-Aluno João Silva da AM
##########################################################################################################################################################################################

import time
import json
from pymavlink import mavutil
import paho.mqtt.client as mqtt

# MQTT Configuration
MQTT_BROKER = 'test.mosquitto.org'
MQTT_PORT = 1883
MQTT_TOPIC = 'uas/telemetry'

# Create MQTT client and connect to the broker
mqtt_client = mqtt.Client()

# Callback function for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    # Subscribe to the topic
    client.subscribe(MQTT_TOPIC)

# Callback function for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print("Message received from MQTT topic:", msg.topic)
    json_data = msg.payload.decode('utf-8')
    json_to_mavlink(json_data)

# Bind the on_connect and on_message functions to the MQTT client
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

# Connect to the broker
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Start the MQTT client loop in a separate thread
mqtt_client.loop_start()

# Create the connection to MAVLink
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# Valores fixos do JSON
fixed_values = {
    "type": "Feature",
    "geometry": {
        "type": "Point",
        "coordinates": [
            0,  # Placeholder, será atualizado com os valores recebidos
            0,  # Placeholder, será atualizado com os valores recebidos
            0   # Placeholder, será atualizado com os valores recebidos
        ]
    },
    "properties": {
        "timestamp": 0,  # Placeholder, será atualizado com os valores recebidos
        "object": "UAS",
        "msg_type": "uas_self_presence",
        "uas_name": "IST_UAS",
        "gcs_name": "IST_GCS",
        "heading": 0,  # Placeholder, será atualizado com os valores recebidos
        "mission": "op1_recce1",
        "comments": "Goal: detect ENY on AO1"
    }
}

def create_json(latitude, longitude, altitude, heading, timestamp):
    # Atualiza os valores no dicionário
    fixed_values["geometry"]["coordinates"] = [longitude / 1e7, latitude / 1e7, altitude / 1e3]
    fixed_values["properties"]["timestamp"] = timestamp
    fixed_values["properties"]["heading"] = heading / 100

    # Converte para JSON
    json_data = json.dumps(fixed_values, indent=4)
    
    # Salva num arquivo
    with open('output.json', 'w') as json_file:
        json_file.write(json_data)
    
    print('JSON criado:', json_data)

    # Publish JSON to MQTT
    mqtt_client.publish(MQTT_TOPIC, json_data)
    print('JSON publicado no MQTT:', json_data)

def gps_raw_int(msg):
    """Handles GPS_RAW_INT"""
    latitude = msg.lat
    longitude = msg.lon
    altitude = msg.alt
    heading = msg.hdg_acc
    timestamp = msg.time_usec // 1000000 if msg.time_usec else int(time.time())
    create_json(latitude, longitude, altitude, heading, timestamp)

def global_position_int(msg):
    """Handles GLOBAL_POSITION_INT"""
    latitude = msg.lat
    longitude = msg.lon
    altitude = msg.alt
    heading = msg.hdg
    timestamp = msg.time_boot_ms // 1000 if msg.time_boot_ms else int(time.time())
    create_json(latitude, longitude, altitude, heading, timestamp)

def json_to_mavlink(json_data):
    """Converts JSON data to a MAVLink ADSB_VEHICLE message and sends it."""
    try:
        data = json.loads(json_data)

        latitude = data["geometry"]["coordinates"][1] * 1e7  # Convert back to degE7
        longitude = data["geometry"]["coordinates"][0] * 1e7  # Convert back to degE7
        altitude = data["geometry"]["coordinates"][2] * 1e3  # Convert back to mm
        timestamp = data["properties"]["timestamp"]
        heading = data["properties"]["heading"] * 100  # Convert back to cdeg

        print(f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}, Heading: {heading}")

        # Create a MAVLink ADSB_VEHICLE message
        msg = master.mav.adsb_vehicle_encode(
            123456,  # ICAO address (example value)
            int(latitude),  # Latitude in degrees * 1e7
            int(longitude),  # Longitude in degrees * 1e7
            0,  # Altitude type (0 for barometric altitude)
            int(altitude),  # Altitude in mm
            int(heading),  # Course over ground in centidegrees
            0,  # Horizontal velocity in cm/s
            0,  # Vertical velocity in cm/s
            b"CALLSIGN",  # Callsign (example value) encoded to bytes
            0,  # Emitter type (example value)
            0,  # Time since last communication in seconds
            0,  # Flags (example value)
            0   # Squawk code (example value)
        )
        
        # Send the ADSB_VEHICLE message
        master.mav.send(msg)
        print('MAVLink ADSB_VEHICLE message sent:', msg)
    except Exception as e:
        print(f"Error converting JSON to MAVLink: {e}")

# Loop principal
while True:
    try:
        # Listen for incoming MAVLink messages
        msg = master.recv_match(blocking=False)
        if msg:
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                global_position_int(msg)
            elif msg.get_type() == 'GPS_RAW_INT':
                gps_raw_int(msg)
    except mavutil.mavlink.MAVLinkException as e:
        print(f"MAVLink communication error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

    time.sleep(1)
