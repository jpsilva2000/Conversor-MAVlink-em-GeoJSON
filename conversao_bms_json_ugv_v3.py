import time
import json
from pymavlink import mavutil
import paho.mqtt.client as mqtt

# MQTT Configuration
MQTT_BROKER = '83.223.228.1'
MQTT_PORT = 1883
MQTT_TOPICS = [
    ('blue/ugs/uavision', 1),
    ('blue/ugs/inesctec/tribe', 2)
]
MQTT_TOPIC = 'blue/ugs/ist/vigilant'  

# Create MQTT client and connect to the broker
mqtt_client = mqtt.Client()

# Dictionary to store topic-to-id mapping for quick lookup
topic_to_id = {topic: id for topic, id in MQTT_TOPICS}

# Callback function for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    # Subscribe to each topic in the list
    for topic, _ in MQTT_TOPICS:
        client.subscribe(topic)

# Função callback quando uma mensagem é recebida
def on_message(client, userdata, msg):
    print("Message received from MQTT topic:", msg.topic)
    json_data = msg.payload.decode('utf-8')
    
    try:
        # Tentativa de decodificar o payload da mensagem como JSON
        payload = json.loads(json_data)
        # Pass the topic ID along with the JSON data
        json_to_mavlink(json_data, topic_to_id[msg.topic])
        print("Mensagem ADSB enviada")
    except json.JSONDecodeError:
        print(f"Received non-JSON message on topic {msg.topic}")

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
        "object": "UGS",
        "msg_type": "uxs_self_presence",
        "uxs_name": "Vigilant",
        "gcs_name": "Ist",
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
    timestamp = int(time.time())
    create_json(latitude, longitude, altitude, heading, timestamp)

def global_position_int(msg):
    """Handles GLOBAL_POSITION_INT"""
    latitude = msg.lat
    longitude = msg.lon
    altitude = msg.alt
    heading = msg.hdg
    timestamp = int(time.time())
    create_json(latitude, longitude, altitude, heading, timestamp)

def json_to_mavlink(json_data, topic_id):
    """Converts JSON data to a MAVLink ADSB_VEHICLE message and sends it."""
    try:
        data = json.loads(json_data)

        latitude = data["geometry"]["coordinates"][1] * 1e7  # Convert back to degE7
        longitude = data["geometry"]["coordinates"][0] * 1e7  # Convert back to degE7
        altitude = data["geometry"]["coordinates"][2] * 1e3  # Convert back to mm
        timestamp = data["properties"]["timestamp"]
        heading = data["properties"]["heading"] * 100  # Convert back to cdeg

        print(f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}, Heading: {heading}")

        # Use the topic ID
        icao_id = topic_id

        # Create a MAVLink ADSB_VEHICLE message
        msg = master.mav.adsb_vehicle_encode(
            icao_id,  # ICAO address (example value)
            int(latitude),  # Latitude in degrees * 1e7
            int(longitude),  # Longitude in degrees * 1e7
            0,  # Altitude type (0 for barometric altitude)
            int(altitude),  # Altitude in mm
            int(heading),  # Course over ground in centidegrees
            0,  # Horizontal velocity in cm/s
            0,  # Vertical velocity in cm/s
            data["properties"]["uxs_name"].encode('UTF-8'), 
            0,  # Emitter type (example value)
            0,  # Time since last communication in seconds
            447,  # Flags (example value)
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
