import time
import json
from pymavlink import mavutil

# Create the connection

master = mavutil.mavlink_connection('udp:127.0.0.1:14551')


# Valores fixos do JSON
fixed_values = {
    "type": "Feature",
    "geometry": {
        "type": "Point",
        "coordinates": [
            0,  # Placeholder, será atualizado com os valores recebidos
            0,  # Placeholder, será atualizado com os valores recebidos
            0  # Placeholder, será atualizado com os valores recebidos
        ]
    },
    "properties": {
        "timestamp": 0,  # Placeholder, será atualizado com os valores recebidos
        "object": "UAS",
        "msg_type": "uas_self_presence",
        "uas_name": "AR3",
        "gcs_name": "Tekever",
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
    
    print('criação json')

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


# Loop principal
while True:
    try:
        # Listen for incoming messages
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


#mavproxy.py --master=udpout:172.18.78.2:14550 --out=udp:127.0.0.1:14551
# --load-module='/home/joao/testes/conexao_bms.py'

