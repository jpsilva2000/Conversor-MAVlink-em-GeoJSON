import time
import json
from pymavlink import mavutil
from datetime import datetime
import paho.mqtt.client as mqtt
import socket

# Configuração do endereço do servidor
SERVER_IP = '192.168.1.85'  # Endereço IP do servidor local
SERVER_PORT = 48882         # Porta do servidor

# Criar a conexão com o MAVProxy
master = mavutil.mavlink_connection('udpin:0.0.0.0:14552', source_system=255, source_component=158)
master.wait_heartbeat()
print("Heartbeat recebido do MAVProxy")


# MQTT Configuration
MQTT_BROKER = 'test.mosquitto.org'
MQTT_PORT = 1883
MQTT_TOPICS = [
    ('red/sensor/swatter/1', 1),
    ('red/sensor/swatter/2', 2),
    ('red/sensor/swatter/3', 3),
    ('red/sensor/swatter/4', 4)
]
MQTT_TOPIC = 'red/ugs/ist/vigilant'  

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

        # Criar relatório XML com os dados recebidos
        latitude = payload["geometry"]["coordinates"][1] * 1e7
        longitude = payload["geometry"]["coordinates"][0] * 1e7
        transp_id = payload["properties"]["uxs_name"]
        filename = create_bms_xml(latitude, longitude, transp_id)    
           
        # Enviar o relatório XML para o servidor
        send_xml_file(filename, SERVER_IP, SERVER_PORT)
  

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

#################################################################################################
# Função para criar o relatório XML
def create_bms_xml(latitude, longitude, transp_id):
    current_time = datetime.utcnow()
    
    xml_data = f'''<?xml version="1.0" encoding="utf-8"?>
<mtf:FriendlyForceInformation xmlns:mtf="urn:nato:mtf:app-11(d):1:ffi" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
    <OperationCodeword>
        <OperationCodeword>TESTE</OperationCodeword>
    </OperationCodeword>
    <MessageIdentifier>
        <MessageTextFormatIdentifier>FFI</MessageTextFormatIdentifier>
        <Standard>APP-11(D)</Standard>
        <Version>1</Version>
        <Originator>PRT</Originator>
        <ReferenceTimeOfPublication>
            <DateTimeIso>
                <Year4Digit>{current_time.strftime("%Y")}</Year4Digit>
                <MonthNumeric>{current_time.strftime("%m")}</MonthNumeric>
                <Day>{current_time.strftime("%d")}</Day>
                <TimeDelimiter>T</TimeDelimiter>
                <HourTime>{current_time.strftime("%H")}</HourTime>
                <MinuteTime>{current_time.strftime("%M")}</MinuteTime>
                <SecondTime>{current_time.strftime("%S")}</SecondTime>
                <TimeZoneZulu>Z</TimeZoneZulu>
            </DateTimeIso>
        </ReferenceTimeOfPublication>
        <MessageSecurityPolicy>NATO</MessageSecurityPolicy>
        <MessageClassification>
            <SecurityClassificationExtended>UNCLASSIFIED</SecurityClassificationExtended>
        </MessageClassification>
    </MessageIdentifier>
    <GeodeticDatum>
        <GeodeticDatum>WGE</GeodeticDatum>
    </GeodeticDatum>
    <TrackInformation>
        <TrackSource>
            <TransponderId>{transp_id}</TransponderId>
            <System>UGV</System>
            <Subsystem>General</Subsystem>
            <Nationality>
                <GeographicalEntity>PRT</GeographicalEntity>
            </Nationality>
        </TrackSource>
        <TrackSecurityLabel>
            <TrackSecurityPolicy>NATO</TrackSecurityPolicy>
            <TrackSecurityClassification>
                <SecurityClassificationExtended>UNCLASSIFIED</SecurityClassificationExtended>
            </TrackSecurityClassification>
            <TrackSecurityCategory>RELEASABLE TO PRT ARMY</TrackSecurityCategory>
        </TrackSecurityLabel>
        <TrackPositionalData>
            <Time>
                <Year4Digit>{current_time.strftime("%Y")}</Year4Digit>
                <MonthNumeric>{current_time.strftime("%m")}</MonthNumeric>
                <Day>{current_time.strftime("%d")}</Day>
                <TimeDelimiter>T</TimeDelimiter>
                <HourTime>{current_time.strftime("%H")}</HourTime>
                <MinuteTime>{current_time.strftime("%M")}</MinuteTime>
                <SecondTime>{current_time.strftime("%S")}</SecondTime>
                <TimeZoneZulu>Z</TimeZoneZulu>
            </Time>
            <Location>
                <LatitudeDegrees>{str(int(latitude / 1e7))}</LatitudeDegrees>
                <LatitudeMinutes04DecimalPlaces>{"{:.4f}".format(abs((latitude / 1e7 - int(latitude / 1e7)) * 60))}</LatitudeMinutes04DecimalPlaces>
                <LatitudinalHemisphere>{"N" if latitude >= 0 else "S"}</LatitudinalHemisphere>
                <Hyphen>-</Hyphen>
                <LongitudeDegrees>{str(abs(int(longitude / 1e7)))}</LongitudeDegrees>
                <LongitudeMinutes04DecimalPlaces>{"{:.4f}".format(abs((longitude / 1e7 - int(longitude / 1e7)) * 60))}</LongitudeMinutes04DecimalPlaces>
                <LongitudinalHemisphere>{"E" if longitude >= 0 else "W"}</LongitudinalHemisphere>
            </Location>
        </TrackPositionalData>
        <TrackIdentificationData>
            <UnitSymbol>SFGPEV---------</UnitSymbol>
            <SymbolStandard>APP-6(B)</SymbolStandard>
            <UnitShortName>{transp_id}</UnitShortName>
        </TrackIdentificationData>
    </TrackInformation>
</mtf:FriendlyForceInformation>'''

    filename = "bms_report.xml"
    with open(filename, 'w') as f:
        f.write(xml_data)
    
    return filename

# Função para enviar o arquivo XML para o servidor
def send_xml_file(filename, server_ip, server_port):
    with open(filename, 'rb') as f:
        data = f.read()
    
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((server_ip, server_port))
            s.sendall(data)
            print(f"Arquivo {filename} enviado para {server_ip}:{server_port}")
    except Exception as e:
        print(f"Erro ao enviar o arquivo: {e}")

#########################################################################################################

# Loop principal
while True:
    try:
        # Listen for incoming MAVLink messages
        msg = master.recv_match(blocking=False)
        if msg:
            if msg.get_type() == 'GPS_RAW_INT':
                gps_raw_int(msg)
                latitude = msg.lat
                longitude = msg.lon
                transp_id = "LAFVIN"
                filename = create_bms_xml(latitude, longitude, transp_id)    
                # Enviar o relatório XML para o servidor
                send_xml_file(filename, SERVER_IP, SERVER_PORT)

            elif msg.get_type() == 'GLOBAL_POSITION_INT':
                global_position_int(msg)
                latitude = msg.lat
                longitude = msg.lon
                transp_id = "LAFVIN"
                filename = create_bms_xml(latitude, longitude, transp_id)    
                # Enviar o relatório XML para o servidor
                send_xml_file(filename, SERVER_IP, SERVER_PORT)

    except mavutil.mavlink.MAVLinkException as e:
        print(f"MAVLink communication error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
