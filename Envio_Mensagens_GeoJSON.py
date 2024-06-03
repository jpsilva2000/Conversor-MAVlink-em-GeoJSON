import time
import json
import paho.mqtt.client as mqtt

# MQTT Configuration
MQTT_BROKER = 'test.mosquitto.org'
MQTT_PORT = 1883
MQTT_TOPIC = 'uas/telemetry'

# Create MQTT client and connect to the broker
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Função para enviar mensagens JSON
def send_test_messages():
    # Lista de mensagens de teste
    test_messages = [
        {
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates": [-85.206, 45.4124, 100]
            },
            "properties": {
                "timestamp": 1627473047,
                "object": "UAS",
                "msg_type": "uas_self_presence",
                "uas_name": "AR5",
                "gcs_name": "Tekever",
                "heading": 90,
                "mission": "op1_recce1",
                "comments": "Goal: detect ENY on AO1"
            }
        },
        {
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates": [32, -9, 105]
            },
            "properties": {
                "timestamp": 1627473050,
                "object": "UAS",
                "msg_type": "uas_self_presence",
                "uas_name": "AR4",
                "gcs_name": "Tekever",
                "heading": 95,
                "mission": "op1_recce1",
                "comments": "Goal: detect ENY on AO1"
            }
        },
        {
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates": [34, -10, 100]
            },
            "properties": {
                "timestamp": 1627473047,
                "object": "UAS",
                "msg_type": "uas_self_presence",
                "uas_name": "Ogassa",
                "gcs_name": "UAVision",
                "heading": 90,
                "mission": "op1_recce1",
                "comments": "Goal: detect ENY on AO1"
            }
        }
        # Adicione mais mensagens de teste conforme necessário
    ]

    for msg in test_messages:
        json_data = json.dumps(msg)
        mqtt_client.publish(MQTT_TOPIC, json_data)
        print('Enviado JSON para MQTT:', json_data)
        time.sleep(1)  # Pausa de 1 segundo entre mensagens

# Envie mensagens de teste
send_test_messages()

# Desconecte do servidor MQTT
mqtt_client.disconnect()
