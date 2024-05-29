import time
import json
from pymavlink import mavutil

# Cria a conexão
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

def json_to_mavlink(json_data):
    """Converts JSON data to a MAVLink ADSB_VEHICLE message and sends it."""
    try:
        data = json.loads(json_data)

        latitude = data["geometry"]["coordinates"][1] * 1e7  # Convert back to degE7
        longitude = data["geometry"]["coordinates"][0] * 1e7  # Convert back to degE7
        altitude = data["geometry"]["coordinates"][2] * 1e3  # Convert back to mm
        heading = data["properties"]["heading"] * 100  # Convert back to cdeg

        # Debug print to check values
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
        print('MAVLink message sent:', msg)
    except Exception as e:
        print(f"Error converting JSON to MAVLink: {e}")

# Loop principal
while True:
    try:
        # Read JSON from file and send as MAVLink message
        with open('input.json', 'r') as json_file:
            json_data = json_file.read()
            json_to_mavlink(json_data)
    except mavutil.mavlink.MAVLinkException as e:
        print(f"MAVLink communication error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

    time.sleep(1)
