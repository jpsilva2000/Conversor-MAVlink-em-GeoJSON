##########################################################################################################################################################################################
#Desenvolvido em parceria pela equipa do IST a participar no ARTEX e pelo Alferes-Aluno João Silva da AM
##########################################################################################################################################################################################

import time
from pymavlink import mavutil


# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()

def send_heartbeat():
    """Sends a heartbeat message with predefined parameters."""
    try:
        # Create a MAVLink heartbeat message
        msg = master.mav.heartbeat_encode(
            mavutil.mavlink.MAV_TYPE_GENERIC,  # Type of MAV (Ground Control Station)
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            0,
            0,
            mavutil.mavlink.MAV_STATE_STANDBY
        )
        # Send the heartbeat message
        master.mav.send(msg)
    except Exception as e:
        print(f"Error sending heartbeat: {e}")

def send_position(latitude, longitude, altitude):
    """Sends a position message with given parameters."""
    try:
        # Create a MAVLink message for position
        msg = master.mav.global_position_int_encode(
            0,
            int(latitude * 1e7),  # Latitude in degrees * 1e7
            int(longitude * 1e7),  # Longitude in degrees * 1e7
            int(altitude * 1e3),  # Altitude in meters * 1000
            0,  # Relative altitude in meters (not supported)
            0,  # Ground X-Speed (not supported)
            0,  # Ground Y-Speed (not supported)
            0,  # Ground Z-Speed (not supported)
            1   #Vehicle heading
        )
        # Send the position message
        master.mav.send(msg)
    except Exception as e:
        print(f"Error sending position: {e}")

# Send heartbeat messages and receive messages
while True:
    try:
        # Send heartbeat every second
        send_heartbeat()
        
        # Send position every 5 seconds (latitude, longitude, altitude)
        send_position(38.7, -9, 10)  # Examplo da localização de Famalicão

    except mavutil.mavlink.MAVLinkException as e:
        print(f"MAVLink communication error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

    time.sleep(1)  # Send heartbeat every second


