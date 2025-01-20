import time
import serial
from pymavlink import mavutil
from mission1 import execute_mission_type_1
from mission2 import execute_mission_type_2
from mission3 import execute_mission_type_3

def connect_to_flight_controller(port='/dev/ttyUSB0', baudrate=57600):
    # Open serial connection to the flight controller
    ser = serial.Serial(port, baudrate)
    return mavutil.mavlink_connection(ser)

def main():
    print("Main function is executing.")
    print("Choose connection option (1: Drone): ")

    connection_option = input()
    if connection_option == "1":
        port = input("Enter serial port (default: /dev/ttyUSB0): ") or '/dev/ttyUSB0'
        baudrate = int(input("Enter baud rate (default: 57600): ") or 57600)
        mavlink_connection = connect_to_flight_controller(port, baudrate)
    else:
        print("Invalid option. Exiting.")
        return

    # Main loop to handle incoming messages from Mission Planner
    while True:
        msg = mavlink_connection.recv_match(blocking=True, timeout=1)
        if msg:
            print("Received message:", msg)

            # Handle mission messages from Mission Planner
            if msg.get_type() == 'MISSION_ITEM':
                # Extract mission data from the message
                mission_type = msg.custom_type  # Assuming custom_type is used to define mission type
                lat = msg.x  # Latitude from MAVLink
                lng = msg.y  # Longitude from MAVLink
                alt = msg.z  # Altitude from MAVLink

                # Execute the corresponding mission based on the mission type
                if mission_type == 1:
                    execute_mission_type_1(lat, lng, alt, mavlink_connection)
                elif mission_type == 2:
                    execute_mission_type_2(lat, lng, alt, mavlink_connection)
                elif mission_type == 3:
                    execute_mission_type_3(lat, lng, alt, mavlink_connection)
                else:
                    print("Unknown mission type. Skipping execution.")

if __name__ == "__main__":
    main()
