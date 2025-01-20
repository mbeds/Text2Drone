import time

def execute_mission_type_1(lat, lng, alt, mavlink_connection):
    print(f"Executing Mission Type 1: Navigating to lat={lat}, lng={lng}, alt={alt}")
    # Add logic to move the drone to the coordinates or perform mission-specific tasks
    mavlink_connection.mav.set_position_target_global_int_send(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000, int(lat * 1e7), int(lng * 1e7), alt, 0, 0, 0, 0, 0, 0, 0
    )
    # Wait for completion or add specific mission logic
    time.sleep(5)
    print("Mission Type 1 completed.")
