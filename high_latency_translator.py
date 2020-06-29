import time
from pymavlink import mavutil

vehicle = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
basestation = mavutil.mavlink_connection('udpout:0.0.0.0:14770')

refresh_interval = 10  # Seconds
heartbeat_count = 0


def wait_conn(master):
    msg = None
    while not msg:
        master.mav.ping_send(
            int(time.time()),  # Unix time
            0,  # Ping number
            0,  # Request ping of all systems
            0  # Request ping of all components
        )
        msg = master.recv_match()
        time.sleep(0.5)


def high_latency_out(v, b):
    b.mav.high_latency_send(
        v.base_mode,  # base_mode
        0,  # custom_mode
        0,  # landed_state
        v.roll,  # roll
        v.pitch,  # pitch
        v.heading,  # heading
        v.throttle,  # throttle
        0,  # heading_sp
        v.latitude,  # latitude
        v.longitude,  # longitude
        v.altitude_amsl,  # altitude_amsl
        0,  # altitude_sp
        v.airspeed,  # airspeed
        0,  # airspeed_sp
        v.groundspeed,  # groundspeed
        0,  # climb_rate
        v.gps_nsat,  # gps_nsat
        v.gps_fix_type,  # gps_fix_type
        v.battery_remaining,  # battery_remaining
        0,  # temperature
        0,  # temperature_air
        0,  # failsafe
        0,  # wp_num
        0)  # wp_distance


def high_latency2_out(v, b):
    b.mav.high_latency2_send(
        v.time_boot_ms,  # timestamp
        v.mav_type,  # type
        v.autopilot,  # autopilot
        v.custom_mode,  # custom_mode
        v.latitude,  # latitude
        v.longitude,  # longitude
        v.altitude_amsl,  # altitude
        1,  # target_altitude
        v.heading,  # heading
        1,  # target_heading
        1,  # target_distance
        v.throttle,  # throttle
        v.airspeed,  # airspeed
        1,  # airspeed_sp
        v.groundspeed,  # groundspeed
        v.wind_speed,
        v.wind_heading,  # wind_heading
        1,  # eph
        1,  # epv
        1,  # temperature_air
        1,  # climb_rate
        v.battery_remaining,  # battery
        0,  # wp_num
        0,  # failure_flags
        v.battery_voltage,  # custom0
        0,  # custom1
        0  # custom2
    )


def process_message(msg, v):
    msg_type = msg['mavpackettype']

    if msg_type == 'HEARTBEAT':
        v.custom_mode = int(msg['custom_mode'])
        # v.type = int(msg['type'])
        v.autopilot = int(msg['autopilot'])
    elif msg_type == 'BATTERY_STATUS':
        v.battery_remaining = abs(int(msg['battery_remaining']))
        v.battery_voltage = abs(int(msg['voltages'][0] * 0.001))
    elif msg_type == 'VFR_HUD':
        v.heading = int(msg['heading'] * 0.5)
        v.airspeed = int(msg['airspeed'])
        v.groundspeed = int(msg['groundspeed'])
        v.throttle = int(msg['throttle'])
    elif msg_type == 'GPS_RAW_INT':
        v.gps_nsat = int(msg['satellites_visible'])
        v.gps_fix_type = int(msg['fix_type'])
        v.latitude = int(msg['lat'])
        v.longitude = int(msg['lon'])
        v.altitude_amsl = int(msg['alt'] * 0.001)
        v.eph = int(msg['eph'])
        v.epv = int(msg['epv'])
    elif msg_type == 'ATTITUDE':
        v.roll = int(msg['roll'])
        v.pitch = int(msg['pitch'])
        v.time_boot_ms = int(msg['time_boot_ms'])
    elif msg_type == 'WIND':
        v.wind_heading = int(msg['direction'] * 0.5)
        v.wind_speed = int(msg['speed'])


def init_params(v):
    v.custom_mode = 0
    v.type = 11
    v.autopilot = 3
    v.battery_remaining = 0
    v.heading = 0
    v.airspeed = 0
    v.groundspeed = 0
    v.throttle = 0
    v.gps_nsat = 0
    v.gps_fix_type = 0
    v.latitude = 0
    v.longitude = 0
    v.altitude_amsl = 0
    v.eph = 0
    v.epv = 0
    v.roll = 0
    v.pitch = 0
    v.time_boot_ms = 0
    v.wind_heading = 0
    v.wind_speed = 0
    v.battery_voltage = 0


def send_buf(target, buf, force_mavlink1=False):
    target.file.write(buf)
    target.seq = (target.seq + 1) % 256
    target.total_packets_sent += 1
    target.total_bytes_sent += len(buf)


wait_conn(vehicle)
init_params(vehicle)

# Make messages to the base station look like they are from the vehicle
basestation.mav.srcSystem = vehicle.sysid
basestation.mav.srcComponent = 1

# Make messages to the vehicle look like they are from system 200
vehicle.mav.srcSystem = 200
vehicle.mav.srcComponent = 1

while True:
    try:
        vehicle_message = vehicle.recv_msg()
        basestation_message = basestation.recv_msg()

        if vehicle_message:
            # Update Vehicle State
            vehicle_message_dict = vehicle_message.to_dict()
            process_message(vehicle_message_dict, vehicle)
            if vehicle_message_dict['mavpackettype'] == 'HEARTBEAT':
                heartbeat_count += 1
                if heartbeat_count == refresh_interval:
                    heartbeat_count = 0
                    print("Vehicle: ", vehicle_message_dict)
                    try:
                        high_latency2_out(vehicle, basestation)
                    except Exception as e:
                        print(e)

        elif basestation_message:
            # Forward BaseStation Message to Vehicle
            print("BaseStation: ", basestation_message.to_dict())
            vehicle.mav.send(basestation_message)

        else:
            # No Message at this time
            # Wait a little
            time.sleep(0.1)

    except:
        pass
