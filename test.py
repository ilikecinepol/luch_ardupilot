from pymavlink import mavutil
import time

connect = mavutil.mavlink_connection('udpin:localhost:14551')

connect.wait_heartbeat()
print(f'Heartbeat from system {connect.target_system} {connect.target_component}')
mode = 'GUIDED'

# Get mode ID
mode_id = connect.mode_mapping()[mode]

connect.mav.command_long_send(connect.target_system, connect.target_component,
                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
msg = connect.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)


def go_to_local(x, y):
    # print(abs(connect.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict()['x'] - x))
    while abs(connect.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict()['x'] - x) > 1.0 and abs(
            connect.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict()['y'] - y) > 1.0:
        connect.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, connect.target_system,
                                                                          connect.target_component,
                                                                          mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                          int(0b110111111000), x, y,
                                                                          -10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))
        current_x = connect.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict()['x']
        current_y = connect.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict()['y']
        # print(x, y)
        time.sleep(0.1)


while abs(connect.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict()['z']) < 1.0:
    connect.mav.command_long_send(connect.target_system, connect.target_component,
                                  mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
    val = abs(connect.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict()['z'])
    print(val)

print('takeoff complete')


go_to_local(10, 10)
go_to_local(50, 10)
go_to_local(10, 10)
go_to_local(60, 0)
go_to_local(60, -50)

connect.mav.send(
    mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, connect.target_system, connect.target_component,
                                                                   mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                                   int(0b110111111000), int(-35.3621581 * 10 ** 7),
                                                                   int(149.1659052 * 10 ** 7),
                                                                   10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))