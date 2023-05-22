from pymavlink import mavutil
import time

connect = mavutil.mavlink_connection('udpin:localhost:14551')

connect.wait_heartbeat()
print(f'Heartbeat from system {connect.target_system} {connect.target_component}')
# mode = 'GUIDED'
# # Check if mode is available
# if mode not in connect.mode_mapping():
#     print('Unknown mode : {}'.format(mode))
#     print('Try:', list(master.mode_mapping().keys()))
#     sys.exit(1)
#
# # Get mode ID
# mode_id = connect.mode_mapping()[mode]

connect.mav.command_long_send(connect.target_system, connect.target_component,
                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
msg = connect.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)


# connect.mav.send(
#     mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, connect.target_system, connect.target_component,
#                                                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#                                                                    int(0b110111111000), int(-35.3626794 * 10 ** 7),
#                                                                    int(149.1658018 * 10 ** 7),
#                                                                    10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))


while True:
    connect.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, connect.target_system,
                                                                      connect.target_component,
                                                                      mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                      int(0b110111011111), 10, 10,
                                                                      -10, 100, 0, 50, 0, 0, 0, 1.57, 0.5))

    msg = connect.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    print(msg)
