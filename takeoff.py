from pymavlink import mavutil

connect = mavutil.mavlink_connection('udpin:localhost:14551')

connect.wait_heartbeat()
print(f'Heartbeat from system {connect.target_system} {connect.target_component}')

connect.mav.command_long_send(connect.target_system, connect.target_component,
                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
msg = connect.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
connect.mav.command_long_send(connect.target_system, connect.target_component,
                              mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

msg = connect.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)