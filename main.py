from pymavlink import mavutil

connect = mavutil.mavlink_connection('udpin:localhost:14551')

connect.wait_heartbeat()
print(f'Heartbeat from system {connect.target_system} {connect.target_component}')
while True:
    msg = connect.recv_match(type='ATTITUDE', blocking=True)
    print(msg)