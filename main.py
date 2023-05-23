from pymavlink import mavutil
import time
import sys


class Drone():
    """Class of quadrocopter inizialise"""

    def __init__(self, url, mode):
        """Constructor"""
        self.url = url
        # self.mode = mode
        self.master = mavutil.mavlink_connection(url)
        self.master.wait_heartbeat()
        self.attitude = abs(self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict()['z'])

    def arming(self, status):
        """Arm/disarm"""
        if status == True:
            self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        else:
            self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

    def set_mode(self, mode):
        """set fly mode"""
        # Check if mode is available
        if mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            sys.exit(1)

        # Get mode ID
        mode_id = self.master.mode_mapping()[mode]

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()

        # Continue waiting if the acknowledged command is not `set_mode`
        if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            pass

        # Print the ACK result !
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)

    def takeoff(self, attitude):
        """takeoff"""
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, attitude)

    def go_to_point(self, x, y):
        """Flight to goal point for coords"""
        # print(abs(self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict()['x'] - x))
        while abs(self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict()['x'] - x) > 1.0 and abs(
                self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict()['y'] - y) > 1.0:
            self.master.mav.send(
                mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.master.target_system,
                                                                              self.master.target_component,
                                                                              mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                              int(0b110111111000), x, y,
                                                                              -10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))
            current_x = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict()['x']
            current_y = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True).to_dict()['y']
            # print(x, y)
            time.sleep(0.1)

    def manual_control(self, goal_attitude, linear_x=0, linear_y=0, linear_z=500):
        """manual control of drone"""
        self.master.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.master.target_system,
                                                                          self.master.target_component,
                                                                          mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                          int(0b110111000011), 10, 10,
                                                                          -goal_attitude, linear_x, linear_y, linear_z, 0, 0, 0, 1.57, 0.5))

    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        """ Set RC channel pwm value
        Args:
            channel_id (TYPE): Channel ID
            pwm (int, optional): Channel pwm value 1100-1900
        """
        if channel_id < 1 or channel_id > 18:
            print("Channel does not exist.")
            return

        # Mavlink 2 supports up to 18 channels:
        # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system,  # target_system
            self.master.target_component,  # target_component
            *rc_channel_values)  # RC channel list, in microseconds.


class Router(x, y, power):
    """Class of router emulated"""
    def __init__(self):
        self.x = x
        self.y = y
        self.power = power

    def transmit(self):
        

drone = Drone(url='udpin:localhost:14551')
drone.arming(True)
drone.set_mode('GUIDED')
goal_attitude = 5
drone.takeoff(goal_attitude)
time.sleep(5)
drone.go_to_point(10, 15)
drone.manual_control(goal_attitude=goal_attitude, linear_x=750)
time.sleep(5)
print('the end')
