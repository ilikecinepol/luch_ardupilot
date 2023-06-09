import time

import pymurapi as mur
import math

auv = mur.mur_init()
course_motor1 = 2
course_motor2 = 1
depth_motor_1 = 3
depth_motor_2 = 0
stepper_motor = 4

last_error = 0
i_component = 0


def limiter(value, min=-100, max=100):
    return min if value < min else max if value > max else value


def motor_kyrs(power):
    if power < 30:
        power = power * 1.19
    elif power < 40:
        power = power * 1.08
    elif power < 50:
        power = power * 1.03
    elif power < 60:
        power = power * 1
    elif power < 70:
        power = power * 1
    else:
        power = power * 0.97
        
    return power
    
def motor_glub(power):
    if power < 30:
        power = power * 1.38 
    elif power < 40:
        power = power * 1.29
    elif power < 50:
        power = power * 1.22
    elif power < 60:
        power = power * 1.18
    elif power < 70:
        power = power * 1.15
    elif power < 80:
        power = power * 1.15
    elif power < 90:
        power = power * 1.15
    elif power < 100:
        power = power * 1.14
        
    return power    

def moving(linear_x=0, linear_y=0, linear_z=0, angular_x=0, angular_y=0, angular_z=0):
    global course_motor1, course_motor2, depth_motor1, depth_motor2, stepper_motor
    auv.set_motor_power(course_motor1, limiter(linear_y) + angular_z)
    auv.set_motor_power(course_motor2, limiter(motor_kyrs(linear_y)) - angular_z)
    auv.set_motor_power(depth_motor_1, limiter(linear_z))
    auv.set_motor_power(depth_motor_2, limiter(motor_glub(linear_z)))
    auv.set_motor_power(stepper_motor, limiter(linear_x))


def relay_controller(value, setpoint):
    power = 20 if value > setpoint else -20
    moving(l_z=power)


def pid_controller(value, setpoint, p=0, i=0, d=0, delta_error=0.05):
    global last_error, i_component
    eror = setpoint - value
    if abs(eror) > delta_error:
        i_component += i_component * eror
    else:
        i_component = 0
    p_component = eror * p
    d_component = d * (eror - last_error)
    i_component = i_component * i
    last_error = eror
    time.sleep(0.0005)
    return p_component + d_component + i_component


def to_360(angle):
    return angle if angle > 0.0 else angle + 360.0


def keep_angle(goal_angle, p=0.2, i=0.5, d=0.01):
    current_angle = to_360(auv.get_yaw())
    print('current_angle:', current_angle)
    goal_angle = to_360(goal_angle)
    print('goal_angle:', goal_angle)
    power = pid_controller(to_360(current_angle), to_360(goal_angle), p=p, i=i, d=d)
    return power


def keep_depth(goal_depth=0, p=-40, i=-5, d=0.1):
    current_depth = auv.get_depth()
    # print('current_depth', current_depth)
    power = pid_controller(current_depth, goal_depth, p=p, i=i, d=d)
    return power


def rad_to_deg(rad):
    return pi / 180 * rad


def go_to_goal_xy(goal_x, goal_y, x=160, y=120, k_x=-0.2, k_y=-0.2):
    power_x = (goal_x - x) * k_x
    power_y = (goal_y - y) * k_y
    # print(power_x)
    moving(linear_x=power_x, linear_y=power_y)


def go_to_goal(x_goal, y_goal, x=160, y=120, k_lin=0.3, k_ang=-0.2):
    yaw = auv.get_yaw()
    distance = abs(math.sqrt(((x_goal - x) ** 2) + ((y_goal - y) ** 2)))
    k_lin = -k_lin if y_goal - y > 0 else k_lin
    k_ang = -k_ang if x_goal - x > 0 else k_ang
    linear_speed = distance * k_lin
    if distance > 1:
        desired_angle_goal = 180 / math.pi * (math.atan2(y_goal - y, x_goal - x))
        angular_speed = (desired_angle_goal - to_360(yaw)) * k_ang
    else:
        angular_speed = 0
    return linear_speed, angular_speed



if __name__ == '__main__':

    while True:
        moving(linear_z = 20)

