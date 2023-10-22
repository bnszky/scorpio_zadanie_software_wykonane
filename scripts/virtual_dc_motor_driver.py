#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Float32

MAX_POSITION = 4095

prev_pos = 0
prev_time = 0

# Get time and position difference and compute RPM
def comp_rpm(delta_pos, delta_time):
    vel = delta_pos/MAX_POSITION * 60/delta_time
    return vel

def get_pos(msg):
    curr_pos = msg.data
    global prev_pos
    global prev_time

    curr_time = rospy.get_time()
    delta_time = curr_time - prev_time
    delta_pos = curr_pos - prev_pos

    # if encounter anything similar to this problem: 4095->1 1-4095=-4094 => -4094+4095+1 = 2
    if abs(delta_pos) > MAX_POSITION/2:
        if(delta_pos < 0):
            delta_pos = delta_pos + MAX_POSITION + 1
        else:
            delta_pos = delta_pos - MAX_POSITION - 1

    pub.publish(comp_rpm(delta_pos, delta_time))

    prev_pos = curr_pos
    prev_time = curr_time


if __name__ == "__main__":
    rospy.init_node("virtual_dc_motor_driver")

    rospy.Subscriber("/virtual_dc_motor/get_position", UInt16, get_pos)
    pub = rospy.Publisher("/virtual_dc_motor_driver/get_velocity", Float32, queue_size=10)

    rospy.spin()