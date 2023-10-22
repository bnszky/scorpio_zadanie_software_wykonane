#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int8

# If velocity is between velocity_goal+-ACC_RANGE control signal will be not changed
# You can change it to 0 and then algorithm will try to adjust exact target speed
ACC_RANGE = 4.0

# How often control signal changes 
FREQUENCY = 200

def get_curr_velocity(msg):
    global curr_velocity
    curr_velocity = msg.data
    
def get_velocity_goal(msg):
    global velocity_goal
    velocity_goal = msg.data

if __name__ == "__main__":
    curr_velocity = 0.0
    velocity_goal = 0.0

    rospy.init_node("virtual_dc_motor_controller")
    rospy.loginfo("Node has been started")

    rospy.Subscriber("/virtual_dc_motor_driver/get_velocity", Float32, get_curr_velocity)
    rospy.Subscriber("/virtual_dc_motor_controller/set_velocity_goal", Float32, get_velocity_goal)
    pub = rospy.Publisher("/virtual_dc_motor/set_cs", Int8, queue_size=10)

    control_signal = 0
    r = rospy.Rate(FREQUENCY)

    # In/decrease control signal to adjust to velocity goal
    # Control signal takes only 201 values so this algorithm works definitely fast
    # And changes speed smoothly 
    while not rospy.is_shutdown():
        if(curr_velocity < velocity_goal-ACC_RANGE):
            if control_signal < 100:
                control_signal+=1
            else:
                rospy.logwarn("Velocity goal is unreachable! There is no possibility to increase velocity")
        elif curr_velocity > velocity_goal+ACC_RANGE:
            if control_signal > -100:
                control_signal-=1
            else:
                rospy.logwarn("Velocity goal is unreachable! There is no possibility to increase velocity")
        pub.publish(control_signal)

        r.sleep()