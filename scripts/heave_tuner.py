#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Float64, Float32MultiArray

THRUSTER = 6
THRUST_TOPIC = "/rose_tvmc/thrust"
DEPTH_TOPIC = "/depth_data"
THRUST_K = 2.90 / 100
MASS = 18
BYOF = 0.1 * 9.8
DEPTH = 0
U = 0

rospy.init_node("heave_tuner")

depth_msg = Float64()
depth_msg.data = DEPTH

pub = rospy.Publisher("/depth_data", Float64, queue_size=1)

def subx(tx: Float32MultiArray):
    global U, DEPTH
    start = time.time_ns()
    
    thrust = tx.data[THRUSTER] * THRUST_K * 3 * 9.8
    
    print("\r", thrust, end="")
    f = BYOF - thrust
    
    a = f / MASS
    
    time.sleep(0.01)
    end = time.time_ns()
    
    t = (end - start) / 1e+9
    U += a * t

    ds = U * t + 1/2 * a * (t ** 2)
    
    DEPTH = DEPTH - ds
    
    depth_msg.data = DEPTH
    
    pub.publish(depth_msg)


sub = rospy.Subscriber("/rose_tvmc/thrust", Float32MultiArray, subx, queue_size=1)

pub.publish(depth_msg)

rospy.spin()

