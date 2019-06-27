#!/usr/bin/env python
import numpy as np
import tensorflow
from ppo import PPONet
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
import math
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

# NN_MODEL = os.path.join(os.path.dirname(os.path.abspath("__file__")),'models/ppo_model_ep_100000.ckpt')
NN_MODEL = ('/home/amsl/ros_catkin_ws/src/motion_planner_with_drl/scripts/models/ppo_model_ep_100000.ckpt')
print(NN_MODEL)

lidar = np.zeros(36)
goal = np.zeros(2)
target_yaw = 0.0
dis = 0.0
target_sub_ = False

def angle_nomalize(z):
    return np.arctan2(np.sin(z),np.cos(z))

def angle_diff(a,b):
    a = angle_nomalize(a)
    b = angle_nomalize(b)
    d1 = a -b
    d2 = 2.0 * np.pi - abs(d1)
    if d1 > 0.0:
        d2 *= -1.0
    if abs(d1) < abs(d2):
        return d1
    else:
        return d2

def get_yaw(q):
    quaternion = np.array((q.x, q.y, q.z, q.w))
    e = tf.transformations.euler_from_quaternion(quaternion)
    return e[2]

def callback_target(data):
    global target_yaw 
    global dis
    global target_sub_ 
    target_yaw = get_yaw(data.pose.orientation)
    dis = np.sqrt(data.pose.position.x*data.pose.position.x + data.pose.position.y*data.pose.position.y)
    target_sub_ = True 

def callback_laser(data):
    global lidar
    for i in range(len(lidar)):
        l = []
        for j in range(i*20,(i+1)*20-1):
            l.append(data.ranges[j])
        lidar[i] = np.amin(l)
        if lidar[i] == float('inf') or math.isnan(lidar[i]):
            lidar[i]  = 10.0
   
if __name__ == '__main__':
    sess = tensorflow.Session()
    brain = PPONet(sess,39,2,[np.array([0.,-1.]),np.array([1.,1.])],[512,512,512])
    saver = tensorflow.train.Saver()
    saver.restore(sess,NN_MODEL)

    rospy.init_node('motion_planner_with_drl')
    laser_sub = rospy.Subscriber('/local_map/scan',LaserScan, callback_laser)
    target_sub = rospy.Subscriber('/direction/relative',PoseStamped,callback_target)
    vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size = 100)

    rate = rospy.Rate(10)
    vel = Twist()

    while not rospy.is_shutdown():
        print("=======motion planner with DRL========")
        if target_sub_:
            theta = target_yaw

            s = np.zeros(39)
            min_lidar = 10.0
            for i in range(len(lidar)):
                if lidar[i] < min_lidar:
                    min_lidar = lidar[i]
                    min_index = i
                s[i] = lidar[i]
            s[36] = dis
            s[37] = np.sin(theta)
            s[38] = np.cos(theta)
            s = np.array([s])
            action = brain.predict_a(s).reshape(-1)
            vel.linear.x = action[0]*0.8
            vel.angular.z = action[1]*0.6
            print("action: v=%f[m/s], w=%f[rad/s]" % (action[0]*0.6,action[1]*0.6))
            vel_pub.publish(vel)
        rate.sleep()
