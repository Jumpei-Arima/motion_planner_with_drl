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

NN_MODEL = os.path.join(os.path.dirname(os.path.abspath("__file__")),'models/ppo_model_ep_100000.ckpt')
print(NN_MODEL)

lidar = np.zeros(36)
goal = np.zeros(2)
target_yaw = 0.0
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
    e = tf.transformations.euler_from_quaternion(q)
    return e[2]

def callback_target(data):
    global target_yaw 
    global target_sub_ 
    target_yaw = get_yaw(data.pose.orientation)
    target_sub_ = True 

def callback_laser(data):
    global lidar
    for i in range(len(lidar)):#-1,-1,-1):
        l = []
        for j in range(i*20,(i+1)*20-1):
            l.append(data.ranges[j])
        lidar[i] = np.amin(l)
        if lidar[i] == float('inf') or math.isnan(lidar[i]):
            lidar[i]  = 10.0
    lidar = lidar[::-1]
   
if __name__ == '__main__':
    sess = tensorflow.Session()
    brain = PPONet(sess,39,2,[np.array([0.,-1.]),np.array([1.,1.])],[512,512,512])
    saver = tensorflow.train.Saver()
    saver.restore(sess,NN_MODEL)

    rospy.init_node('motion_planner_with_drl')
    laser_sub = rospy.Subscriber('/local_map/scan',LaserScan, callback_laser)
    target_pub = rospy.Subscriber('/direction/relative',PoseStamped,callback_target)
    vel_pub = rospy.Publisher('/tinypower/command_velocity',Twist,queue_size = 100)

    # listener = tf.TransformListener()

    rate = rospy.Rate(10)
    vel = Twist()
    target_orientation = PoseStamped()

    while not rospy.is_shutdown():
        # try:
        #     (trans,rot) = listener.lookupTransform("localmap","base_link",rospy.Time(0.0))
        # except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     continue
        # yaw = get_yaw(rot)
        if target_sub_:
            theta = target_yaw
            print(theta)
            # target_orientation.header.frame_id = "localmap"
            target_orientation.header.frame_id = "base_link"
            target_orientation.header.stamp = rospy.Time.now() 
            target_orientation.pose.position.x = 0 
            target_orientation.pose.position.y = 0 
            quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
            target_orientation.pose.orientation.x = quaternion[0]
            target_orientation.pose.orientation.y = quaternion[1]
            target_orientation.pose.orientation.z = quaternion[2]
            target_orientation.pose.orientation.w = quaternion[3]
            target_pub.publish(target_orientation)

            s = np.zeros(39)
            min_lidar = 10.0
            for i in range(len(lidar)):
                if lidar[i] < min_lidar:
                    min_lidar = lidar[i]
                    min_index = i
                s[i] = lidar[i]
            s[36] = 5.0 
            s[37] = np.sin(theta)
            s[38] = np.cos(theta)
            #print(s[36])
            s = np.array([s])
            action = brain.predict_a(s).reshape(-1)
            vel.linear.x = action[0]
            vel.linear.y = 0.0
            vel.angular.z = action[1]
            if min_lidar < 0.6:
                vel.linear.x = 0.0
                a = 1
                if min_index > 18:
                    a = -1
                vel.angular.z = a*0.3
                print("aaaaaa")
            print(action[0],action[1])
            vel_pub.publish(vel)
        rate.sleep()
