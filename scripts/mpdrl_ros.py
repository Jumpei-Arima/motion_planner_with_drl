#!/usr/bin/env python3

import os
import math

import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import tf

from pfrl_agent import pfrl_ppo_agent

class MotionPlannerWithDRL:
    def __init__(self):
        rospy.init_node('mpdrl', anonymous=True)

        # subscriber
        scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        goal_sub = rospy.Subscriber('/target', PoseStamped, self.target_callback)

        # publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.input_scan_pub = rospy.Publisher('/scan/input', LaserScan, queue_size=10)

        self.scan_data = None
        self.target_data = None

        self.MAX_DIS = 10.0
        self.LIDAR_SIZE = 36
        self.MAX_RANGE = 10.0

        model_path = os.path.join(os.path.dirname(__file__), "model", "ppo")
        self.agent = pfrl_ppo_agent(model_path)
        
    def scan_callback(self, data):
        scan = LaserScan()
        scan = data
        scan_ranges =list(scan.ranges)
        scan_ranges = self.rotate(scan_ranges, int(len(scan_ranges)/2))
        min_idx = int((-np.pi*0.5 - scan.angle_min) / scan.angle_increment)
        max_idx = len(scan.ranges) - int((scan.angle_max - np.pi*0.5) / scan.angle_increment)
        kernel_size = int((max_idx - min_idx) / self.LIDAR_SIZE)
        ranges = []
        for i,r in enumerate(scan_ranges):
            if not scan.range_min < r < scan.range_max:
                ranges.append(scan.range_max)
            else:
                ranges.append(r)
        ranges = np.array(ranges[min_idx:int(min_idx+kernel_size*self.LIDAR_SIZE)])
        ranges = ranges.reshape([-1, kernel_size])
        self.scan_data = np.amin(ranges, axis=1).tolist()
        input_scan = scan
        input_scan.angle_min = np.pi * 0.5
        input_scan.angle_max = -np.pi * 0.5
        input_scan.angle_increment = np.pi / self.LIDAR_SIZE
        ranges = []
        for r in self.scan_data:
            ranges.append(r)
        input_scan.ranges = ranges
        self.input_scan_pub.publish(input_scan)
    
    def target_callback(self, data):
        target = PoseStamped()
        target = data
        yaw = self.get_yaw(target.pose.orientation)
        x = target.pose.position.x
        y = target.pose.position.y
        dis = math.sqrt(x*x + y*y)
        dis = min(dis, self.MAX_DIS)
        self.target_data = [dis, math.sin(yaw), math.cos(yaw)]

    def process(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            print("=======================")
            print("motion planner with drl")
            action = [0.0, 0.0]
            if((self.target_data is not None) and \
                    (self.scan_data is not None)):
                state = self.scan_data + self.target_data
                print("===== target =====")
                print(self.target_data)
                action = self.agent.get_action(state)
                if self.target_data[0] <= 0.3:
                    action = [0.0, 0.0]
            else:
                print("target: ", self.target_data is not None)
                print("scan: ", self.scan_data is not None)
            cmd_vel = Twist()
            if(action[0]<0.0):
                action[0] = 0.0
            elif(action[0]>0.3):
                action[0] = 0.3
            # if(action[1]<-0.5):
            #     action[1] = -0.5
            # elif(action[1]>0.5):
            #     action[1] = 0.5
            cmd_vel.linear.x = action[0]
            cmd_vel.angular.z = action[1]
            print("===== action =====")
            print(action)
            self.cmd_vel_pub.publish(cmd_vel)
            r.sleep()

    def get_yaw(self, quaternion):
        e = tf.transformations.euler_from_quaternion(
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return e[2]
    
    def rotate(self, l, n):
        return l[n:] + l[:n]

if __name__ == '__main__':
    mpdrl = MotionPlannerWithDRL()
    try:
        mpdrl.process()
    except rospy.ROSInterruptException:
        pass
