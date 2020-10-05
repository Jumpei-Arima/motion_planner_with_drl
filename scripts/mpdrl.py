#!/usr/bin/env python3

import os

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan

from policy import Policy

class MotionPlannerWithDRL:
    def __init__(self):
        rospy.init_node('mpdrl', anonymous=True)

        # subscriber
        scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        goal_sub = rospy.Subscriber('/target', PoseStamped, self.target_callback)

        # publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        self.scan_data = None
        self.target_data = None

        self.MAX_DIS = 10.0
        self.KERNEL_SIZE = 20
        self.MAX_RANGE = 10.0

        model_path = os.path.join(os.path.dirname(__file__), "model", "pol.pkl")
        self.policy = Policy(model_path,0.0,1.0,1.0)
        
    def scan_callback(self, data):
        scan = LaserScan()
        scan = data
        self.scan_data = []
        for i in range(len(scan.ranges)/self.KERNEL_SIZE):
            min_range = scan.range_max
            for j in range(self.KERNEL_SIZE):
                r = scan.ranges[i*self.KERNEL_SIZE+j]
                min_range = min(min_range, r)
            min_range = min(min_range, self.MAX_RANGE)
            self.scan_data.append()
    
    def target_callback(self, data):
        target = PoseStamped()
        target = data
        yaw =self.get_yaw(target.pose.orientation)
        x = target.pose.position.x
        y = target.pose.position.y
        dis = math.sqrt(x*x + y*y)
        dis = max(dis, self.MAX_DIS)
        self.target_data = [dis, math.sin(yaw), math.cos(yaw)]

    def process(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            print("=======================")
            print("motion planner with drl")
            action = [0.0, 0.0]
            if((self.target_data is not None) and \
                    (self.scan_data is not None)):
                state = []
                state.append(self.scan_data)
                state.append(self.target_data)
                action = self.policy.get_action(state)
                if self.target_data[0] == 0.0:
                    action = [0.0, 0.0]
            else:
                print("target: ", self.target_data is not None)
                print("scan: ", self.scan_data is not None)
            cmd_vel = Twist()
            cmd_vel.linear.x = action[0]
            cmd_vel.angular.z = action[1]
            print(cmd_vel)
            self.cmd_vel_pub.publish(cmd_vel)
            r.sleep()

    def get_yaw(self, quaternion):
        e = tf.transformations.euler_from_quaternion(
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return e[2]

if __name__ == '__main__':
    mpdrl = MotionPlannerWithDRL()
    try:
        mpdrl.process()
    except rospy.ROSInterruptException:
        pass
