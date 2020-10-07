#!/usr/bin/env python3

import math
import yaml
import random

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
import tf
import tf2_ros
import tf2_geometry_msgs


class Navigator:
    def __init__(self):
        rospy.init_node('navigator_with_rviz', anonymous=True)

        self.WORLD_FRAME = 'map'
        self.ROBOT_FRAME = 'base_link'

        # param
        self.HZ = rospy.get_param("~HZ", 10)
        self.GOAL_TOLERANCE = rospy.get_param("~GOAL_TOLERANCE", 0.3)
        self.TIMEOUT = rospy.get_param("~TIMEOUT", 180)
        if rospy.get_param("~USE_MAP"):
            MAP_PATH = rospy.get_param("~MAP_PATH")
            self.use_map = True
            with open(MAP_PATH) as f:
                self.map_data = yaml.load(f)
            self.poses = []
            for node in self.map_data["NODE"]:
                self.poses.append([node["point"]["x"], node["point"]["y"]])
            first_goal = PoseStamped()
            first_goal.header.stamp = rospy.Time.now()
            first_goal.header.stamp = self.WORLD_FRAME
            first_goal.pose.position.x = self.poses[0][0]
            first_goal.pose.position.y = self.poses[0][1]
            self.goal = first_goal
        else:
            # subscriber
            target_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.target_callback)
            self.use_map = False
            self.goal = None

        # publisher
        self.local_goal_pub = rospy.Publisher('/target', PoseStamped, queue_size=10)

        self.finish = False

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.start_time = rospy.Time.now()

    def target_callback(self, data):
        self.goal = data
        print("next goal: ")
        print(self.goal)
        self.finish = False
        self.start_time = rospy.Time.now()

    def process(self):
        r = rospy.Rate(self.HZ)
        while not rospy.is_shutdown():
            try:
                robot2map = self.tfBuffer.lookup_transform(self.ROBOT_FRAME, self.WORLD_FRAME, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                continue
            if((self.goal is not None) and not self.finish):
                local_goal = tf2_geometry_msgs.do_transform_pose(self.goal, robot2map)
                yaw = self.angle_diff(math.atan2(local_goal.pose.position.y, local_goal.pose.position.x),
                                            self.yaw_from_quaternion(local_goal.pose.orientation)-np.pi)
                local_goal.pose.orientation = self.quaternion_from_euler(yaw)
            else:
                local_goal = PoseStamped()
            # check goal
            time = rospy.Time.now().to_sec() - self.start_time.to_sec()
            if not self.finish:
                if(math.sqrt(local_goal.pose.position.x**2 + local_goal.pose.position.y**2) < self.GOAL_TOLERANCE):
                    print("==== goal ====")
                    print("time: ", time)
                    self.finish = True
                elif time>self.TIMEOUT:
                    print("==== timeout ====")
                    print("time: ", time)
                    self.finish = True
            if self.finish and self.use_map:
                next_goal = PoseStamped()
                next_goal.header.stamp = rospy.Time.now()
                next_goal.header.frame_id = self.WORLD_FRAME
                next_goal_idx = random.randint(0,len(self.poses)-1)
                next_goal.pose.position.x = self.poses[next_goal_idx][0]
                next_goal.pose.position.y = self.poses[next_goal_idx][1]
                print("next goal: [%d] " % next_goal_idx)
                print(next_goal)
                self.goal = next_goal
                self.finish = False
                self.start_time = rospy.Time.now()
            local_goal.header.stamp = rospy.Time.now();
            local_goal.header.frame_id = self.ROBOT_FRAME;
            self.local_goal_pub.publish(local_goal)
            r.sleep()

    def yaw_from_quaternion(self, quaternion):
        e = tf.transformations.euler_from_quaternion(
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return e[2]

    def quaternion_from_euler(self, yaw):
        qx,qy,qz,qw  = tf.transformations.quaternion_from_euler(0,0,yaw)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def angle_normalize(self, z):
        return np.arctan2(np.sin(z), np.cos(z))

    def angle_diff(self, a, b):
        a = self.angle_normalize(a)
        b = self.angle_normalize(b)
        d1 = a-b
        d2 = 2.0 * np.pi - abs(d1)
        if d1 > 0.0:
            d2 *= -1.0
        if abs(d1) < abs(d2):
            return d1
        else:
            return d2


if __name__ == '__main__':
    navigator = Navigator()
    try:
        navigator.process()
    except rospy.ROSInterruptException:
        pass
