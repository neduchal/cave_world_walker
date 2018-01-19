#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan

class MovementClass:

    def __init__(self):
        #self.scan = None
        self.position = []
        self.currentGoal = 0
        self.goals = []
        #self.scan_subscriber = rospy.Subscriber("base_scan", LaserScan, scanCallback)
        self.position_subscriber = rospy.Subscriber("pose_from_map", PoseStamped, self.positionCallback)
        self.cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def scanCallback(self, data):
        self.scan = data

    def positionCallback(self, data):
        yaw = np.arctan2(2 * data.pose.orientation.w * data.pose.orientation.z, 1 - 2 * data.pose.orientation.z**2)
        self.position = [data.pose.position.x, data.pose.position.y, yaw]

    def cmdPublish(self, throttle, steering):
        cmd = Twist()

        cmd.linear.x = throttle
        cmd.angular.z = steering
        self.cmd_vel_publisher.publish(cmd)

    def getDistBetweenHeadingAndPoint(self):
        return np.sqrt((self.position[0] - self.goals[self.currentGoal][0])**2 + (self.position[1] - self.goals[self.currentGoal][1])**2)

    def getAngleBetweenHeadingAndPoint(self):
        angle = np.arctan2(self.goals[self.currentGoal][1] - self.position[1], self.goals[self.currentGoal][0] - self.position[0]) - self.position[2]
        if np.abs(angle) > 3.14:
            if angle > 0:
                angle = -(6.28 - angle)
            else:
                angle =  6.28 + angle
        return np.min([angle, 6.28 - np.abs(angle)])

    def run(self):
        rate = rospy.Rate(10) # 10hz
        throttle = 0
        steering = 0
        while not rospy.is_shutdown():
            if self.position == []:
                continue
            if self.currentGoal == len(self.goals):
                rospy.signal_shutdown("exit")
                break
            steering = self.getAngleBetweenHeadingAndPoint()
            if(self.getDistBetweenHeadingAndPoint() > 0.1):
                throttle = 0.3
            else:
                throttle = 0
                while not np.abs(self.position[2] - self.goals[self.currentGoal][2]) < 0.2:
                    angle =  self.goals[self.currentGoal][2] - self.position[2]

                    steering = angle
                    self.cmdPublish(throttle, steering)
                else: self.currentGoal += 1

            self.cmdPublish(throttle, steering)

if __name__ == "__main__":
    rospy.init_node('cave_movement', anonymous=True)

    goals = [[5.7,0, 1.57],[5.7,1.6, 0], [8,1.6, 1.57], [8,3, 3.14], [2,3, 1.57], [2,5.8, 0], [5,5.8, 0.78],
             [6,7, 0], [7, 7, -1.57], [8,6, 1.57], [8,8.5, 3.14], [2,8.5, -1.57],[2,7.5, -3.14],
             [-0.6, 7.5, -1.57], [-0.6, 4.5, 0], [1.5,4.5,-1.57], [1.5, 0, 3.14]]

    movement = MovementClass()
    movement.goals = goals
    print("Im here")
    movement.run()
