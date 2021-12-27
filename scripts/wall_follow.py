#! /usr/bin/env python3
"""
    Author: Shankrith S
    Email: shankrith1618@gmail.com
"""

# import ros 
import rospy
from rospy.timer import sleep
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

import numpy as np
import math

class WallFollow:
    # Import parameters from ROS param server
    LASER_SCAN_TOPIC = rospy.get_param("fetch/laser_scan_topic","/base_scan")
    CMD_VEL_TOPIC = rospy.get_param("fetch/cmd_vel_topic", "/cmd_vel")
    WALL_SIDE = rospy.get_param("wall_follow/wall_side","LEFT")
    MAX_VELOCITY = rospy.get_param("velocity", 0.3)
    DESIRED_WALL_DIST = rospy.get_param("wall_dist", 0.5)
    LOOP_RATE = rospy.get_param("wall_follow/loop_rate", 50)

    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher(self.CMD_VEL_TOPIC, Twist, queue_size=10)
        self.laser_scan_sub = rospy.Subscriber(self.LASER_SCAN_TOPIC, LaserScan, self.laser_cb)
        self.cmd_timer = rospy.Timer(rospy.Duration(1.0/self.LOOP_RATE), self.control_cb)
        self.laser_scan = LaserScan()

        self.state = {"distance": 0.0, "deriv": 0.0}
        self.gains = np.array([8.0, 0.1, 0.5]) # PDI Gains
        self.error = np.array([0.0, 0.0, 0.0]) #error, d(error)/dt, int of error
        self.cmd_vel = Twist()

        self.dist_pub = rospy.Publisher("distances", Float32, queue_size=10)
        self.visual_scan_pub = rospy.Publisher("/visual_laser_scan", LaserScan, queue_size=1)

        self.debug = True
    
    def control_cb(self, event):
        if(self.debug):
            rospy.loginfo(self.state)
            self.dist_pub.publish(self.state)
        self.cal_cmd_vel()
        self.cmd_vel_pub.publish(self.cmd_vel)
    
    def cal_cmd_vel(self):
        """
        Generates a cmd_vel with PID controller
        """
        current_error = self.state["distance"] - self.DESIRED_WALL_DIST
        self.error[1] = current_error - self.error[0] #derivative of error
        self.error[0] = current_error
        self.error[2] += current_error
        
        side = None
        if(self.WALL_SIDE == "LEFT"):
            side = 1
        else:
            side = -1

        self.cmd_vel.angular.z = side * self.gains[0] * self.error[0]
        self.cmd_vel.angular.z += side * self.gains[1] * self.state["deriv"]
        self.cmd_vel.angular.z += side * self.gains[1] * self.error[1]

        # cmd_vel_msg.angular.z = max(min(direction*(kp*e+kd*diff_e) + 
        # kp_angle*(angle_min-((math.pi)/2)*direction), 2.5), -2.5)

        self.cmd_vel.linear.x = self.MAX_VELOCITY

    def laser_cb(self, laser_data):
        self.laser_scan = laser_data
        self.estimate_state()
    
    def estimate_state(self):
        """
        Given laser scan data find distance from wall for a given side
        """
        beta = np.radians(45) # range of cone of measurements to use for dist   
        ref_ang = np.radians(60) # angle from straight ahead on which cone is centered
        meas = np.array(self.laser_scan.ranges)
        angle_min = self.laser_scan.angle_min
        angle_max = self.laser_scan.angle_max
        angle_increment = self.laser_scan.angle_increment
        angle_ranges = np.linspace(angle_min, angle_max, len(meas))

        if self.WALL_SIDE == "LEFT":
            # right side
            start_i = int(len(meas)/2 - ((ref_ang + beta/2)/angle_increment))
            stop_i = int(start_i + beta/angle_increment)
            side_i = int(len(meas)/2-(ref_ang)/angle_increment)
        elif self.WALL_SIDE == "RIGHT":
            # left side
            start_i = int(len(meas)/2 + (ref_ang - beta/2)/angle_increment)
            stop_i = int(start_i + beta/angle_increment)
            side_i = int(len(meas)/2+(ref_ang)/angle_increment)

        dists = np.absolute(meas[start_i:stop_i] * np.sin(angle_ranges[start_i:stop_i]))

        # filter out points that are more than 1 standard deviation away from
        # the mean

        mean = np.mean(dists)
        sd = np.std(dists)

        filt = lambda x: x if mean-sd < x < mean + sd else mean

        filtered = list(map(filt, dists))

        fit_line = np.polyfit(angle_ranges[start_i:stop_i], filtered, 1)

        self.state["distance"] = np.polyval(fit_line, angle_ranges[side_i])

        # _ = plt.plot(angle_ranges[start_i:stop_i], filtered, np.poly1d(fit_line)(angle_ranges[start_i:stop_i]))
        # plt.show()

        # deriv > 1 for convex corner, < 1 for concave
        self.state["deriv"] = fit_line[0] # slope of line of best fit


        if self.debug:
            # publishes fake LaserScan to visualize range used to calculate
            # distance to wall
            scan_narrow_msg = LaserScan()
            scan_narrow_msg.header.seq += 1
            scan_narrow_msg.header.stamp = rospy.Time.now()
            scan_narrow_msg.header.frame_id = self.laser_scan.header.frame_id
            scan_narrow_msg.angle_min = self.laser_scan.angle_min
            scan_narrow_msg.angle_max = self.laser_scan.angle_max
            scan_narrow_msg.angle_increment = self.laser_scan.angle_increment
            scan_narrow_msg.time_increment = self.laser_scan.time_increment
            scan_narrow_msg.scan_time = rospy.Time.now().to_sec()
            scan_narrow_msg.range_min = self.laser_scan.range_min
            scan_narrow_msg.range_max = self.laser_scan.range_max
            scan_narrow_msg.intensities = self.laser_scan.intensities
            scan_narrow_ranges = [0] * len(self.laser_scan.ranges)
            for i in range(start_i, stop_i):
                scan_narrow_ranges[i] = meas[i]
            scan_narrow_msg.ranges = scan_narrow_ranges
            self.visual_scan_pub.publish(scan_narrow_msg)




if __name__ == '__main__':
    rospy.init_node('wall_follower')
    wall_follower_obj = WallFollow()
    rospy.spin()