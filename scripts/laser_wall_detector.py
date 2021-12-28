#! /usr/bin/env python3
# -*-coding:utf-8 -*-
'''
@File    :   laser_wall_detector.py
@Time    :   2021/12/28 11:47:12
@Author  :   Shankrith Chokkalingam S 
@Version :   1.0
@Email :   shankrith1618@gmail.com
@Desc    :   None
'''

#import ros essentials
import rospy
from sensor_msgs.msg import LaserScan

import numpy as np

class LaserWallDist:
    """
    Class that estimates state of the robot given the laser scan data

    """

    LASER_SCAN_TOPIC = rospy.get_param("fetch/laser_scan_topic", "/base_scan")

    def __init__(self):
        self.laser_scan_sub = rospy.Subscriber(self.LASER_SCAN_TOPIC, LaserScan, self.laser_cb)
        self.visual_scan_pub = rospy.Publisher("/visual_laser_scan", LaserScan, queue_size=1)

        self.laser_scan = LaserScan()
        self.regions = ["LEFT", "FLEFT", "FRONT", "FRIGHT", "RIGHT"]
        self.state = {}
        for i in self.regions:
            self.state[i] = np.zeros(2) # distance, derivative aka slope

        self.debug = "False"

    def laser_cb(self, laser_data):
        self.laser_scan = laser_data
        self.estimate_state()
    
    def estimate_state(self):
        
        # TODO: check why front dist is wrong
        meas = np.array(self.laser_scan.ranges)
        angle_min = self.laser_scan.angle_min
        angle_max = self.laser_scan.angle_max
        angle_increment = self.laser_scan.angle_increment
        angle_ranges = np.linspace(angle_min, angle_max, len(meas))

        for i in self.regions:
            beta = np.radians(45)
            if(i == "FRONT"):
                beta = np.radians(10)
                ref_ang = np.radians(0)
                start_i = int(len(meas)/2 - ((ref_ang + beta/2)/angle_increment))
                stop_i = int(start_i + beta/angle_increment)
                side_i = int(len(meas)/2)
                self.state[i] = self.calc_dist(meas, angle_ranges, start_i, stop_i, side_i)

                if(self.debug == "DEBUG_FRONT"):
                    self.visualize_scan_range(start_i,stop_i, meas, side_i)

            
            elif(i == "LEFT" or i == "FLEFT"):
                ref_ang = np.radians(85)
                start_i = int(len(meas)/2 - ((ref_ang + beta/2)/angle_increment))
                stop_i = int(start_i + beta/angle_increment)
                if(i == "LEFT"):
                    side_i = int(len(meas)/2-(np.radians(110))/angle_increment)
                    self.state[i] = self.calc_dist(meas, angle_ranges, start_i, stop_i, side_i)
                    
                    if(self.debug == "DEBUG_LEFT"):
                        self.visualize_scan_range(start_i,stop_i, meas, side_i)

                else:
                    side_i = int(len(meas)/2-(np.radians(60))/angle_increment)
                    self.state[i] = self.calc_dist(meas, angle_ranges, start_i, stop_i, side_i)

                    if(self.debug == "DEBUG_FLEFT"):
                        self.visualize_scan_range(start_i,stop_i, meas, side_i)


            elif(i == "RIGHT" or i == "FRIGHT"):
                ref_ang = np.radians(70)
                start_i = int(len(meas)/2 + ((ref_ang + beta/2)/angle_increment))
                stop_i = int(start_i + beta/angle_increment)

                if(i == "RIGHT"):
                    side_i = int(len(meas)/2+(np.radians(110))/angle_increment)
                    self.state[i] = self.calc_dist(meas, angle_ranges, start_i, stop_i, side_i)
                    if(self.debug == "DEBUG_RIGHT"):
                        self.visualize_scan_range(start_i,stop_i, meas, side_i)

                else:
                    side_i = int(len(meas)/2+(np.radians(60))/angle_increment)
                    self.state[i] = self.calc_dist(meas, angle_ranges, start_i, stop_i, side_i)
                    if(self.debug == "DEBUG_FRIGHT"):
                        self.visualize_scan_range(start_i,stop_i, meas, side_i)

        for i in self.state.keys():
            print("Distance from ", i, ": ", self.state[i])

    def calc_dist(self, meas, angle_ranges, start_i, stop_i, side_i):

        dists = np.absolute(meas[start_i:stop_i] * np.sin(angle_ranges[start_i:stop_i]))

        # filter out points that are more than 1 standard deviation away from
        # the mean

        mean = np.mean(dists)
        sd = np.std(dists)

        filt = lambda x: x if mean-sd < x < mean + sd else mean

        filtered = list(map(filt, dists))

        fit_line = np.polyfit(angle_ranges[start_i:stop_i], filtered, 1)

        distance = np.polyval(fit_line, angle_ranges[side_i])
        deriv = fit_line[0]

        return np.array([distance, deriv])

    def visualize_scan_range(self, start_i, stop_i, meas, side_i):
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
            if(side_i != None):
                scan_narrow_ranges[side_i] = meas[side_i]
            else:
                for i in range(start_i, stop_i):
                    scan_narrow_ranges[i] = meas[i]
            scan_narrow_msg.ranges = scan_narrow_ranges
            self.visual_scan_pub.publish(scan_narrow_msg)
        

def main():
    rospy.init_node('reading_laser')
    laser_reader_obj = LaserWallDist()
    rospy.spin()

if __name__ == '__main__':
    main()