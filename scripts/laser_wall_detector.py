#! /usr/bin/env python3
"""
Author: Shankrith S
Email: shankrith1618@gmail.com
"""
#import ros essentials
import rospy
from sensor_msgs.msg import LaserScan

import numpy as np

class LaserWallDist:

    LASER_SCAN_TOPIC = rospy.get_param("fetch/laser_scan_topic", "/base_scan")

    def __init__(self):
        self.laser_scan_sub = rospy.Subscriber(self.LASER_SCAN_TOPIC, LaserScan, self.laser_cb)
        self.laser_scan = LaserScan()
        self.regions = ["FRONT", "LEFT", "RIGHT", "FLEFT", "FRIGHT", "BLEFT", "BRIGHT"]
        self.state = {}
        for i in self.regions:
            self.state[i] = np.zeros(2) # distance, derivative aka slope

    def laser_cb(self, laser_data):
        self.laser_scan = laser_data
        self.estimate_state()
    
    def estimate_state(self):
        
        #TODO: make code work for other 4 regions xD

        for i in self.regions:
            beta = 0
            ref_ang = 0 
            side = 1
            print(i)
            if i == "FRONT":
                beta = np.radians(45)
                ref_ang = np.radians(0)
                self.state[i] = self.calc_dist(beta, ref_ang, side) 

            elif i == "LEFT":
                beta = np.radians(45)
                ref_ang = np.radians(70)
                side = -1
                self.state[i] = self.calc_dist(beta, ref_ang, side) 

            elif i == "RIGHT":
                beta = np.radians(45)
                ref_ang = np.radians(70)
                side = 1
                self.state[i] = self.calc_dist(beta, ref_ang, side) 

            elif i == "FLEFT":
                beta = np.radians(30)
                ref_ang = 50
                side = -1
                # self.state[i] = self.calc_dist(beta, ref_ang, side) 

            # elif i == "FRIGHT":
            #     beta = np.radians(30)
            #     ref_ang = 50
            #     side = 1
            #     self.state[i] = self.calc_dist(beta, ref_ang, side) 

            else:
                pass

        for i in self.state.keys():
            print("Distance from ", i, ": ", self.state[i])

    def calc_dist(self, beta, ref_ang, side):

        print(beta, ref_ang, side)

        meas = np.array(self.laser_scan.ranges)
        angle_min = self.laser_scan.angle_min
        angle_max = self.laser_scan.angle_max
        angle_increment = self.laser_scan.angle_increment
        angle_ranges = np.linspace(angle_min, angle_max, len(meas))

        start_i = int(len(meas)/2 + side*((ref_ang + beta/2)/angle_increment))
        stop_i = int(start_i + beta/angle_increment)
        side_i = int(len(meas)/2 + side*(ref_ang)/angle_increment)

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
        

def main():
    rospy.init_node('reading_laser')
    laser_reader_obj = LaserWallDist()
    rospy.spin()

if __name__ == '__main__':
    main()