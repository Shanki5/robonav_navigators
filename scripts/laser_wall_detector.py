#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from statistics import mean

def clbk_laser(msg):
    # 720/5 = 144
    regions = [ 
      mean(msg.ranges[0:143]),
      mean(msg.ranges[144:287]),
      mean(msg.ranges[288:431]),
      mean(msg.ranges[432:575]),
      mean(msg.ranges[576:713]),
     ]
    rospy.loginfo(regions)

def main():
    rospy.init_node('reading_laser')
    sub= rospy.Subscriber("/base_scan", LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()