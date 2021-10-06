#!/usr/bin/env python

import rospy
from math import *
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

class stampChangerNode():

    def stamp_callback(self, data): 
        self.scan.header.stamp=data.header.stamp #changing the stamp and the frame_id of the scan
        self.scan.header.frame_id="base_footprint"
        self.pub.publish(self.scan) #publishing changed scan

    def scan_callback(self, data): #storing scan data
        self.scan=data

    def __init__(self):
    #creating publishers and subscribers
        self.pub = rospy.Publisher('changed_scan',LaserScan,queue_size=1)
        self.scan = LaserScan()
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.Subscriber("naoqi_driver/laser", LaserScan, self.stamp_callback)
        	
        
    def run(self):
    	# Main while loop.
    	while not rospy.is_shutdown():
    	    rospy.sleep(1.0)#everything happens in callback functions

if __name__ == '__main__':
    # Initalize the node and name it.
    rospy.init_node('pyclass',anonymous=True)
    try:
        ne = stampChangerNode()
        ne.run()
    except rospy.ROSInterruptException: pass
