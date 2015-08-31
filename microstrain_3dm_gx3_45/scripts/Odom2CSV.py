#! /usr/bin/env python
import roslib; roslib.load_manifest('microstrain_3dm_gx3_45')

import rospy
from nav_msgs.msg import Odometry

from geodesy.utm import UTMPoint

class GPS2CSV(object):
    
    def __init__(self):
    
        rospy.Subscriber("/imu_3dm_node/nav/odom", Odometry, self.gpsCallback)
    
        self.csv = open('out.csv','a')
        self.csv.write('timestamp;lat;lon\n')
        
        self.cb_recv = False
        
        self.zone = 33
        self.band = "U"
        
        self.last_ts = rospy.Time(0)
    
        rospy.loginfo("Waiting for data...")
        
    def close(self):
        
        self.csv.close()
        
    def gpsCallback(self,msg):
        
        if self.cb_recv is False:
            
            self.cb_recv = True
            rospy.loginfo('Pose received.')
            
        if msg.header.stamp > (self.last_ts + rospy.Duration(1)):
        
            pt = UTMPoint(easting=msg.pose.pose.position.x,northing=msg.pose.pose.position.y,altitude=0,zone=self.zone,band=self.band)
            ll = pt.toMsg()
            
            self.csv.write(str(msg.header.stamp.to_nsec()) + ';' + str(ll.latitude) + ';' + str(ll.longitude) + '\n')
            
            self.last_ts = msg.header.stamp
    
    
if __name__ == '__main__':

        rospy.init_node('pose2csv')

        node = GPS2CSV()
        
        rospy.spin()
        
        node.close()
        
        rospy.loginfo('Finished.')
        
        