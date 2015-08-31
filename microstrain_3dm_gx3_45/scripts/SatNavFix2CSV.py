#! /usr/bin/env python
import roslib; roslib.load_manifest('microstrain_3dm_gx3_45')

import rospy
from sensor_msgs.msg import NavSatFix

class GPS2CSV(object):
    
    def __init__(self):
    
        rospy.Subscriber("/imu_3dm_node/gps/fix", NavSatFix, self.gpsCallback)
    
        self.csv = open('out.csv','a')
        self.csv.write('timestamp;lat;lon;alt\n')
        
        self.cb_recv = False
    
        rospy.loginfo("Waiting for data...")
        
    def close(self):
        
        self.csv.close()
        
    def gpsCallback(self,msg):
        
        if self.cb_recv is False:
            
            self.cb_recv = True
            rospy.loginfo('GPS data received.')
        
        
        self.csv.write(str(msg.header.stamp.to_nsec()) + ';' + str(msg.latitude) + ';' + str(msg.longitude) + ';' + str(msg.altitude) + '\n')
    
    
if __name__ == '__main__':

        rospy.init_node('gps2csv')

        node = GPS2CSV()
        
        rospy.spin()
        
        node.close()
        
        rospy.loginfo('Finished.')
        
        