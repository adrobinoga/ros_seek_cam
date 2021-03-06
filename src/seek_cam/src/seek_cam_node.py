#!/usr/bin/env python

import rospy
import time
import cStringIO
import datetime 
import pyseek_amarin_api as seek_api
from genpy.rostime import Time
from sensor_msgs.msg import CompressedImage
from threading import Thread

class CameraHandler:

    def __init__(self):
        self.thermal_pic=None
        self.thermal_pic_seq=0
        self.thermal_pic_tstamp=Time(0,0)
        self.thermal_pic_msg = CompressedImage() # message to publish
        self.cam_api = seek_api.SeekAPI()
    
    def find_camera(self):
        """
        finds and opens a seek thermal camera
        """
       
        # get available camera
        #try:
        self.cam_api.find_cam()
        #except :# PySeekError:
        #    rospy.logerr("No camera found")
        #    return False
        
        rospy.loginfo("Found camera")
        return True
        
        
    def pub_thermalview(self):
        """
        publishes thermal images at thermalview
        """
        
        # publisher setup
        pub = rospy.Publisher('thermalview/compressed', CompressedImage, queue_size=10)
        
        rospy.loginfo("Beginning thermalview")
        
        img_msg = CompressedImage() # message to publish
        
        while not rospy.is_shutdown():

            # get image
            tmpimg = self.cam_api.get_image()
            tmpstr = cStringIO.StringIO()
            tmpimg.save(tmpstr, "PNG")
            self.thermal_pic =tmpstr.getvalue()
            
            if True: # is empty?
                # set timestamp for picture
                now = time.time()
                self.thermal_tstamp = Time(now)
                   
                # fill message fields
                img_msg.header.seq = self.thermal_pic_seq
                img_msg.header.stamp = self.thermal_pic_tstamp
                img_msg.header.frame_id = "thermal_cam"
                
                img_msg.format = 'png'
                img_msg.data = self.thermal_pic
                # end fill
                
                self.thermal_pic_seq += 1
                
                pub.publish(img_msg)
                                      
                # todo: set delay
    
    
def main():

    rospy.init_node('seek_camera')
    
    cam_hand = CameraHandler()
    while not cam_hand.find_camera():
        time.sleep(2)
        
    # start thermal image publisher
    thread_thermalview = Thread(target=cam_hand.pub_thermalview)
    thread_thermalview.start()
        
    while not rospy.is_shutdown():
        pass
        
            
if __name__ == '__main__':
    main()
