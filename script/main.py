#!/usr/bin/env python3

from asyncio import FastChildWatcher
import rospy
import utm
import os
import sys
import math
from novatel_gps_msgs.msg import NovatelPosition
from novatel_gps_msgs.msg import NovatelVelocity
from geometry_msgs.msg import PoseWithCovariance
from std_msgs.msg import Int16 
import pylab as plt
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Bool
import cv2
import pickle

def get_distance(init, now):
    return abs(math.dist(init, now))


class Track_Map_Generate:
    def __init__(self):
        self.sub_pos= rospy.Subscriber("/bestpos",NovatelPosition, self.gpsCB)
        self.subVelocity = rospy.Subscriber("/bestvel", NovatelVelocity,self.velCB)
        self.subSteer =rospy.Subscriber("/MSG_CON/Rx_Steer", Int16,self.steerCB)
        #self.sub_local = rospy.Subscriber("/Perception/Localization/LocalPose", PoseWithCovariance, self.localCB)

        self.path_true = rospy.Publisher("/Track_Path/Exist",Bool,queue_size=1)
        self.rotation_cnt = rospy.Publisher("/Track_Path/Count",Int16,queue_size=1)
        #
        # member variation
        #
        self.path_dir ="/home/sumin/catkin_ws/src/track_map_generate/path/path.txt"
        self.path_state = False

        self.utm_x =0
        self.utm_y =0

        self.init = True

        self.init_x =0
        self.init_y = 0

        self.prev_x =0
        self.prev_y = 0

        self.vel= 0
        self.steer =0 
        self.utm_list = []
        self.interval = 0.5
        self.now_distance = 0
        self.prev_distance  = 0
        self.distance = 0
        self.cnt = 0
        self.frame = np.zeros((500,700),np.uint8)
        self.rising  =False
        self.falling  = False
        self.re_rising = False 
        self.rotation = 0
        #
        # member function
        #
    
        self.path_exist()
        
        #self.utm_local_x =0
        #self.utm_local_y =0
        

    def gpsCB(self, data):
        msg = Bool()
        msg.data = self.path_state
        self.path_true.publish(msg)

        msg_cnt = Int16()
        msg_cnt.data = self.rotation
        self.rotation_cnt.publish(msg_cnt)

        self.utm_x, self.utm_y = utm.from_latlon(data.lat, data.lon)[:2]
        
        if self.init ==True:
            self.init_x, self.init_y = self.utm_x , self.utm_y 
            self.prev_x, self.prev_y = self.utm_x , self.utm_y 
            self.now_distance = get_distance((self.init_x,self.init_y),(self.utm_x, self.utm_y))
            self.prev_distance = 0 
            self.utm_list.append(f"{self.init_x} {self.init_y}\n")
            self.path_exist()
            self.init =False
            rospy.loginfo("Initialize")
        
        else :
            if self.now_distance >self.interval:
                self.rising = True
                self.re_rising =False
                
            if self.now_distance < self.interval and self.rising ==True:
                self.falling = True
              
            if self.now_distance >self.interval and self.rising == True and self.falling == True: 
                self.re_rising = True
                self.cnt+=1

            self.frame = cv2.line(self.frame, (0,398),(700,398),(255,255,255),1,cv2.LINE_AA )
            rospy.loginfo(f"rising : {self.rising}, falling: {self.falling}, re_rising: {self.re_rising}, count: {self.rotation}, distance: {self.now_distance}")
            rospy.loginfo(f"steer: {self.steer}")
            self.now_distance = get_distance((self.init_x,self.init_y),(self.utm_x, self.utm_y))
            if self.rising == True:
                self.distance = get_distance((self.utm_x, self.utm_y), (self.prev_x, self.prev_y) )
                
                if  self.distance >= self.interval:
                    self.frame = cv2.circle(self.frame, (self.cnt,400-10*round(self.now_distance)),2,(255,255,255),1,cv2.LINE_AA )
                    
                    self.cnt+=1
                    rospy.loginfo((self.utm_x, self.utm_y))
                    rospy.loginfo(f"distance: {self.now_distance}")
                    self.prev_distance = self.now_distance
                    self.prev_x, self.prev_y = self.utm_x , self.utm_y

                    self.utm_list.append(f"{self.utm_x} {self.utm_y}\n")
                    cv2.imshow("frame", self.frame)
                    cv2.waitKey(1)


                if  self.rising == True and self.falling == True and self.re_rising==False and self.path_state == False:
                    rospy.loginfo("Map Generate")
                    with open(self.path_dir, "a") as f:
                        f.writelines(self.utm_list)
                    self.path_state = True
                
                if  self.rising == True and self.falling == True and self.re_rising==True :
                    self.rotation+=1
                    msg.data = self.path_state
                    self.path_true.publish(msg)
                    self.rising = False
                    self.falling = False
                    self.re_rising = False
            
            

    def velCB(self,data):
        ver, hor = data.vertical_speed, data.horizontal_speed
        self.vel = math.sqrt(pow(ver,2)+pow(hor,2)) *3.6
        
    def steerCB(self,data):
        self.steer= data.data/71

    def path_exist(self):
        msg = Bool()
        if os.path.isfile(self.path_dir):
            os.remove(self.path_dir)
            msg.data = False
            self.path_true.publish(msg)
        else :
            msg.data = False
            self.path_true.publish(msg)
        

    

    #def localCB(self,data):
    #    self.utm_local_x, self.utm_local_y =  data.pose.position.x, data.pose.position.y
#
    #    print(math.sqrt(pow(self.utm_local_x - self.utm_x,2)+ pow(self.utm_local_y - self.utm_y,2)))
    
def main(args):
    
    rospy.init_node("track_map_generate", anonymous=True)
    tmg = Track_Map_Generate()
    
    try:
        
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")


if __name__ == "__main__":
    main(sys.argv)
