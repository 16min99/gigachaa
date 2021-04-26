#!/usr/bin/env python
#-*- coding:utf-8 -*-
from __future__ import division, print_function #파이썬3문법을 2에서도 쓸수있게해줌
import rospy
import math
import tf
from std_msgs.msg import Float32
from std_msgs.msg import Time
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose
from sensor_msgs.msg import MagneticField
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from ublox_msgs.msg import NavPVT
import matplotlib.pyplot as plt
import numpy as np
from numpy.random import randn
# from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
# from filterpy.common import Q_discrete_white_noise
# from filterpy.stats import plot_covariance_ellipse
import pymap3d as pm
# import pandas as pd
import time as t

import threading

# define
magnet_OFF = [0, 0, 0]
PI = 3.141592
#msg = Odometry()

# ENU base coordinates
# 송도
base_lat = 37.448611
base_lon = 126.654965 
base_alt = 15.4



class Localization():
    def __init__(self):
        rospy.init_node('Position_Node', anonymous=False)
        self.pub = rospy.Publisher('/pose', Odometry, queue_size = 1)

        self.msg = Odometry()

        self.e_gps = 0
        self.n_gps = 0
        self.u_gps = 0
        self.e_gps_pre = 0
        self.n_gps_pre = 0



        # self.e_gps_pre = 0
        # self.n_gps_pre = 0
        # self.u_gps_pre = 0

        self.e_final = 0
        self.n_final = 0
        self.u_final = 0

        self.e_final_pre = 0
        self.n_final_pre = 0

        self.status = 0

        # self.dr_e = 0
        # self.dr_n = 0

        self.yaw_gps = 0
        self.yaw_imu = 0
        self.yaw_final = 0

        self.a = 0
        self.b = 0
        self.c = 0
        self.d = 0

        self.encoder_flag = 0
        self.dis_gps_flag = 0
        self.dis_DR_flag = 0
        self.dis_gps_enc = 0
        self.dis_DR_enc = 0

        ######################g#####
        self.add_old = 0
        self.add_new = 0
        self.displacement = 0
        self.add_displacement = 0

        self.gps_flag = 0
        self.e_DR = 0
        self.n_DR = 0
        self.position_flag = 0
        self.offset = 0

        self.t_Time_call = 0
        self.t_GPS_call = 0
        self.t_IMU_call = 0

        self.heading = 0
        self.gps_out = 0
        self.imu_out = 0
        #############################

    def gps_check(self,dis):
        gps_displacement = ((self.e_gps_pre - self.e_gps)**2 + (self.n_gps_pre - self.n_gps)**2)**(1/2)
        if ((dis*1.2)>=gps_displacement) :
            self.position_flag = 0
            #print("ok")
        else :
            self.position_flag = 1
            # print("e_final",self.e_final)
            # print("e_gps",self.e_gps)
            # print("n_final",self.n_final)
            # print("n_gps",self.n_gps)
            # print("gps:",gps_displacement)
            # print("dis:",self.displacement)



    def msg_write(self,msg):
        self.msg.pose.pose.position.x = float(self.e_final)
        self.msg.pose.pose.position.y = float(self.n_final)
        self.msg.twist.twist.angular.z = self.yaw_final
        self.e_final_pre = self.e_final
        self.n_final_pre = self.n_final

    def GPS_call(self,data):
        self.t_GPS_call = t.time()



        lon = float(data.longitude) #x
        lat = float(data.latitude) #y
        alt = float(data.altitude) #z

        self.status = data.status.status

        self.e_gps,self.n_gps,self.u_gps = pm.geodetic2enu(lat,lon,alt,base_lat,base_lon,base_alt)
        
        if self.dis_gps_flag == 0:
            self.a = self.add_displacement
            self.b = self.add_displacement
            self.dis_gps_flag = 1
        else:
            self.a = self.b
            self.b = self.add_displacement
            self.dis_gps_enc = self.b-self.a

        self.gps_check(self.dis_gps_enc)
        self.e_gps_pre = self.e_gps
        self.n_gps_pre = self.n_gps


        

               
        



    def GPS_Heading(self, data):
        self.yaw_gps = (450-(data.heading * 10**(-5)))%360       
        
    def IMU_call(self,data):
##################################################################3
        self.t_IMU_call = t.time()
#####################################################################        
        quaternion = (data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w) 
        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.yaw_imu = (euler[2] * 180 / PI + 90) % 360


    def Get_Dis(self, data):
        if self.encoder_flag == 0:
            self.add_old = data.data
            self.add_new = data.data
            self.encoder_flag = 1
        else:
            self.add_old = self.add_new
            self.add_new = data.data
            if (self.add_new - self.add_old) > 1000:
                self.Get_Dis() ##########이거 이렇게 하는거 맞음?


        if self.add_new - self.add_old >= 0 :
            self.add_displacement = (self.add_new - self.add_old)*1.6564/100
            #print("dis : ", self.displacement)
        else:
            self.add_displacement = (self.add_new + (256**4 - self.add_old))*1.6564/100

#########################################################################3
    # if self.gps_flag == 0:
    #             self.e_DR = self.e_final
    #             self.n_DR = self.n_final
    #             self.gps_flag = 1

    

    def DR(self, e, n):
        if self.dis_DR_flag == 0:
            self.c = self.add_displacement
            self.d = self.add_displacement
            self.dis_DR_flag = 1
        else:
            self.c = self.d
            self.d = self.add_displacement
            self.dis_DR_enc = self.d-self.c

        e += self.dis_DR_enc*math.cos(self.yaw_final*3.141592/180)
        n += self.dis_DR_enc*math.sin(self.yaw_final*3.141592/180)
        return e,n

##########################################################################

##########################################################################
    def decide_heading(self):
        if self.imu_out == 0:
            if self.gps_out == 0:
                if self.position_flag == 0: #gps o,imu o & 신뢰도 o
                    self.offset = self.yaw_gps - self.yaw_imu 
                    self.yaw_final = self.yaw_imu + self.offset

                else: #gps o,imu o & 신뢰도 x
                    self.yaw_final = self.yaw_imu + self.offset
            else:
                self.yaw_final = self.yaw_imu + self.offset
        
        else:
            if self.gps_out == 0:
                self.yaw_final = self.yaw_gps
            else:
                print("LANE DETECTION")



    def decide_position(self):
        if self.gps_out == 0:
            if self.position_flag == 0 or self.displacement < 0.1 :
                self.e_DR = self.e_final
                self.n_DR = self.n_final
                

                if self.status == 2:
                    self.e_final = self.e_gps
                    self.n_final = self.n_gps
                    print("GPSing")
                    #print("GPS :",self.e_gps,self.n_gps)
                    #print("DR :",self.e_DR,self.n_DR)
                
                else:
                    self.e_final = self.e_kalman
                    self.e_final = self.n_kalman
                    print("GPS+IMU")

            else: 
                self.e_DR, self.n_DR = self.DR(self.e_DR,self.n_DR)
                self.e_final = self.e_DR
                self.n_final = self.n_DR
                print("DR")
                
        else:
            self.e_DR, self.n_DR = self.DR(self.e_DR, self.n_DR)
            self.e_final = self.e_DR
            self.n_final = self.n_DR
            print("DR")
            



#############################################################################
        #칼만필터를 위해 가속도값을 메시지로 담는것
        # self.msg.pose.pose.position.z = self.msg.twist.twist.angular.z
   
        # orientation = tf.transformations.quatee_final_prernion_from_euler(self.msg.twist.twist.angular.y*PI/180, self.msg.twist.twist.angular.x*PI/180, self.msg.pose.pose.position.z*PI/180)

        # self.msg.pose.pose.orientation.y = orientation[0]
        # self.msg.pose.pose.orientation.x = orientation[1]
        # self.msg.pose.pose.orientation.z = orientation[2]
        # self.msg.pose.pose.orientation.w = orientation[3]

    def Time_call(self,time):
        self.t_Time_call = t.time()

        if self.t_Time_call - self.t_GPS_call > 1:
            self.gps_out = 1
        else:
            self.gps_out = 0

        if self.t_Time_call - self.t_IMU_call > 0.5:
            self.imu_out = 1
        else:
            self.imu_out = 0  
        self.decide_heading()
        self.decide_position()
        self.msg_write(self.msg)
        self.pub.publish(self.msg)   
        #print (self.msg.twist.twist.angular.z)





loc = Localization()
rospy.Subscriber('/Displacement', Float32 , loc.Get_Dis)
rospy.Subscriber('/gps_data/fix',NavSatFix,loc.GPS_call)
rospy.Subscriber('/gps_data/navpvt',NavPVT, loc.GPS_Heading)
rospy.Subscriber('/imu/data',Imu,loc.IMU_call)
rospy.Subscriber("/timer",Time, loc.Time_call)
rate = rospy.Rate(500)
rospy.spin()

