import sys
import time
from utilities import Logger

from rclpy.time import Time

from utilities import *
from rclpy.node import Node
from geometry_msgs.msg import Twist


from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters



rawSensors=0; kalmanFilter=1

odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)


class localization(Node):
    
    def __init__(self, type, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","odom_x","odom_y","odom_v","odom_w","stamp"]):

        super().__init__("localizer")
        
        
        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.pose=None
        
        if type==rawSensors:
            self.initRawSensors();
        elif type==kalmanFilter:
            self.initKalmanfilter()
            self.kalmanInitialized = False
        else:
            print("We don't have this type for localization", sys.stderr)
            return            
    
        self.timelast=time.time()
    
    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)

    def initKalmanfilter(self):
        
        self.odom_sub=message_filters.Subscriber(self, odom, "/odom", qos_profile=odom_qos)
        self.imu_sub=message_filters.Subscriber(self, Imu, "/imu", qos_profile=odom_qos)
        
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        
        time_syncher.registerCallback(self.fusion_callback)
        
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):

        if not self.kalmanInitialized:
            x=np.array([odom_msg.pose.pose.position.x,
                        odom_msg.pose.pose.position.y,
                        euler_from_quaternion(odom_msg.pose.pose.orientation),
                        0,
                        0,
                        0])        
            
            # TODO PART 5 Put the Q and R matrices
            # that you conclude from lab Three
            # values from lab3/localization.py
            Q=0.25 * np.eye(6)   # process noise covariance (6 states: x, y, th, w, v, vdot)
            R=0.5 * np.eye(4)    # measurement noise covariance (4 meas: v, w, ax, ay)
            P=0.01 * np.eye(6)   # initial state covariance
                        
            self.kf=kalman_filter(P,Q,R, x)
            
            self.kalmanInitialized = True

        
        dt = time.time() - self.timelast

        self.timelast=time.time()


        z=np.array([odom_msg.twist.twist.linear.x,
                    odom_msg.twist.twist.angular.z,
                    imu_msg.linear_acceleration.x,
                    imu_msg.linear_acceleration.y])

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        v = odom_msg.twist.twist.linear.x
        w = odom_msg.twist.twist.angular.z
        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y
        z = np.array([v,w,ax,ay])
        
        self.kf.predict(dt)
        self.kf.update(z)
        
        xhat=self.kf.get_states()
        kf_x, kf_y, kf_th, kf_w, kf_v, kf_vdot = xhat
        stamp = Time.from_msg(odom_msg.header.stamp).nanoseconds
        
        self.pose=np.array([xhat[0],
                            xhat[1],
                            normalize_angle(xhat[2]),
                            odom_msg.header.stamp])


        values_list = [ax, ay, kf_vdot, kf_v * kf_w, kf_v, kf_w, kf_x, kf_y, x, y, v, w, stamp]
        self.loc_logger.log_values(values_list)
        
    def odom_callback(self, pose_msg):
        
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]
        
        #self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2], Time.from_msg(self.pose[3]).nanoseconds])

        
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    
    spin(LOCALIZER)
