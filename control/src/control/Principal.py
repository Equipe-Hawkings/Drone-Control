#!/usr/bin/env python3
from __future__ import division

import unittest
import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, ParamValue, State, WaypointList, PositionTarget
from mavros_msgs import srv
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode, SetModeRequest, WaypointClear, WaypointPush
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import BatteryState, NavSatFix, Imu
from pymavlink import mavutil
import numpy as np
import time

MAX_TIME_DESARM = 10

class Principal:
    
    #Construtor
    def __init__(self):


        #########Configuração Inicial##########

        #Parameters

        self.current_mode = State()
        

        #Messages

        self.rate = rospy.Rate(60)
        self.setpoint_local_msg = PoseStamped()

        #Services

        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        service_timeout = 30
        rospy.loginfo("Executando - wait_for_services")
        try:
            rospy.wait_for_service('/mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('/mavros/set_mode', service_timeout)
            rospy.loginfo("Services estão ok!")
        except rospy.ROSException:
            rospy.logerr("Falha em conectar às Services")

        #Publishers

        self.setpoint_local_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        #Subscribers

        self.setpoint_local_sub = rospy.Subscriber('/mavros/setpoint_position/local', PoseStamped, self.setpoint_local_callback)
        self.current_state = rospy.Subscriber('/mavros/state', State, self.current_state_callback)

        
    #########Callbacks##########

    def setpoint_local_callback(self, local):
        self.setpoint_local_msg.pose.position.x = local.pose.position.x
        self.setpoint_local_msg.pose.position.y = local.pose.position.y
        self.setpoint_local_msg.pose.position.z = local.pose.position.z
        
    def current_state_callback(self, state):
        self.current_mode = state

    #########Ações de movimento e controle##########

    #Armar

    def arm(self, arm):
        rospy.loginfo("Preparando para armar o drone")
        result = self.arm_srv(arm)
        if result == True:
            rospy.loginfo("Drone armado!")
        else:
            rospy.loginfo("Falha ao armar o drone!")
    
    #Setar modo de voo
        
    def set_mode(self, mode):
        rospy.loginfo("Alterando modo de voo")
        self.set_mode_srv(0,mode)
        if self.current_mode.mode == mode:
            rospy.loginfo("Modo de voo atual: ", self.current_mode.mode)
        else:
            rospy.loginfo("Falha ao alterar o modo de voo!")

    #Setar local setpoint

    def setpoint_position_local(self, x,y,z):
        self.setpoint_local_msg.pose.position.x = x
        self.setpoint_local_msg.pose.position.y = y
        self.setpoint_local_msg.pose.position.z = z
        self.setpoint_local_msg.header.stamp = rospy.Time.now()
        self.setpoint_local_pub.publish(self.setpoint_local_msg)
        self.rate.sleep()
