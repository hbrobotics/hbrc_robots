#!/usr/bin/env python

"""
    wander.py - Go forward indefinitely but turn to avoid obstacles

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import roslib; roslib.load_manifest('scout_apps')
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from ros_arduino_msgs.msg import *
import random
import thread

class Wander():
    def __init__(self):
        rospy.init_node('wander', log_level=rospy.DEBUG)
        
        # Reserve a thread lock so the sensor subscribers don't lock up the publisher loop
        self.mutex = thread.allocate_lock()
        
        rospy.on_shutdown(self.shutdown)
        
        # How fast should we publish Twist messages?
        rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(rate)

        # Maximum linear and angular speeds
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.35)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.0)
        self.turn_speed = rospy.get_param("~turn_speed", 0.1)
        
        # Distance thresholds (in meters) for registering an obstacle
        self.obstacle_threshold_front = rospy.get_param("~obstacle_threshold_front", 0.6)
        self.sonar_obstacle_threshold_side = rospy.get_param("~sonar_obstacle_threshold_side", 0.4)
        self.ir_obstacle_threshold_side = rospy.get_param("~ir_obstacle_threshold_side", 0.3)
        
        # If the motor current exceeds this value (in amps), we are probably stuck on somthing
        self.current_threshold = rospy.get_param("~current_threshold", 1.5)
        
        # If an obstacle is within this distance (in meters), then run an escape routine
        self.escape_threshold = rospy.get_param("~escape_threshold", 0.14)
        
        # What is the minimum of the maximum ranges of all the sensors?
        self.min_max_range = 0.8 # Sharp GP2D12 IR sensor
        
        # Offsets for the IR sensors mounted under the robot
        self.ir_side_offset = 0.07
        self.ir_front_offset = 0.12
        
        # Initialize a few variables
        self.motor_current_left = 0
        self.motor_current_right = 0
        
        # Since motor currents can momentarily spike, we will measure over several samples
        self.current_counter = 0
        self.max_current_counter = 3
        
        # Flag to indicate we are dealing with an obstacle
        self.obstacle_detected = False
        
        # Which direction we are turning
        self.right_left = None
                
        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()
  
        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        # Subscribe to the sensor topics
        sensor_namespace = "/arduino/sensor/"
        rospy.Subscriber(sensor_namespace + 'ir_front_left', Range, self.ir_front_left_callback)
        rospy.Subscriber(sensor_namespace + 'ir_front_center', Range, self.ir_front_center_callback)
        rospy.Subscriber(sensor_namespace + 'ir_front_right', Range, self.ir_front_right_callback)
        rospy.Subscriber(sensor_namespace + 'sonar_front_left', Range, self.sonar_front_left_callback)
        rospy.Subscriber(sensor_namespace + 'sonar_front_center', Range, self.sonar_front_center_callback)
        rospy.Subscriber(sensor_namespace + 'sonar_front_right', Range, self.sonar_front_right_callback)
        
        # And the two current sensors
        rospy.Subscriber(sensor_namespace + 'motor_current_right', AnalogFloat, self.motor_current_right_callback)
        rospy.Subscriber(sensor_namespace + 'motor_current_left', AnalogFloat, self.motor_current_left_callback)
        
        # Wait for all subscribers to start receiving messages
        rospy.wait_for_message(sensor_namespace + 'ir_front_left', Range)
        rospy.wait_for_message(sensor_namespace + 'ir_front_center', Range)
        rospy.wait_for_message(sensor_namespace + 'ir_front_right', Range)
        rospy.wait_for_message(sensor_namespace + 'sonar_front_left', Range)
        rospy.wait_for_message(sensor_namespace + 'sonar_front_center', Range)
        rospy.wait_for_message(sensor_namespace + 'sonar_front_right', Range)
        rospy.wait_for_message(sensor_namespace + 'motor_current_right', AnalogFloat)
        rospy.wait_for_message(sensor_namespace + 'motor_current_left', AnalogFloat)

        rospy.loginfo("Ready to wander...")
                
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        while not rospy.is_shutdown():
            self.mutex.acquire()
            self.process_sensors()
            self.mutex.release()
            self.cmd_vel_pub.publish(self.cmd_vel)
            r.sleep()      
    
    def ir_front_left_callback(self, msg):
        self.mutex.acquire()
        self.ir_front_left = min(msg.max_range, max(msg.min_range, msg.range)) - self.ir_side_offset
        self.mutex.release()
        
    def ir_front_center_callback(self, msg):
        self.mutex.acquire()
        self.ir_front_center = min(msg.max_range, max(msg.min_range, msg.range)) - self.ir_front_offset
        self.mutex.release()
        
    def ir_front_right_callback(self, msg):
        self.mutex.acquire()
        self.ir_front_right = min(msg.max_range, max(msg.min_range, msg.range)) - self.ir_side_offset
        self.mutex.release()
        
    def sonar_front_left_callback(self, msg):
        self.mutex.acquire()
        self.sonar_front_left = min(msg.max_range, max(msg.min_range, msg.range))
        self.mutex.release()
        
    def sonar_front_center_callback(self, msg):
        self.mutex.acquire()
        self.sonar_front_center = min(msg.max_range, max(msg.min_range, msg.range))
        self.mutex.release()
        
    def sonar_front_right_callback(self, msg):
        self.mutex.acquire()
        self.sonar_front_right = min(msg.max_range, max(msg.min_range, msg.range))
        self.mutex.release()
        
    def motor_current_right_callback(self, msg):
        self.mutex.acquire()
        self.motor_current_right = msg.value
        self.mutex.release()
        
    def motor_current_left_callback(self, msg):
        self.mutex.acquire()
        self.motor_current_left = msg.value
        self.mutex.release()
        
    def sonar_front_ave_callback(self, msg):
        self.mutex.acquire()
        self.sonar_front_ave = msg.value
        self.mutex.release()
    
    def process_sensors(self):    
        # Check for excessive motor currents suggesting that we are stuck on something
        if self.motor_current_right > self.current_threshold or self.motor_current_left > self.current_threshold:
            self.current_counter += 1
            if self.current_counter > self.max_current_counter:
                rospy.loginfo("MOTOR CURRENT ESCAPE -- L: " + str(self.motor_current_left) + " R: " + str(self.motor_current_right))
                self.escape(self.motor_current_left - self.motor_current_right)
                self.current_counter = 0
        else:
            self.current_counter = 0
        
        # If there something really up close (inbetween the left/right IR beams) then escape
        if self.ir_front_left < self.escape_threshold or self.ir_front_right < self.escape_threshold:
            rospy.loginfo("IR SENSOR ESCAPE -- L: " + str(self.ir_front_left) + " R: " + str(self.ir_front_right))
            self.escape(self.ir_front_left - self.ir_front_right)

        # If something is within the threshold, stop and turn away
        distance_front = min(self.sonar_front_center, self.ir_front_center) - self.obstacle_threshold_front
        distance_left = min(self.ir_front_left - self.ir_obstacle_threshold_side, self.sonar_front_left - self.sonar_obstacle_threshold_side)
        distance_right = min(self.ir_front_right - self.ir_obstacle_threshold_side, self.sonar_front_right - self.sonar_obstacle_threshold_side)

        # Determine the direction to turn
        self.right_left = (distance_left - distance_right) #* random.uniform(0.8, 1.0)
        self.turn_radius = 0.5 * abs(self.right_left) / max(distance_left, distance_right)
        
        if distance_front < 0 or distance_left < 0 or distance_right < 0:
            if not self.obstacle_detected:
                # Set a flag so we keep turning in the same direction until clear
                self.obstacle_detected = True
                
            # First slow down. Slowing down this way reduces the abruptness of just stopping.
            self.cmd_vel.linear.x /= 1.5
            
            # Set the rotation speed to max.
            self.cmd_vel.angular.z = sign(self.right_left) * self.max_angular_speed # * random.uniform(0.8, 1.0)
        else:
            # Go straight but turn slightly if there are obstacles nearby
            self.obstacle_detected = False
            
            # Set the forward speed proportional to the amount of clearance around the robot
            ave_clearance = (self.sonar_front_right + self.sonar_front_center + self.sonar_front_left) / 3.0
            self.cmd_vel.linear.x = min(self.max_linear_speed, self.max_linear_speed * ave_clearance / 1.2)
            
            # Here we turn a bit away from distance obstacles
            self.cmd_vel.angular.z = 0 #sign(self.right_left) * self.turn_radius
    
    def escape(self, left_right):
        # Stop
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(self.cmd_vel)
        rospy.sleep(0.5)
        
        # Back up
        self.cmd_vel.linear.x = -self.max_linear_speed / 2        
        backup_time = 1.75
        n_intervals = 10
        backup_interval = backup_time / n_intervals
        for i in range(n_intervals):
            self.cmd_vel_pub.publish(self.cmd_vel)
            rospy.sleep(backup_interval)
        
        # Stop
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(self.cmd_vel)
        rospy.sleep(0.5)
        
        # Rotate
        if left_right < 0:
            self.cmd_vel.angular.z = -self.max_angular_speed
        else:
            self.cmd_vel.angular.z = self.max_angular_speed
            
        rotate_time = 2.0
        n_intervals = 10
        rotate_interval = rotate_time / n_intervals
        for i in range(n_intervals):
            self.cmd_vel_pub.publish(self.cmd_vel)
            rospy.sleep(rotate_interval)
        
    def shutdown(self):
        # When shutting down be sure to stop the robot!
        rospy.loginfo("Stopping the robot!")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        
def sign(x):
    if x >= 0:
        return 1
    else:
        return -1
 
if __name__=="__main__":
    try:
        Wander()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Wander node terminated.")