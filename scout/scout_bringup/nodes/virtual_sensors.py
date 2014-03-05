#!/usr/bin/env python

"""
  virtual_sensors.py
  
  Publish computed sensor values from raw sensor data
  
"""

import roslib; roslib.load_manifest('scout')
import rospy
from ros_arduino_msgs.msg import *
from sensor_msgs.msg import Range

class Topic():
    def __init__(self, name, type, field):
        self.name = name
        self.type = type
        self.field = field

class VirtualSensor(object):
    def __init__(self, name, topics, types, thresholds, frame_id="/base_link", OutputType=AnalogFloat):
        self.name = name
        self.topics = topics
        self.types = types
        self.thresholds = thresholds
        self.n_topics = len(topics)
        self.values = [0]*self.n_topics
        self.states = [0]*self.n_topics
        self.frame_id = frame_id
        
        self.subscribe()
        
        self.msg = OutputType()
        self.pub = rospy.Publisher("~sonar_front_ave", OutputType)
        
        # Compute the running average
        self.sample_size = 10
        self.data = list()
        
    def subscribe(self):
        for i in range(self.n_topics):
            rospy.Subscriber(self.topics[i], self.types[i], lambda msg, index=i: self.callback(msg, index))
        
        rospy.loginfo("Waiting for message topics...")
        for i in range(self.n_topics):
            rospy.wait_for_message(self.topics[i], self.types[i])
            
    def callback(self, msg, index):
        if self.types[index] == Range:
            self.values[index] = msg.range
        else:
            self.values[index] = msg.value
        
#        if self.values[topic] > self.thresholds[topic]:
#            self.states[topic] = 1
#        else:
#            self.states[topic] = 0

    def compute(self):
        # First average over the sensors
        ave = 0.0
        for i in range(self.n_topics):
            ave += self.values[i]
            
        ave /= self.n_topics
        
        # Now get the running average
        self.data.append(ave)
        if len(self.data) > self.sample_size:
            self.data.pop(0)
        
        ave = 0
        for i in range(len(self.data)):   
            ave += self.data[i]
        
        ave /= len(self.data)
            
        return ave
        
    def publish(self):
        # Add a timestamp and publish the message
        self.msg.header.frame_id = self.frame_id
        self.msg.header.stamp = rospy.Time.now()
        self.msg.value = self.compute()
        self.pub.publish(self.msg)

class VirtualSensors():
    def __init__(self):
        rospy.init_node('virtual_sensors', log_level=rospy.DEBUG)
        
        self.obstacle_threshold_front = rospy.get_param("~obstacle_threshold_front", 0.65)
        self.obstacle_threshold_side = rospy.get_param("~obstacle_threshold_side", 0.35)
        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate)
        
        name_space = "/arduino/sensor/"
        
        virtual_sonar = VirtualSensor("sonar", [name_space + 'sonar_front_center', name_space + 'sonar_front_right', name_space + 'sonar_front_left'], [Range, Range, Range], [0.65, 0.35, 0.35])

        rospy.loginfo("Publishing virtual sensor data")
        
        while not rospy.is_shutdown():
            virtual_sonar.publish()
            r.sleep()


#        
#    def sensor_callback(self, msg):
#        self.data.append(msg)
#        n_sensors = len(msg.value)
#
#        if len(self.data) > self.sample_size:
#            self.data.pop(0)
#            
#        n_samples = len(self.data)
#        
#        # Compute the running average
#        data_ave = SensorState()
#        data_ave.header = msg.header
#        data_ave.name = msg.name
#        
#        for i in range(n_sensors):
#            sum = 0
#            for j in range(n_samples):
#                sum += self.data[j].value[i]
#
#            data_ave.header = msg.header
#            data_ave.value.append(sum / n_samples)
        
        # Filter outliers by replacing them with the running average
#        current = n_samples -1
#        last = n_samples - 2
#        if len(self.data) > 1:
#            for i in range(n_sensors):
#                if abs(self.data[current].value[i] - self.data[last].value[i-1]) / min(self.data[current].value[i], self.data[last].value[i-1]) > self.delta_threshold:
#                    self.data[current].value[i] = data_ave.value[i]
        
#        self.pub_ave.publish(data_ave)
    
 
if __name__=="__main__":
    try:
        VirtualSensors()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Virtual sensors node terminated.")