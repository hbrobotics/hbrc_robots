#!/usr/bin/env python

"""
  filter_sensor_data.py
  
  Filter sensor data on the /arduino/sensor_state topic to compute running averages and remove outliers
  
"""

import roslib; roslib.load_manifest('scout')
import rospy
from element.msg import *
import os

class FilterSensorData():
    def __init__(self):
        rospy.init_node('filter_sensor_data', log_level=rospy.DEBUG)
                
        self.delta_threshold = rospy.get_param("~delta_threshold", 2.0)
        
        self.data = list()
        self.sample_size = 20
        
        # Publisher for the filtered value
        self.pub_ave = rospy.Publisher("/arduino/sensor_state_ave", SensorState)
        
        # Subscribe to the /element/sensors topic
        rospy.Subscriber('/arduino/sensor_state', SensorState, self.sensor_callback)
        
        rospy.loginfo("Ready to filter sensor data")

        
    def sensor_callback(self, msg):
        self.data.append(msg)
        n_sensors = len(msg.value)

        if len(self.data) > self.sample_size:
            self.data.pop(0)
            
        n_samples = len(self.data)
        
        # Compute the running average
        data_ave = SensorState()
        data_ave.header = msg.header
        data_ave.name = msg.name
        
        for i in range(n_sensors):
            sum = 0
            for j in range(n_samples):
                sum += self.data[j].value[i]

            data_ave.header = msg.header
            data_ave.value.append(sum / n_samples)
        
        # Filter outliers by replacing them with the running average
#        current = n_samples -1
#        last = n_samples - 2
#        if len(self.data) > 1:
#            for i in range(n_sensors):
#                if abs(self.data[current].value[i] - self.data[last].value[i-1]) / min(self.data[current].value[i], self.data[last].value[i-1]) > self.delta_threshold:
#                    self.data[current].value[i] = data_ave.value[i]
        
        self.pub_ave.publish(data_ave)
    
 
if __name__=="__main__":
    try:
        FilterSensorData()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Filter node terminated.")