#!/usr/bin/env python3

import rospy
import message_filters
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from slam.msg import incorporated_sensor_data
import math

# front_measurements: point_cloud2
# rear_measurements: point_cloud2
# odom_measurements: 
class sensor_incorporation:
    def __init__(self):
        self.front_sub = message_filters.Subscriber('/robot/front_laser/scan', LaserScan)
        self.back_sub = message_filters.Subscriber('/robot/rear_laser/scan', LaserScan)
        self.odom_sub = message_filters.Subscriber('/robot/robotnik_base_control/odom', Odometry)
        self.sensor_topic = rospy.Publisher('/sensors_topic', incorporated_sensor_data, queue_size=1)
        self.timesynced = message_filters.ApproximateTimeSynchronizer([self.front_sub, self.back_sub, self.odom_sub], queue_size=10, slop=0.002)
        # self.timesynced = message_filters.TimeSynchronizer([self.front_sub, self.back_sub, self.odom_sub], queue_size=10)
        self.timesynced.registerCallback(self.merge)
        self.merged = incorporated_sensor_data
        self.angles = {}

    def merge(self, front_measurements, rear_measurements, odom_measurements):
        self.angles = {}
        merged = incorporated_sensor_data()
        merged.frame_id = 'robot_map'
        merged.header = odom_measurements.header
        
        merged.angle_min = front_measurements.angle_min
        merged.angle_max = rear_measurements.angle_max
        merged.angle_increment = 2*front_measurements.angle_increment
        merged.time_increment = front_measurements.time_increment
        merged.scan_time = front_measurements.scan_time
        merged.range_min = front_measurements.range_min
        merged.range_max = front_measurements.range_max
        
        
        merged.ranges[0:270] = front_measurements.ranges[0:540:2]
        merged.ranges[270:360] = rear_measurements.ranges[180:360:2]
        
        # # Front measurements:
        # for index in range(len(front_measurements.ranges)):
        #     angle = front_measurements.angle_min + index*front_measurements.angle_increment
        #     if front_measurements.ranges[index] > front_measurements.range_min and front_measurements.ranges[index] < front_measurements.range_max:
        #         self.angles[angle] = front_measurements.ranges[index]
        #     elif front_measurements.ranges[index] <front_measurements.range_min:
        #         self.angles[angle] = front_measurements.range_min
        #     else:
        #         self.angles[angle] = front_measurements.range_max

        #     # self.angles[angle]= front_measurements.ranges[index]

        # # Rear measurements:
        # for index in range(len(rear_measurements.ranges)):
        #     angle = rear_measurements.angle_min + index * rear_measurements.angle_increment + math.pi
        #     if angle > math.pi:
        #         angle -= 2 * math.pi
        #     if angle not in self.angles:
        #         if rear_measurements.ranges[index] > rear_measurements.range_min and rear_measurements.ranges[index] < rear_measurements.range_max:
        #             self.angles[angle] = rear_measurements.ranges[index]
        #         else:
        #             self.angles[angle] = -1

        # for angle in self.angles:
        #     index = round((angle - merged.angle_min)/merged.angle_increment)
        #     merged.ranges[index] = self.angles[angle]
        
        merged.pose = odom_measurements.pose
        merged.twist = odom_measurements.twist
        self.merged = merged
        self.sensor_topic.publish(self.merged)
        # print('msg sent')

    

def main():
    
    rospy.init_node('sensor_incorporation')
    sensor_data = sensor_incorporation()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass