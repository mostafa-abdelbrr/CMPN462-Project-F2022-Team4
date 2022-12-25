#!/usr/bin/env python3

import rospy
from rospy.rostime import Time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import pynput
from pynput.keyboard import Key, Listener


rospy.init_node('turtlesim_mover')
# Assign node as a publisher to this topic
velocity_publisher = rospy.Publisher('/robot/robotnik_base_control/cmd_vel', Twist, queue_size=10)
vel_msg=Twist()
speed=0.5


# Get the current time in seconds
    # start = Time.now().to_sec()
vel_msg.linear.x = 0
vel_msg.linear.y = 0
vel_msg.linear.z = 0
velocity_publisher.publish(vel_msg)

def on_press(key):
    print("hello",type(key))
    try:
        x='{0}'.format(key.char)
        
        print("hii",x)
        if x=='w':  #move forward
            
            while vel_msg.linear.x<15:  #max speed =10
                print(vel_msg.linear.x,vel_msg.linear.y)
                vel_msg.linear.x+=1
                vel_msg.linear.y+=0
                vel_msg.linear.z+=0
                velocity_publisher.publish(vel_msg)
        if x=='s': #move backward
            while vel_msg.linear.x>-15:  #max speed =10
                print(vel_msg.linear.x,vel_msg.linear.y)
                vel_msg.linear.x-=1
                vel_msg.linear.y+=0
                vel_msg.linear.z+=0
                velocity_publisher.publish(vel_msg)
        if x=='a':  #move forward
            
            while vel_msg.angular.z<15:  #max speed =10
            
                vel_msg.angular.z+=1
            
                print("angular z",vel_msg.angular.z)
                velocity_publisher.publish(vel_msg)
        if x=='d':  #move forward
            
            while vel_msg.angular.z>-15:  #max speed =10
                # print(vel_msg.linear.x,vel_msg.linear.y)
                vel_msg.angular.z-=1
                print("angular z",vel_msg.angular.z)
                velocity_publisher.publish(vel_msg)
    except AttributeError:
        print('special key {0} pressed'.format(key))

   





def on_release(key):
    try:
        x='{0}'.format(key.char)
        if x=='w':
            while vel_msg.linear.x>0: #decrease speed till it stops
                if vel_msg.linear.x>=2:
                    vel_msg.linear.x-=2
                    
                else:
                    vel_msg.linear.x=0
                print("release w",vel_msg.linear.x)
                velocity_publisher.publish(vel_msg)
                print("release")
        if x=='s':
            while vel_msg.linear.x<0: #decrease speed till it stops
                if vel_msg.linear.x<=-2:
                    vel_msg.linear.x+=2
                else:
                    vel_msg.linear.x=0
                print("release s",vel_msg.linear.x)
                velocity_publisher.publish(vel_msg)
                print("release")
        if x=='a':
            while vel_msg.angular.z>0: #decrease speed till it stops
                if vel_msg.angular.z>=2:
                    vel_msg.angular.z-=2
                    
                else:
                    vel_msg.angular.z=0
                print("release d",vel_msg.angular.z)
                velocity_publisher.publish(vel_msg)
                print("release")
        if x=='d':
            while vel_msg.angular.z<0: #decrease speed till it stops
                if vel_msg.angular.z<=-2:
                    vel_msg.angular.z+=2
                else:
                    vel_msg.angular.z=0
                print("release s",vel_msg.angular.z)
                velocity_publisher.publish(vel_msg)
                print("release")
    except AttributeError:
        print('special key {0} pressed'.format(key)) 




with Listener(on_press = on_press,
            on_release = on_release) as listener:

    listener.join()