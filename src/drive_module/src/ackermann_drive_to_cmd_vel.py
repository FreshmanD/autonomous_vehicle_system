#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

def cmd_callback(data):    
  msg = Twist()
  msg.linear.x = data.drive.speed
  msg.angular.z = data.drive.speed * data.drive.steering_angle
  rospy.loginfo("A to C publishing speed: linear.x: %f, angular.z: %f", msg.linear.x, msg.angular.z)
  pub.publish(msg)
  

if __name__ == '__main__': 
  try:
    
    rospy.init_node('ackermann_drive_to_cmd_vel')
        
    wheelbase = rospy.get_param('~wheelbase', 1.0)
    
    sub = rospy.Subscriber('/vehicle_control_ackermann', AckermannDriveStamped, cmd_callback, queue_size=1)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    rospy.loginfo("Node 'ackermann_drive_to_cmd_vel' started.")
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass