#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

def key_callback(msg: Int8):
    twist = Twist()
    
    if ((msg.data == 115) or (msg.data == 83)): # S down
        twist.linear.x = -1.0
    elif ((msg.data == 119) or (msg.data == 87)): # W up
        twist.linear.x = 1.0
    elif ((msg.data == 65) or (msg.data == 97)): # A Left
        twist.angular.z = 1.0
        twist.linear.x = 1.0
    elif ((msg.data == 68) or (msg.data == 100)): # D Right
        twist.angular.z = -1.0
        twist.linear.x = 1.0
    else:
        twist.linear.x = 0.0
        twist.angular.z = 0.0

    pub.publish(twist)
    rospy.Publisher(f'/{turtle_name}/', Twist, queue_size=10)
    

if __name__ == '__main__':
    rospy.init_node('turtle_movement')

    # Get the turtle name from a ROS parameter (unique to each player)
    turtle_name = rospy.get_param('~turtle_name', 'turtle1')

    # Initialize the publisher for this turtle's velocity commands
    pub = rospy.Publisher(f'/{turtle_name}/cmd_vel', Twist, queue_size=10)
    
    # Subscribe to the specific turtle's key topic
    rospy.Subscriber(f'/{turtle_name}/key', Int8, key_callback)

    rospy.loginfo(f"Turtle movement node started for {turtle_name}.")
    rospy.spin()
