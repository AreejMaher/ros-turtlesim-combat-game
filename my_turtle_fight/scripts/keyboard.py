#!/usr/bin/env python3
import getch
import rospy
from std_msgs.msg import Int8

def keys():
    rospy.init_node('keyboard_node', anonymous=True)
    
    # Get the turtle name 
    turtle_name = rospy.get_param('~turtle_name', 'turtle1')
    turtle_attack_topic = f"/{turtle_name}/attack_time" 
    turtle_key_topic = f"/{turtle_name}/key"
    pub1 = rospy.Publisher(turtle_key_topic, Int8, queue_size=10)
    pub2 = rospy.Publisher(turtle_attack_topic, Int8, queue_size=10)

    rate = rospy.Rate(100) 
    while not rospy.is_shutdown():
        k = ord(getch.getch())  
        if k in [119, 115, 100, 97, 113, 65, 87, 83, 68, 81]:  # Filtered keys (q/Q, w/W, s/S, d/D, a/A)
            rospy.loginfo(f"Key pressed for {turtle_name}: {chr(k)}")
            pub1.publish(k)
        if ( (k == 113) or (k == 81)):
            pub2.publish(k)
        rate.sleep()

if __name__ == '__main__':
    try:
        keys()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")
