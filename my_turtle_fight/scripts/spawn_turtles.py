#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn

def spawn_turtle(x, y, theta, name):
    rospy.wait_for_service('/spawn')
    try:
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        spawn_turtle(x, y, theta, name)
        rospy.loginfo(f"Turtle '{name}' spawned at ({x}, {y}) with theta={theta}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('spawn_turtles')

    # Coordinates and names for the turtles
    turtles = [
        {'x': 1.0, 'y': 9.0, 'theta': 0.0, 'name': 'turtle2'},
        {'x': 9.0, 'y': 3.0, 'theta': 0.0, 'name': 'turtle3'},
        {'x': 5.0, 'y': 8.0, 'theta': 0.0, 'name': 'turtle4'}
    ]

    for turtle in turtles:
        spawn_turtle(turtle['x'], turtle['y'], turtle['theta'], turtle['name'])

    rospy.loginfo("All turtles have been spawned.")
