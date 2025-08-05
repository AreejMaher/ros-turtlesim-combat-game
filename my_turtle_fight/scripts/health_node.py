#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int8
from turtlesim.srv import Kill



turtles = ['turtle1','turtle2','turtle3','turtle4']

attacked_turtle = None
attacker_turtle = None



health = {
    'turtle1': 100,
    'turtle2': 100,
    'turtle3': 100,
    'turtle4': 100
}





def attacked_callback(msg : String ):
    global attacked_turtle
    attacked_turtle = msg.data
    rospy.loginfo(f"attacked_turtle: {attacked_turtle}")

def attacker_callback(msg : String):
    global attacker_turtle
    attacker_turtle = msg.data
    rospy.loginfo(f"attacker_turtle: {attacker_turtle}")

def attacked_health_callback(msg2 : Int8):
    health[attacked_turtle] = msg2.data
    rospy.loginfo(f"health of {attacked_turtle} is {health[attacked_turtle]} in call back")

def health_callback(msg: Int8, turtle : str):
    global health
    health[turtle] = msg.data
    rospy.loginfo(f'turtle is {turtle}')

def print_remaining_turtles():
    remaining_turtles = [turtle for turtle in turtles if turtle in health]
    if len(remaining_turtles) == 1:
        rospy.loginfo(f"{remaining_turtles[0]} is the winner!")
    else:
        rospy.loginfo(f"Remaining turtles: {remaining_turtles}")
    


if __name__ == '__main__':

    rospy.init_node("health_node", anonymous=True)

    rospy.Subscriber("/attacked", String , attacked_callback)
    rospy.Subscriber("/attacker", String , attacker_callback)

    for turtle in turtles:
        rospy.Subscriber(f"/{turtle}/health", Int8, health_callback, turtle)
    rospy.Subscriber(f"/{attacked_turtle}/health", Int8, attacked_health_callback)


    rospy.wait_for_service('/kill')
    kill_service = rospy.ServiceProxy('/kill', Kill)


    # rospy.spin ()
    try:
        while not rospy.is_shutdown():
            if attacked_turtle is not None:
                if attacked_turtle in health:
                    # rospy.loginfo(f"health of {attacked_turtle} is {health[attacked_turtle]}")
                    if health[attacked_turtle] == 0:
                        health_status = "died"
                        rospy.loginfo(f"{attacked_turtle} died")
                        try:
                            response = kill_service(attacked_turtle)
                            del health[attacked_turtle]
                            rospy.loginfo(f"{attacked_turtle} killed successfully")
                            print_remaining_turtles()  # Print remaining turtles and check for winner
                        # Check for game over
                            if len(health) == 1:
                                rospy.loginfo(f"{list(health.keys())[0]} is the winner!")
                                break
                                break  # Exit the loop if there is a winner

                        except rospy.ServiceException as e:
                            rospy.logwarn(f"Service call failed: {e}")

            rospy.sleep(1)
            

    except rospy.ROSInterruptException:
        pass
    





