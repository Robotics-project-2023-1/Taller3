#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

def test_node():
    rospy.init_node('test_node')
    rate = rospy.Rate(10) # 10Hz
    
    pub_zone = rospy.Publisher('/robot_manipulator_zone', String, queue_size=10)
    pub_position = rospy.Publisher('/robot_manipulator_position', Vector3, queue_size=10)
    pub_goal = rospy.Publisher('/robot_manipulator_goal', Vector3, queue_size=10)
    
    zone_options = ["zone1", "zone2", "zone3"]
    
    while not rospy.is_shutdown():
        # Publica mensajes en los tópicos
        zone_msg = String()
        zone_msg.data = zone_options[0] # Selecciona la primera zona
        pub_zone.publish(zone_msg)
        
        position_msg = Vector3()
        position_msg.x = 1.0 # Posición en x
        position_msg.y = 2.0 # Posición en y
        position_msg.z = 3.0 # Posición en z
        pub_position.publish(position_msg)
        
        goal_msg = Vector3()
        goal_msg.x = 4.0 # Posición deseada en x
        goal_msg.y = 5.0 # Posición deseada en y
        goal_msg.z = 6.0 # Posición deseada en z
        pub_goal.publish(goal_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        test_node()
    except rospy.ROSInterruptException:
        pass
