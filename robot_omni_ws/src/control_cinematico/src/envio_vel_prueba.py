#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from geometry_msgs.msg import Twist

def publish_velocity(linear_speed_x):
    # Inicializa el nodo de ROS
    rospy.init_node('velocity_publisher', anonymous=True)

    # Crea un objeto de publicación en el topic 'cmd_vel'
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Crea un mensaje de tipo Twist
    velocity_msg = Twist()
    velocity_msg.angular.z = linear_speed_x

    # Bucle principal
    rate = rospy.Rate(10)  # Frecuencia de publicación (en Hz)
    while not rospy.is_shutdown():
        # Publica el mensaje de velocidad
        pub.publish(velocity_msg)
        rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: rosrun tu_paquete velocity_publisher.py <linear_speed_x>")
        sys.exit(1)

    try:
        linear_speed_x = float(sys.argv[1])
    except ValueError:
        print("Error: La velocidad debe ser un número")
        sys.exit(1)

    try:
        publish_velocity(linear_speed_x)
    except rospy.ROSInterruptException:
        pass

