#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

def lidar_callback(data):
    # Esta funcion se ejecutara cada vez que se reciba un mensaje del LiDAR
     # Convertir los rangos del LiDAR a un arreglo NumPy
    ranges = np.array(data.ranges)

    # Reemplazar NaN por 12
    ranges[np.isnan(ranges)] = 12
    rospy.loginfo('Size of ranges: {}'.format(len(ranges)))
    #rospy.loginfo("Datos del LiDAR recibidos: {}".format(ranges))

def lidar_listener():
    rospy.init_node('lidar_reader', anonymous=True)  # Inicia el nodo
    rospy.Subscriber("/scan", LaserScan, lidar_callback)  # Suscribete al topic del LiDAR
    rospy.spin()  # Manten el nodo en ejecucion

if __name__ == '__main__':
    lidar_listener()
