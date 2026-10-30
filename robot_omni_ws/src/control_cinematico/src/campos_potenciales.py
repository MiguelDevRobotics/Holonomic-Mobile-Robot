#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import rospy
from tf import TransformListener
import tf
import sys
from geometry_msgs.msg import Twist
import math
import numpy as np
from sensor_msgs.msg import LaserScan

class RobotControl:
    def __init__(self):
        rospy.init_node('punto_deseado', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)  # Suscribirse al topic del LiDAR
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.velocity_msg = Twist()
        self.listener = TransformListener()
        self.k = 4
        self.rate = rospy.Rate(10)
        self.num_angulos = 508  # Numero de ángulos para el LiDAR
        self.ranges = []  # Variable de instancia para almacenar los rangos del LiDAR



    def C_rep(self, K=5, Q=3):
        Qd = Q
        Katrac = K
        Frepx = 0
        Frepy = 0

        xsi = self.xs #[s[0] for s in S]
        ysi = self.ys #[s[1] for s in S]
        N = len(xsi)

        dmax = 1.0
        dmin = 0.2
        Krep = Katrac 

        for i in range(N):
            dsi = math.sqrt(xsi[i] ** 2 + ysi[i] ** 2)
            if dsi >= dmax:
                Frepxi = 0
                Frepyi = 0
            elif dmin <= dsi < dmax:
                Frepxi = -(Krep / dsi ** 3) * (1 / dsi - 1 / dmax) * xsi[i]
                Frepyi = -(Krep / dsi ** 3) * (1 / dsi - 1 / dmax) * ysi[i]
            else:
                Frepxi = 0
                Frepyi = 0
            Frepx += Frepxi
            Frepy += Frepyi
        rospy.loginfo("repulsiones:{},{}".format(Frepx,Frepy))
            
        return [Frepx, Frepy]

    def C_atrac(self, xc, yc, xd, yd):
        x = xc
        y = yc
        d = math.sqrt((x - xd) ** 2 + (y - xd) ** 2)
        Qd = 3
        Katrac = 5
        if d < Qd:
            Fx = Katrac * (x - xd)
            Fy = Katrac * (y - yd)
        else:
            Fx = (Qd / d) * Katrac * (x - xd)
            Fy = (Qd / d) * Katrac * (y - yd)
        return [-Fx, -Fy]

    def lidar_callback(self, data):
        self.ranges = np.array(data.ranges)
        self.ranges[np.isnan(self.ranges)] = 12
        self.angulos = np.linspace(0, 2*np.pi, len(self.ranges))
        self.xs = self.ranges * np.cos(self.angulos)
        self.ys = self.ranges * np.sin(self.angulos)
        #rospy.loginfo("Datos del LiDAR recibidos: {}".format(self.ranges))

    def tf_listener(self, Xd, Yd, Thd):
        print("Esperando transformaciones...")

        try:
            while not rospy.is_shutdown():
                try:
                    (trans, rot) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
                    roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
                    x, y = trans[0], trans[1]
                    th = yaw
                    xy = [x, y]

                    ex = (x - Xd)
                    ey = (y - Yd)
                    eTh = (th - Thd)
                    error_planar = math.sqrt(ex * ex + ey * ey)

                    rospy.loginfo("Transformacion:\nX: %s\nY: %s\nRotacion: %s\nError en el plano: %s \nError de rotacion: %s" % (x, y, th, error_planar, math.degrees(eTh)))
                    
                    Ua = self.C_atrac(x, y, Xd, Yd)
                    Ur = self.C_rep()
                    ux = 0 * Ua[0] + Ur[0]
                    uy = 0 * Ua[1] + Ur[1]
                    u_th = -self.k * eTh

                    Vf = ux * math.cos(th) + uy * math.sin(th)
                    Vl = -ux * math.sin(th) + uy * math.cos(th)
                    W = u_th

                    if abs(Vf) > 0.095:
                        Vf = 0.095 if Vf > 0 else -0.095

                    if abs(Vl) > 0.095:
                        Vl = 0.095 if Vl > 0 else -0.095

                    if abs(W) > 0.4:
                        W = 0.4 if W > 0 else -0.4

                    if error_planar <= 0.01 and abs(eTh) <= math.radians(5):
                        self.velocity_msg.linear.x = 0
                        self.velocity_msg.linear.y = 0
                        self.velocity_msg.angular.z = 0

                        self.pub.publish(self.velocity_msg)
                        break

                    self.velocity_msg.linear.x = Vf
                    self.velocity_msg.linear.y = Vl
                    self.velocity_msg.angular.z = W

                    self.pub.publish(self.velocity_msg)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn("No se pudo obtener la transformacion. Volviendo a intentar...")

                self.rate.sleep()

        except rospy.ROSInterruptException:
            rospy.loginfo("Interrumpido por el usuario.")

        finally:
            rospy.loginfo('Deteniendo el robot')
            self.rate.sleep()
            self.velocity_msg.linear.x = 0
            self.velocity_msg.linear.y = 0
            self.velocity_msg.angular.z = 0
            self.pub.publish(self.velocity_msg)
            rospy.loginfo('El nodo se ha cerrado correctamente')

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Los valores deben estar dados en <metros> <metros> <grados>: rosrun tu_paquete punto_deseado.py <x> <y> <theta>")
        sys.exit(1)

    try:
        print("Iniciando...")
        Xd = float(sys.argv[1])
        Yd = float(sys.argv[2])
        Thd = float(sys.argv[3]) * 3.1416 / 180
        
        controlador = RobotControl()
        controlador.tf_listener(Xd, Yd, Thd)
    
    except rospy.ROSInterruptException:
        pass
