#!/usr/bin/env python2.7

import rospy
from tf import TransformListener
import tf
import sys
from geometry_msgs.msg import Twist
import math

def tf_listener():
    rospy.init_node('punto_deseado', anonymous=True)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    velocity_msg = Twist()
    listener = TransformListener()
    k = 4
    rate = rospy.Rate(10)

    print("Esperando transformaciones...")

    try:
        Xd = float(sys.argv[1])
        Yd = float(sys.argv[2])
        Thd = float(sys.argv[3]) * 3.1416 / 180
        
        while not rospy.is_shutdown():
            try:
                (trans, rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
                x, y = trans[0], trans[1]
                th = yaw
                xy = [x, y]

                # Calculo de errores
                ex = (x - Xd)
                ey = (y - Yd)
                eTh = (th - Thd)
                error_planar = math.sqrt(ex * ex + ey * ey)

                rospy.loginfo("Transformacion:\nX: %s\nY: %s\nRotacion: %s\nError en el plano: %s \nError de rotacion: %s" % (x, y, th, error_planar,math.degrees(eTh)))

                # Calculo de ganancias
                ux = -k * ex
                uy = -k * ey
                u_th = -k * eTh

                # Calculo de velocidades
                Vf = ux * math.cos(th) + uy * math.sin(th)
                Vl = -ux * math.sin(th) + uy * math.cos(th)
                W = u_th

                #Limitacion de las ganancias
                if Vf >= 0.095:
                    Vf = 0.095
                elif Vf <= -0.095:
                    Vf = -0.095

                if Vl >= 0.095:
                    Vl = 0.095
                elif Vl <= -0.095:
                    Vl = -0.095

                if W >= 0.4:
                    W = 0.4
                elif W <= -0.4:
                    W = -0.4

                if error_planar <= 0.01 and abs(eTh) <= math.radians(5):
                    velocity_msg.linear.x = 0
                    velocity_msg.linear.y = 0
                    velocity_msg.angular.z = 0

                    pub.publish(velocity_msg)

                    break

                velocity_msg.linear.x = Vf
                velocity_msg.linear.y = Vl
                velocity_msg.angular.z = W

                pub.publish(velocity_msg)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("No se pudo obtener la transformacion. Volviendo a intentar...")

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrumpido por el usuario.")

    finally:
        rospy.loginfo('Deteniendo el robot')

        rate.sleep()
        velocity_msg.linear.x = 0
        velocity_msg.linear.y = 0
        velocity_msg.angular.z = 0
        pub.publish(velocity_msg)

        rospy.loginfo('El nodo se ha cerrado correctamente')

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Los valores deben estar dados en <metros> <metros> <grados>: rosrun tu_paquete punto_deseado.py <x> <y> <theta>")
        sys.exit(1)

    try:
        print("Iniciando...")
        tf_listener()
    except rospy.ROSInterruptException:
        pass
