#!/usr/bin/env python2.7

import rospy
from tf import TransformListener
import tf
import sys
from geometry_msgs.msg import Twist
import math

def tf_listener():
    # Inicializa el nodo de ROS
    print("Inicializando el nodo: 'punto_deseado'")
    rospy.init_node('punto_deseado', anonymous=True)
   

    # Crea un objeto de publicacion en el topic 'cmd_vel'
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Crea un mensaje de tipo Twist
    velocity_msg = Twist()
    
    # Crea un objeto TransformListener
    listener = TransformListener()

    # Constante de control
    k = 4

    # Bucle principal
    rate = rospy.Rate(10)
    print("iniciando control")
    while not rospy.is_shutdown():
        try:
            
	    print("Obteniendo valores deseados")
            Xd = float(sys.argv[1])  
            Yd = float(sys.argv[2]) 
            Thd = float(sys.argv[3])*3.1416/180           
            print("Calculando el error")
            ex = 10;
            ey = 10;
            eTh = 10;
            while (math.sqrt(ex*ex+ey*ey) > 0.03 or abs(eTh)>math.radians(5)):

                (trans, rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            
                # Convertir quaternion a angulos de Euler (en radianes)
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            
                # Convertir angulos de Euler de radianes a grados
                #roll_deg = math.degrees(roll)
                #pitch_deg = math.degrees(pitch)
                #yaw_deg = math.degrees(yaw)

                x = trans[0]
                y = trans[1]
                th = yaw
                xy=[x,y]
                rospy.loginfo("Transformacion: \nX: %s \nY: %s\nRotacion: %s\nError en el plano: %s" % (x, y, th, math.sqrt(ex*ex+ey*ey)))

                ex = (x-Xd);
                ey = (y-Yd);
                eTh = (th-Thd);

                ux = -k*ex;
                uy = -k*ey;
                u_th = -k*eTh;

                Vf = ux*math.cos(th)+uy*math.sin(th)
                Vl = -ux*math.sin(th)+uy*math.cos(th)
                W = u_th

                if Vf >= 0.05:
                    Vf=0.05
                elif Vf <= -0.05:
                    Vf = -0.05 

                if Vl >= 0.05:
                    Vl=0.05
                elif Vl <= -0.05:
                    Vl = -0.05

                if W >= 0.3:
                    W=0.3
                elif W <= -0.3:
                    W = -0.3

                velocity_msg.linear.x = Vf
                velocity_msg.linear.y = Vl
                velocity_msg.angular.z = W

                pub.publish(velocity_msg)
                rate.sleep()



            velocity_msg.linear.x = 0
            velocity_msg.linear.y = 0
            velocity_msg.angular.z = 0
            pub.publish(velocity_msg)

            break

        except :
           
            velocity_msg.linear.x = 0
            velocity_msg.linear.y = 0
            velocity_msg.angular.z = 0
            pub.publish(velocity_msg)

        rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Los valores deben estar dados en <metros> <metros> <grados>: rosrun tu_paquete punto_deseado.py <x> <y> <theta>")
        sys.exit(1)

    try:
	print("Iniciando...")
        tf_listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrumpido por el usuario.")
        pass

