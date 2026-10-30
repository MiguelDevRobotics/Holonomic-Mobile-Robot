#!/usr/bin/env python2.7

import rospy
from tf import TransformListener
import tf

def tf_listener():
    # Inicializa el nodo de ROS
    rospy.init_node('tf_listener', anonymous=True)

    # Crea un objeto TransformListener
    listener = TransformListener()

    # Bucle principal
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            
            rospy.loginfo("Transformacion: \nTranslacion: %s \nRotacion: %s" % (trans, rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        rate.sleep()

if __name__ == '__main__':
    try:
        tf_listener()
    except rospy.ROSInterruptException:
        pass

