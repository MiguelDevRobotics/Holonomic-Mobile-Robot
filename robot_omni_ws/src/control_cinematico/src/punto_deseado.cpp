#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h" //Required for using the ft class
#include "geometry_msgs/Pose2D.h" //To publish the tracking errors using the Pose2D message type
#include <tf2_ros/transform_listener.h>


#include <iostream>
#include <stdexcept> // Para std::invalid_argument
#include <cstdlib> // Para std::strtod
#include <cmath>

using namespace std;

ros::Publisher vel_pub;
ros::Subscriber pose_sub;


//Global message types
geometry_msgs::Twist vel_msg;

double x, y, yaw, l = 0.1; //Global variables (robot posture) and the offset of the outside point
int key;

double t, t0, Vf_max = 0.22, Vl_max = 0.22, W_max = 2.84; //Timer, initial time, maximum velocities [m/s, rad/s], respectively

double T = 100, k = 0.15; //Trajectory period, controller gains kx = ky = k
double ex, ey, eTh, Xd, Yd, Thd; //Tracking errors, desired position, and time derivative, respectively
double Vf,Vl, W; //Control signals


/*void OdomCallback(const nav_msgs::Odometry::ConstPtr & msg){//Callback function to get the robot posture 
  double roll, pitch; //Local variables
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;

  //Operations to convert from quaternion to Euler angles
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);  //Euler angles are given in radians
}*/

int main(int argc, char **argv)
{
    if (argc != 4) {
        cerr << "Uso: " << argv[0] << " <x> <y> <theta>" << std::endl;
        return 1; // Salir con código de error
    }

	ros::init(argc, argv, "omni_robot_controller"); //Initialize the node
        ros::NodeHandle nh; //Create the node handle
        ros::Rate loop_rate(50); //Node frequency (Hz)
	tf2_ros::Buffer tfBuffer;
 	tf2_ros::TransformListener tfListener(tfBuffer);
	 ros::Rate rate(10.0);
	geometry_msgs::TransformStamped transformStamped;
   try{
     transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0)); 
	
    double x = transformStamped.transform.translation.x;
    double y = transformStamped.transform.translation.y;

    // Extraer la orientación alrededor del eje z (yaw)
    tf::Quaternion quat;
    tf::quaternionMsgToTF(transformStamped.transform.rotation, quat);
    double roll, pitch;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);      
}
   catch (tf2::TransformException &ex) {
     ROS_WARN("%s",ex.what());
     ros::Duration(1.0).sleep();
	vel_msg.linear.x = 0; vel_msg.linear.y = 0; vel_msg.angular.z = 0;
        vel_pub.publish(vel_msg);
	return 1;
}
	
    try {
        float Xd = stof(argv[1]);
        float Yd = stof(argv[2]);
        float Thd = stof(argv[3]);


        

        int counter = 0;

        
        ex = (0-Xd);
        ey = (0-Yd);
        eTh = (0-Thd);

        vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10); //To publish in the topic
        //pose_sub = nh.subscribe("/tf", 10, OdomCallback); //To subscribe to the topic


        vel_msg.linear.z = vel_msg.angular.x =  vel_msg.angular.y = 0;

        while (sqrt(ex*ex+ey*ey)>0.02){

		
	    
	    

            float th = yaw;

            ex = (x-Xd);
            ey = (y-Yd);
            eTh = (th-Thd);

            float ux = -k*ex;
            float uy = -k*ey;
            float u_th = -k*eTh;

            Vf = ux*cos(th)-uy*sin(th);
            Vl = -ux*sin(th)+uy*cos(th);
            W = u_th;

            if(abs(Vf)>Vf_max){Vf = Vf_max*abs(Vf)/Vf;}
            if(abs(Vl)>Vl_max){Vl = Vl_max*abs(Vl)/Vl;}
            if(abs(W)>W_max){W = W_max*abs(W)/W;}

            vel_msg.linear.x = Vf; vel_msg.linear.y = Vl; vel_msg.angular.z = W;
            vel_pub.publish(vel_msg); //Publish the control signals
        }

        //Stop the mobile robot
        vel_msg.linear.x = 0; vel_msg.linear.y = 0; vel_msg.angular.z = 0;
        vel_pub.publish(vel_msg);

    } catch (const std::invalid_argument& e) {
        std::cerr << "Error: Al menos uno de los argumentos no es un número flotante válido." << std::endl;
        return 1; // Salir con código de error
    }

    return 0;
 
}

