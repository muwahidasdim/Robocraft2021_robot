#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Range.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"


double obstacle_distance_left;
double obstacle_distance_front;
double obstacle_distance_right;


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  obstacle_distance_right = *std::min_element (msg->ranges.begin()+60, msg->ranges.begin()+100);
  obstacle_distance_front = *std::min_element (msg->ranges.begin()+100, msg->ranges.begin()+140);
  obstacle_distance_left = *std::min_element (msg->ranges.begin()+140, msg->ranges.begin()+180); 
}

ros::Publisher odom_pub;

    float pos_x=0;
    float pos_z=0;


void callback(const geometry_msgs::Pose2D &msg )
{
  static tf::TransformBroadcaster odom_broadcaster;
  nav_msgs::Odometry odom_msg;
  geometry_msgs::TransformStamped odom_trans;
  geometry_msgs::Quaternion odom_quat;




    odom_quat = tf::createQuaternionMsgFromYaw(msg.theta);

     odom_trans.header.stamp = ros::Time::now();
     odom_trans.header.frame_id = "odom";
     odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = msg.x;
    odom_trans.transform.translation.y = msg.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
 
       //send the transform
    odom_broadcaster.sendTransform(odom_trans);


    odom_msg.pose.pose.position.x = msg.x;
    odom_msg.pose.pose.position.y = msg.y;
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation = odom_quat;//odom_msg.pose.pose.orientation=msg.theta;//function to convert theta
    /*geometry_msgs::Twist cmd_vel;
    float pos_x_test=0;
    ROSINFO("%f",pos_x);
    pos_x_test=odom_msg.pose.pose.position.x-pos_x;
    pos_x=odom_msg.pose.pose.position.x;
        if(pos_x_test>=0.5)
        {
            cmd_vel.linear.x=0;
            cmd_vel.angular.z=0;
            pos_x_test=0;
        }

    */
    odom_pub.publish (odom_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle nh;

        ros::Subscriber odom = nh.subscribe("odom",100,callback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom_publisher",100);
   // ros::Publisher cmd_vel_msg = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  //Subscriber for /base_scan
   ros::Subscriber laser_sub = nh.subscribe("base_scan", 100, laserCallback);

    ros::Rate loop_rate(10); //10 Hz
     //initializations:
  geometry_msgs::Twist cmd_vel;
  //geometry_msgs::Pose2D msg;

  
    //float pos_x_test=0;
    //pos_x=msg.x;
    //ROS_INFO(msg.x);
    

  while (ros::ok()){


        cmd_vel.linear.x=1.0;
            cmd_vel.angular.z=0.0;


    /*pos_x_test=msg.x-pos_x;
    
        if(pos_x_test>=0.5)
        {
            cmd_vel.linear.x=0;
            cmd_vel.angular.z=0;
            float pos_z_test=0;
            pos_x=msg.x;
            pos_z=msg.theta;
            

            cmd_vel.linear.x=0;
            cmd_vel.angular.z=1;
            while(pos_z_test<=1.57079633)
            {
                pos_z_test=msg.theta-pos_z;


            }
            cmd_vel.linear.x=1;
            cmd_vel.angular.z=0;
            
            pos_x_test=0;
        }

    //if(obstacle_distance_left<=0.15)
    ROS_INFO("left distance stop %f", obstacle_distance_left);

    //if(obstacle_distance_front<=0.15)
    ROS_INFO("front distance stop %f", obstacle_distance_front);

    //if(obstacle_distance_right<=0.15)
    ROS_INFO("right distance stop %f", obstacle_distance_right);
    cmd_vel_msg.publish(cmd_vel);

  ROS_INFO("HELLOOOOO %f");*/


    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}




