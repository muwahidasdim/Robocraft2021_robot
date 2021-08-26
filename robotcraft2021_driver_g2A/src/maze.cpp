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
double speed[2]={0,0};
double prev[2]={0,0};


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  obstacle_distance_right = *std::min_element (msg->ranges.begin()+60, msg->ranges.begin()+100);
  obstacle_distance_front = *std::min_element (msg->ranges.begin()+100, msg->ranges.begin()+140);
  obstacle_distance_left = *std::min_element (msg->ranges.begin()+140, msg->ranges.begin()+180); 
}

ros::Publisher odom_pub;




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


    odom_pub.publish (odom_msg);
}


void decision()
{
      
      if(obstacle_distance_right<0.15)
      {
        speed[0]=0.0;
        speed[1]=0.3;
      }
      if(obstacle_distance_right>0.5)
      {
        speed[0]=0.0;
        speed[1]=-0.3;
      }
      if(obstacle_distance_right<0.5 && obstacle_distance_front>0.5)
      {
        speed[0]=0.5;
        speed[1]=0.0;
      }
      if(obstacle_distance_right<0.5 && obstacle_distance_front<0.5)
      {
        speed[0]=0.0;
        speed[1]=0.3;
      }
      if(obstacle_distance_right>0.5 && obstacle_distance_front>0.5)
      {
        speed[0]=0.0;
        speed[1]=0.3;
      }
  
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle nh;

    ros::Subscriber odom = nh.subscribe("odom",100,callback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom_publisher",100);
    ros::Publisher cmd_vel_msg = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  //Subscriber for /base_scan
   ros::Subscriber laser_sub = nh.subscribe("base_scan", 100, laserCallback);

    ros::Rate loop_rate(10); //10 Hz



  //initializations:
  geometry_msgs::Twist cmd_vel;

  while (ros::ok()){

       
   decision();
   

    cmd_vel.linear.x = speed[0];
    cmd_vel.angular.z = speed[1];
    


    if(obstacle_distance_left<=0.15)
    ROS_INFO("left distance stop %f", obstacle_distance_left);

    if(obstacle_distance_front<=0.15)
    ROS_INFO("front distance stop %f", obstacle_distance_front);

    if(obstacle_distance_right<=0.15)
    ROS_INFO("right distance stop %f", obstacle_distance_right);
    cmd_vel_msg.publish(cmd_vel);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}


