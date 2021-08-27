
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Range.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"


ros::Publisher odom;
ros::Publisher ir_front_sensor;
ros::Publisher ir_left_sensor;
ros::Publisher ir_right_sensor;
ros::Publisher cmd_vel_msg;
geometry_msgs::Twist velCommand;
nav_msgs::Odometry odom_msg;

//variable declaration
    
    float pos_x_test=0;
    float pos_x=0;


void callback(const geometry_msgs::Pose2D &msg )
{

static tf::TransformBroadcaster odom_broadcaster;
geometry_msgs::Twist cmd_vel;
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

        

       


    odom.publish (odom_msg);
    
    
   
}
sensor_msgs::Range rangeMsg_F;
void callback_F(const std_msgs::Float32 &msg )
{
    	
	rangeMsg_F.header.stamp = ros::Time::now();
	rangeMsg_F.header.frame_id = "/front_ir";
	rangeMsg_F.radiation_type = 1,                     
	rangeMsg_F.field_of_view = 0.034906585;
	rangeMsg_F.min_range = 0.1;
	rangeMsg_F.max_range = 0.8;
	rangeMsg_F.range = msg.data/100.0;

    
    ir_front_sensor.publish(rangeMsg_F);
   
   
}
sensor_msgs::Range rangeMsg_L;
void callback_L(const std_msgs::Float32 &msg )
{
    	
	rangeMsg_L.header.stamp = ros::Time::now();
	rangeMsg_L.header.frame_id = "/left_ir";
	rangeMsg_L.radiation_type = 1,                     
	rangeMsg_L.field_of_view = 0.034906585;
	rangeMsg_L.min_range = 0.1;
	rangeMsg_L.max_range = 0.8;
	rangeMsg_L.range = msg.data/100.0;


    ir_left_sensor.publish(rangeMsg_L);
    
   
}
sensor_msgs::Range rangeMsg_R;
void callback_R(const std_msgs::Float32 &msg )
{
    	
	rangeMsg_R.header.stamp = ros::Time::now();
	rangeMsg_R.header.frame_id = "/right_ir";
	rangeMsg_R.radiation_type = 1,                     
	rangeMsg_R.field_of_view = 0.034906585;
	rangeMsg_R.min_range = 0.1;
	rangeMsg_R.max_range = 0.8;
	rangeMsg_R.range = msg.data/100.0;


    ir_right_sensor.publish(rangeMsg_R);
    
   
}

int main(int argc, char **argv){
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    ROS_ERROR("starting publish");
    ros::Subscriber pose = n.subscribe("pose",100,callback);
    ros::Subscriber front_distance = n.subscribe("front_distance",5,callback_F);
    ros::Subscriber left_distance = n.subscribe("left_distance",5,callback_L);
    ros::Subscriber right_distance = n.subscribe("right_distance",5,callback_R);
    odom = n.advertise<nav_msgs::Odometry>("odom",100);



    ir_front_sensor = n.advertise<sensor_msgs::Range>("ir_front_sensor",5);
    ir_left_sensor = n.advertise<sensor_msgs::Range>("ir_left_sensor",5);
    ir_right_sensor = n.advertise<sensor_msgs::Range>("ir_right_sensor",5);
    cmd_vel_msg =n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    ros::Rate loop_rate(2);

    while(ros::ok){

        
       //ROS_ERROR("starting publish");
            
	velCommand.linear.x = 0.2;
            velCommand.angular.z = -1.5;	            
	 //pos_x_test=odom_msg.pose.pose.position.x-pos_x;
	

        
    	/*if(rangeMsg_F.range<=2.0)
	{
		velCommand.linear.x = 0.3;
            velCommand.angular.z = 1.0;
		ROS_ERROR("starting publish");
	}
       /* if(pos_x_test>=0.30)
        {
            velCommand.linear.x=0;
            velCommand.angular.z=0;
            float pos_z_test=0;
            float pos_z=msg.theta;
            

            velCommand.linear.x=0;
            velCommand.angular.z =1;
            while(pos_z_test<=1.57079633)
            {
                pos_z_test=msg.theta-pos_z;
                pos_z=msg.theta;

            }
            velCommand.linear.x=1;
            velCommand.angular.z =0;
            
            pos_x_test=0;
            pos_x=odom_msg.pose.pose.position.x;
        }*/
	if( rangeMsg_R.range<1.5)
      {
        
        if(rangeMsg_R.range<1.0 && rangeMsg_F.range>=2.5)
        {
          velCommand.linear.x=0.3;
          velCommand.angular.z=-1.0;

          ROS_INFO("1");
        }
        else if(rangeMsg_F.range>=2.5)
        {
          
           velCommand.linear.x=0.2;
          velCommand.angular.z=-1.0;
          ROS_INFO("22"); 
        }
        
        else if(rangeMsg_F.range<2.5)
        {
          
           velCommand.linear.x=0.2;
            velCommand.angular.z=-1.5;

            ROS_INFO("3");
         }
      }
      else if(rangeMsg_R.range>2.0 )
      {
        if(rangeMsg_F.range>=2.0 )
        {
          velCommand.linear.x=0.4;
          velCommand.angular.z=1.0;

          ROS_INFO("4"); 
        }
        else if(rangeMsg_F.range<2.0)
        {
          velCommand.linear.x=0.2;
          velCommand.angular.z=0.9;

          ROS_INFO("5");
        }
        
      }
      else if(rangeMsg_R.range<=1.5 && rangeMsg_L.range<=2.0)
      {
        if(rangeMsg_R.range<1.0 && rangeMsg_F.range>=2.5)
        {
          velCommand.linear.x=0.2;
          velCommand.angular.z=-1.0;

          ROS_INFO("6");
        }
        else if(rangeMsg_F.range<2.5)
        {
          velCommand.linear.x=0.2;
          velCommand.angular.z=-1.0;

          ROS_INFO("7");
        }
        else if(rangeMsg_F.range>=2.5)
        {
          velCommand.linear.x=1.0;
          velCommand.angular.z=0.0;

          ROS_INFO("8");
        }
      }
      else if(rangeMsg_R.range>2.5)
      {

          velCommand.linear.x=0.3;
          velCommand.angular.z=1.0;
          ROS_INFO("9");
          
      }

        cmd_vel_msg.publish(velCommand);


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
