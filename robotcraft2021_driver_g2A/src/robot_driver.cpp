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

void callback_F(const std_msgs::Float32 &msg )
{
    sensor_msgs::Range rangeMsg;	
	rangeMsg.header.stamp = ros::Time::now();
	rangeMsg.header.frame_id = "/front_ir";
	rangeMsg.radiation_type = 1,                     
	rangeMsg.field_of_view = 0.034906585;
	rangeMsg.min_range = 0.1;
	rangeMsg.max_range = 0.8;
	rangeMsg.range = msg.data/100.0;

    
    ir_front_sensor.publish(rangeMsg);
   
   
}

void callback_L(const std_msgs::Float32 &msg )
{
    sensor_msgs::Range rangeMsg;	
	rangeMsg.header.stamp = ros::Time::now();
	rangeMsg.header.frame_id = "/left_ir";
	rangeMsg.radiation_type = 1,                     
	rangeMsg.field_of_view = 0.034906585;
	rangeMsg.min_range = 0.1;
	rangeMsg.max_range = 0.8;
	rangeMsg.range = msg.data;


    ir_left_sensor.publish(rangeMsg);
    
   
}

void callback_R(const std_msgs::Float32 &msg )
{
    sensor_msgs::Range rangeMsg;	
	rangeMsg.header.stamp = ros::Time::now();
	rangeMsg.header.frame_id = "/right_ir";
	rangeMsg.radiation_type = 1,                     
	rangeMsg.field_of_view = 0.034906585;
	rangeMsg.min_range = 0.1;
	rangeMsg.max_range = 0.8;
	rangeMsg.range = msg.data;


    ir_right_sensor.publish(rangeMsg);
    
   
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
    cmd_vel_msg =n.advertise<geometry_msgs::Twist>("cmd_vel",100);
    ros::Rate loop_rate(10);

    while(ros::ok){

        //ROS_ERROR("starting publish");
            velCommand.linear.x = 1.0;
            velCommand.angular.z = 0.0;
             pos_x_test=odom_msg.pose.pose.position.x-pos_x;

        
    
        if(pos_x_test>=0.05)
        {
            velCommand.linear.x=0;
            velCommand.angular.z=0;
            /*float pos_z_test=0;
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
            pos_x=msg.x;*/
        }
        
       

        cmd_vel_msg.publish(velCommand);


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

