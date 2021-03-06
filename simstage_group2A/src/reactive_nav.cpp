// "reactive_navigation" node: subscribes laser data and publishes velocity commands

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

double obstacle_distance_left;
double obstacle_distance_front;
double obstacle_distance_right;


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  obstacle_distance_right = *std::min_element (msg->ranges.begin()+60, msg->ranges.begin()+100);
  obstacle_distance_front = *std::min_element (msg->ranges.begin()+100, msg->ranges.begin()+140);
  obstacle_distance_left = *std::min_element (msg->ranges.begin()+140, msg->ranges.begin()+180); 
}
  double speed[2]={0,0};
  double prev[2]={0.5,0.5};




void decision_triple()
{
      if(obstacle_distance_right < 0.5 && obstacle_distance_left < 0.5 && obstacle_distance_front > 0.5)
      {
          speed[0]=0.5;
          speed[1]=0.0;
          
      }
      else if(obstacle_distance_right > 0.5 && obstacle_distance_left < 0.5 && obstacle_distance_front < 0.5)
      {
          speed[0]= 0.0;
          speed[1]=-0.5;
      }
      else if(obstacle_distance_right < 0.5 && obstacle_distance_left > 0.5 && obstacle_distance_front < 0.5)
      {
        speed[0]=0.0;
        speed[1]=0.5;
      }
      else if (obstacle_distance_right > 0.5 && obstacle_distance_left < 0.5 && obstacle_distance_front > 0.5)
      {
        speed[0]=0.0;
        speed[1]=-0.5;
      }
      else if (obstacle_distance_right < 0.5 && obstacle_distance_left > 0.5 && obstacle_distance_front > 0.5)
      {
        speed[0]=0.0;
        speed[1]=0.5;
      }
      else if (obstacle_distance_right > 0.5 && obstacle_distance_left > 0.5 && obstacle_distance_front < 0.5)
      {
        speed[0]=0.0;
        speed[1]=0.5;
      }
      else if (obstacle_distance_right > 0.5 && obstacle_distance_left > 0.5 && obstacle_distance_front > 0.5)
      {
        speed[0]=0.5;
        speed[1]=0.0;
      }
      else
      {
        speed[0]=-0.5;
        speed[1]=0.0;
      }
  
}




int check=0;
bool decision_double()
{
  bool y;
      if(obstacle_distance_right < 1 && obstacle_distance_left < 1 && obstacle_distance_front > 1){
          speed[0]=0.5;
          speed[1]=0.0;
      
          
      }
      else if(obstacle_distance_right > 1 && obstacle_distance_left < 1 && obstacle_distance_front <1){
          speed[0]= 0.0;
          speed[1]=-0.3;
      }
      else if(obstacle_distance_right < 1 && obstacle_distance_left > 1 && obstacle_distance_front < 1)
      {
        speed[0]=0.0;
        speed[1]=0.3;
      }
      else if (obstacle_distance_right > 1 && obstacle_distance_left < 1 && obstacle_distance_front > 1)
      {
        decision_triple();
      }
      else if (obstacle_distance_right < 1 && obstacle_distance_left > 1 && obstacle_distance_front > 1)
      {
        decision_triple();
      }
      else if (obstacle_distance_right > 1 && obstacle_distance_left > 1 && obstacle_distance_front < 1 )
      {
        decision_triple();
      }
      else if (obstacle_distance_right > 1 && obstacle_distance_left > 1 && obstacle_distance_front > 1)
      {
        speed[0]=0.5;
        speed[1]=0.0;
      }
      else
      {
        decision_triple();

      }

}

int x=0;
int loop =0;
int loop_r=0;
//initializations:
  geometry_msgs::Twist cmd_vel_msg;
void decision()
{
      
      if( obstacle_distance_right<1.5 && obstacle_distance_left>=2.0)
      {
        
        if(obstacle_distance_right<1 && obstacle_distance_front>=2.0)
        {
          speed[0]=0.3;
          speed[1]=0.4;
          prev[0]=0.3;
          prev[1]=0.4;
          ROS_INFO("1");
        }
        else if(obstacle_distance_front>=2.0)
        {
          
           speed[0]=0.5;
          speed[1]=-0.5;
          ROS_INFO("22");
          
          
        }
        
        else if(obstacle_distance_front<=0.5)
        {
          
            speed[0]=0.0;
            speed[1]=-0.3;
            prev[0]=0.0;
          prev[1]=-0.3;
            ROS_INFO("3");
        }
      }
      else if(obstacle_distance_right>1.5 && obstacle_distance_left>=2.0)
      {
        if(obstacle_distance_front>=2.0 )
        {
          speed[0]=0.5;
          speed[1]=-0.4;
          prev[0]=0.5;
          prev[1]=-0.4; 
          ROS_INFO("4"); 
        }
        else if(obstacle_distance_front<2.0)
        {
          speed[0]=0.5;
          speed[1]=-0.4;
          prev[0]=0.0;
          prev[1]=0.4;
          ROS_INFO("5");
        }
        
      }
      else if(obstacle_distance_right<=1.5 && obstacle_distance_left<=2.0)
      {
        if(obstacle_distance_right<1.0 && obstacle_distance_front>=2.0)
        {
          speed[0]=0.5;
          speed[1]=0.2;
          prev[0]=0.5;
          prev[1]=0.2;
          ROS_INFO("6");
        }
        else if(obstacle_distance_front<=2.0)
        {
          speed[0]=0.0;
          speed[1]=0.5;
          prev[0]=0.3;
          prev[1]=0.5;
          ROS_INFO("7");
        }
        else if(obstacle_distance_front>=2.0)
        {
          speed[0]=0.5;
          speed[1]=0.0;
          prev[0]=0.5;
          prev[1]=0.0;
          ROS_INFO("8");
        }
      }
      else if(obstacle_distance_right>1.5 )
      {

          speed[0]=0.2;
          speed[1]=-0.4;
          prev[0]=0.2;
          prev[1]=-0.4;
          ROS_INFO("9");
          
      }
      

}


int main(int argc, char **argv){
    
  bool check= true;
  bool left= false, right = false, front =false;

  ros::init(argc, argv, "reactive_nav");
  
  ros::NodeHandle n;

  //Publisher for /cmd_vel
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  //Subscriber for /base_scan
  ros::Subscriber laser_sub = n.subscribe("base_scan", 100, laserCallback);

  ros::Rate loop_rate(10); //10 Hz



  

  while (ros::ok()){

   
   decision();
   

    cmd_vel_msg.linear.x = speed[0];
    cmd_vel_msg.angular.z = speed[1];
    

   /*
   if(obstacle_distance_right> 1)
   {
      ROS_INFO("right distance %f", obstacle_distance_right);
     cmd_vel_msg.linear.x = 0.5;
     cmd_vel_msg.angular.z =0.0;
   } 
   else
   {
  ROS_INFO("right distance stop %f", obstacle_distance_right);
     cmd_vel_msg.linear.x = 0.0;
     cmd_vel_msg.angular.z = 0.0;
   }
 */
    //publish velocity commands:
    ROS_INFO("left distance stop %f", obstacle_distance_left);
    ROS_INFO("front distance stop %f", obstacle_distance_front);
    ROS_INFO("right distance stop %f", obstacle_distance_right);
    cmd_vel_pub.publish(cmd_vel_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

