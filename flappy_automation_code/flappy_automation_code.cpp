#include "ros/ros.h"
#include "flappy_automation_code/flappy_automation_code.hpp"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include "math.h"
#include <iostream> 
#include <vector>  

class SubscribeAndPublish
{
  public:
       double velx;
       double upper;
       double lower;
       double theta3f; 
       double theta4f;
       double theta5f;
       double theta6f;
       double medd;
       bool init =0;
       int frm = 0;
       double meddy;
       SubscribeAndPublish()
       {
        pub = nh.advertise<geometry_msgs::Vector3>("/flappy_acc", 1);
        sub = nh.subscribe("/flappy_laser_scan", 1, &SubscribeAndPublish::accCallback, this);
        sub2 = nh.subscribe("/flappy_vel", 1, &SubscribeAndPublish::velCallback, this);
       }

  private:
       ros::NodeHandle nh; 
       ros::Publisher pub;
       ros::Subscriber sub;
       ros::Subscriber sub2;

  void accCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void velCallback(const geometry_msgs::Vector3::ConstPtr& msg);
};

void SubscribeAndPublish::velCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
velx=msg->x;
//ROS_INFO("velcoty %f", velx);
}


void SubscribeAndPublish::accCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  theta3f=msg->ranges[3]*cos(-0.19634954631);
  theta4f=msg->ranges[5]*cos(-0.19634954631);
  theta5f=-1*msg->ranges[3]*sin(-0.19634954631);
  theta6f=-1*msg->ranges[5]*sin(-0.19634954631);
  
  upper=msg->ranges[0]+msg->ranges[1]+msg->ranges[2]+msg->ranges[3];
  lower=msg->ranges[5]+msg->ranges[6]+msg->ranges[7]+msg->ranges[8];


  geometry_msgs::Vector3 acc_cmd;
acc_cmd.x = 0;
    acc_cmd.y = 0;


  meddy=msg->ranges[4];

  /*if(msg->ranges[3]*cos(-0.19634954631)>2 && msg->ranges[4]>2 && msg->ranges[5]*cos(-0.19634954631)>2)
  {
    acc_cmd.x = 1.1;
    acc_cmd.y = 0;
  }

  else
  {
    acc_cmd.x=-velx*velx/.1;
    if(msg->ranges[3]*sin(-0.19634954631)>0.5 && msg->ranges[5]*sin(0.19634954631)>0.5 && msg->ranges[4]>1)
        acc_cmd.y = 0;
else{
    if(msg->ranges[0]*sin(0.19634954631*4)<0.35)
	acc_cmd.y = 1;
    else
    {
    if(msg->ranges[8]*sin(0.19634954631*4)<0.35)
	acc_cmd.y = -1;
    else
    {
    
    if(msg->ranges[3]*sin(0.19634954631)<0.5)
	acc_cmd.y = .5;
    if(msg->ranges[5]*sin(0.19634954631)<0.5)
	acc_cmd.y = -.5;

    }
    }

    }
  }*/

frm++;
  pub.publish(acc_cmd);
//ROS_INFO("all %f",  msg->ranges[0]*sin(0.19634954631*4));
  ROS_INFO("%f %f %f %f %f %f %f %f %f", msg->ranges[0], msg->ranges[1], msg->ranges[2], msg->ranges[3], msg->ranges[4], msg->ranges[5], msg->ranges[6], msg->ranges[7],msg->ranges[8]);

  //ROS_INFO("frame: %i forward %f down - %f up- %f down| %f up| %f", frm, msg->ranges[4], theta3f , theta4f, theta5f , theta6f);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"flappy_automation_code");

  SubscribeAndPublish subscribeAndPublish;

  ros::spin();
  return 0;
}
