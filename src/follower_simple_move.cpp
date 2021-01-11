#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <sstream>
#include "PathHolder.h"
#include "LocalMap.h"

ros::Publisher vel_pub;
geometry_msgs::Twist vel_msg;
geometry_msgs::PoseStamped target_pose;
bool bMove = false;
int Robot_ID = 1;

void MoveCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // 收到移动目标点
    target_pose = *msg;
    bMove = true;
}

void BehaviorsCallback(const std_msgs::String::ConstPtr& msg)
{
    // 收到停止移动指令
    int nFindIndex = 0;
    nFindIndex = msg->data.find("move stop");
    if( nFindIndex >= 0 )
    {
        bMove = false;
        vel_msg.linear.x = 0;
        vel_msg.linear.y = 0;
        vel_msg.angular.z = 0;
        vel_pub.publish(vel_msg);
        ROS_WARN("[move stop] ");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "follower_simple_move"); 

    ros::NodeHandle n;
    // 订阅主题获取移动目标点
    ros::Subscriber move_sub = n.subscribe("/wpb_home/move", 10,MoveCallback);
    // 订阅主题获取停止移动信号
    ros::Subscriber beh_sub = n.subscribe("/wpb_home/behaviors", 10,BehaviorsCallback);
    // 发布主题对机器人进行速度控制
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    sleep(1);
    ros::Rate r(30);
    while(ros::ok())
    {
        if(bMove == false)
        {
            // 处于停止移动状态
            ros::spinOnce();
            r.sleep();
            continue;
        }

        // 直接根据目标点计算速度
        vel_msg.linear.x = target_pose.pose.position.x / 4;
        vel_msg.linear.y = target_pose.pose.position.y / 4;
        vel_msg.angular.z = 0;
        
        // 对机器人速度进行一个限制，保证安全
        if( vel_msg.linear.x > 0.1)
             vel_msg.linear.x = 0.1;
        if( vel_msg.linear.x < -0.1)
             vel_msg.linear.x = -0.1;
        if( vel_msg.linear.y > 0.1)
             vel_msg.linear.y = 0.1;
        if( vel_msg.linear.y < -0.1)
             vel_msg.linear.y = -0.1;

        vel_pub.publish(vel_msg);
        //printf("移动目标 ( %.2f , %.2f )  速度= ( %.2f , %.2f )\n",target_pose.position.x,target_pose.position.y,vel_msg.linear.x,vel_msg.linear.y);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}