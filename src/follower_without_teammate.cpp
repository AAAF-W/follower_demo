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
CPathHolder path_holder;
float lidar_data[360];
geometry_msgs::PoseStamped target_pose;
bool bMove = false;
int Robot_ID = 1;

void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // 接收激光雷达数据
    for(int i=0;i<360;i++)
    {
        lidar_data[i] = scan->ranges[i];
    }
}

void TMPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    // 收到队友的坐标但是不处理（比较实验）
    // int nNumTM = msg->poses.size();
    // if(nNumTM < 2)
    //     return;
    // TeamMatePose(0, msg->poses[0].position.x, msg->poses[0].position.y);
    // TeamMatePose(1, msg->poses[1].position.x, msg->poses[1].position.y);
    //ROS_WARN("[TMPoseCallback] ( %.2f , %.2f )    ( %.2f , %.2f )",msg->poses[0].position.x,msg->poses[0].position.y,msg->poses[1].position.x,msg->poses[1].position.y);
}


void MoveCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // 收到移动目标点
    target_pose = *msg;
    bMove = true;

    // tf::Quaternion q(target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w);
    // float fYaw = tf::getYaw(q)*180/3.1415;
    // ROS_WARN("[follower_without_teammate] = ( %.2f , %.2f )  %.2f",target_pose.pose.position.x ,target_pose.pose.position.y,fYaw);
}

void BehaviorsCallback(const std_msgs::String::ConstPtr& msg)
{
    // 收到停止指令
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

// 计算朝向角
double CalDirectAngle(double inFromX, double inFromY, double inToX, double inToY)
{
    double res = 0;
    double dx = inFromX - inToX;
    double dy = -(inFromY - inToY);
    if (dx == 0)
    {
        if (dy > 0)
        {
            res = 180 - 90;
        }
        else
        {
            res = 0 - 90;
        }
    }
    else
    {
        double fTan = dy / dx;
        res = atan(fTan) * 180 / 3.1415926;

        if (dx < 0)
        {
            res = res - 180;
        }
    }
    res = 180 - res;
    if (res < 0)
    {
        res += 360;
    }
    if (res > 360)
    {
        res -= 360;
    }
    res = res*3.1415926/180;
    // 将角度值限制在(-PI,PI)范围内，这样比较好计算旋转速度 
    if(res > M_PI)
        res = res - 2*M_PI;
    return res;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "follower_without_teammate"); 

     //从yaml文件里读取参数
    ros::NodeHandle n_param("~");
    if(n_param.getParam("robot_id", Robot_ID))
    {
        // 如果robot_id参数存在，将其显示到窗体标题上
        std::ostringstream stringStream;
        stringStream << "wpb_" << Robot_ID <<"_LocalMap";
        std::string name_space = stringStream.str();
        InitHelper(name_space);
        ROS_WARN("[follower_without_teammate] with (%s) start!",name_space.c_str());
    }
    else
    {
        InitHelper("LocalMap");
    ROS_WARN("[follower_without_teammate] start!");
    }

    ros::NodeHandle n;
    ros::Subscriber lidar_sub = n.subscribe("/scan", 10, LidarCallback);
    ros::Subscriber move_sub = n.subscribe("/wpb_home/move", 10,MoveCallback);
    ros::Subscriber beh_sub = n.subscribe("/wpb_home/behaviors", 10,BehaviorsCallback);
    ros::Subscriber tm_pose_sub = n.subscribe("/team_mate_pose", 10,TMPoseCallback);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    // 初始化雷达数值
    for(int i=0;i<360;i++)
    {
        lidar_data[i] = 10.0;
    }
    
    sleep(1);
    ros::Rate r(30);
    while(ros::ok())
    {
        if(bMove == false)
        {
            ros::spinOnce();
            r.sleep();
            continue;
        }

        ClearTarget();
        // 设置移动目标点
        path_holder.SetTarget(target_pose.pose.position.x,target_pose.pose.position.y);
        //ROS_WARN("[follower_without_teammate] = ( %.2f , %.2f ) ",target_pose.pose.position.x ,target_pose.pose.position.y);
        int nNum = path_holder.arPoint.size();
        for(int i=0;i<nNum;i++)
        {
            //printf("pnt[%d] = (%.2f , %.2f)\n",i,path_holder.arPoint[i].x,path_holder.arPoint[i].y);
            float pnt_x = path_holder.arPoint[i].x;
            float pnt_y = path_holder.arPoint[i].y;
            SetTarget(pnt_y/0.05+50,pnt_x/0.05+50);
        }
        // 设置激光雷达的障碍点
        SetRanges(lidar_data);
        // 规划路径
        //ROS_WARN("outline");
        OutLine();
        // 寻找要移动的目标路径点
        bool bTargetFound = false;
        float move_x = 0;
        float move_y = 0;
        int nLocalPathNum = GetHelperNum();
        if(nLocalPathNum > 3)
        {
            move_x = GetFixX();
            move_y = GetFixY();
            bTargetFound = true;
        }
        // 避障路径里没有满足条件的路径点，就在静态路径里找
        if(bTargetFound == false)
        {
            int nNum = path_holder.arPoint.size();
            for(int i=0;i<nNum;i++)
            {
                float pnt_x = path_holder.arPoint[i].x;
                float pnt_y = path_holder.arPoint[i].y;
                double tmpDist = sqrt(pnt_x*pnt_x + pnt_y*pnt_y) ;
                if(tmpDist > 0.3)
                {
                    move_x = pnt_x;
                    move_y = pnt_y;
                    bTargetFound = true;
                    break;
                }
            }
            // 和目标点的距离小于0.3，直接移动过去
            if(bTargetFound == false)
            {
                move_x = path_holder.arPoint[nNum -1].x;
                move_y = path_holder.arPoint[nNum -1].y;
                bTargetFound = true;
            }
        }
        // else
        //     printf("避障路径点为目标点\n");
        // 根据寻找到的路径点，发送速度
        if(bTargetFound == true)
        {
            vel_msg.linear.x = move_x * 0.5;
            vel_msg.linear.y = move_y * 0.5;
        }
        else
        {
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_msg.angular.z = 0;
        }
        // 和目标点距离近到一定程度时，机器人朝向转向目标角度
        float pnt_x = target_pose.pose.position.x;
        float pnt_y = target_pose.pose.position.y;
        double finalDist = sqrt(pnt_x*pnt_x + pnt_y*pnt_y) ;
        if(finalDist < 0.5)
        {
            // 足够近了，和目标角保持一致
            tf::Quaternion quat(target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w);
            float fFinalYaw = tf::getYaw(quat);
            vel_msg.angular.z = fFinalYaw/3;
            //ROS_WARN("[fFinalYaw] = %.2f    vz= %.2f",fFinalYaw,vel_msg.angular.z);
        }
        else
        {
            // 离目标点较远，朝向目标点，以免路上的障碍物被侧柱遮挡
            float destYaw = CalDirectAngle(0,0,pnt_x,pnt_y);// 计算朝向角
            vel_msg.angular.z = destYaw/3;
            //ROS_WARN("[destYaw] = %.2f    vz= %.2f",destYaw,vel_msg.angular.z);
        }
        vel_pub.publish(vel_msg);
        //printf("移动目标 ( %.2f , %.2f )  速度= ( %.2f , %.2f )\n",move_x,move_y,vel_msg.linear.x,vel_msg.linear.y);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}