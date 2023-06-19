/**
 * @brief 底盘控制 发布者的实现 自定義航點
 * @file SpeedControlSet_pub2.cpp
 * @author szf
 *
 * @addtogroup src
 */

#include <ros/ros.h>
#include "mavros_msgs/SpeedControlSet_sub.h"
#include <mavros_msgs/SpeedControlStatus.h>
#include <mavros_msgs/SpeedControlSet.h>
#include <math.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <robot_pose_ekf/GetStatus.h>
#include <robot_pose_ekf/GetStatusRequest.h>
#include <stdio.h>
#include "mavros_msgs/Posture.h"
#include "amtraj/am_traj.hpp"
#include <ros/time.h>
#include <ros/duration.h>

/**
 * @brief 实例化发布者、订阅者对象
 *
 */
ros::Publisher send_publisher;
ros::Subscriber xbox_sub;
ros::Subscriber posture_sub;
ros::Subscriber traj_rviz_sub;

/**
 * @brief 创建变量
 *
 */
nav_msgs::Path path_real;                  //用来接收T265的里程计数据，便于在rviz可视化
nav_msgs::Path path_reference;             //用来在rviz中画参考路径
geometry_msgs::Quaternion orientation;     //用来接收T265的位姿数据
geometry_msgs::Quaternion orientation_ref; //暂时没用到
mavros_msgs::SpeedControlSet_sub pub;      //自定义的mavros消息
mavros_msgs::SpeedControlSet_sub ref;
bool safe = true; //用来设置急刹车
bool flag = true; //设置初始时刻
float speed[3];
float pose[3]; //用来接收T265卡尔曼滤波后的里程计数据，做闭环控制
int t = 0;
int point_count = 1;     //时间
typedef struct
{
    /* data */
    double v_cur;
    double s_cur;
    double t_sum;
} current;

std::vector<Eigen::Vector3d> wPs;//航点

/**
 * @brief callbacks
 *
 * @param xbox
 */

void xboxCallback(const geometry_msgs::Twist::ConstPtr &xbox)
{
    if (xbox->angular.z != 0 || speed[0] > 1 || speed[1] > 1 || speed[2] > 1)
    {
        safe = false;
    }
    else if (xbox->angular.z == 0)
    {
        safe = true;
    }
}

void Poseture_Callback(const mavros_msgs::PostureConstPtr &poseture)
{
    pose[0] = poseture->pos_x_state;
    pose[1] = poseture->pos_y_state;
    orientation.z = poseture->zangle_state;
}

/**
 * @brief 主函数
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    //初始化节点
    ros::init(argc, argv, "SpeedControlPublisher");

    //创建ROS句柄
    ros::NodeHandle speed_control_nh;

    //创建发布者
    send_publisher = speed_control_nh.advertise<mavros_msgs::SpeedControlSet_sub>("/mavros/speed_control/send_topic", 1000);

    //订阅xbox
    xbox_sub = speed_control_nh.subscribe<geometry_msgs::Twist>("xbox", 10, xboxCallback);
    //订阅码盘
    posture_sub = speed_control_nh.subscribe<mavros_msgs::Posture>("/mavros/posture/posture", 10, Poseture_Callback);

    //初始化被发布消息
    pub.vw_set_sub = 0;
    pub.vx_set_sub = 0;
    pub.vy_set_sub = 0;
    pub.x_set_sub = 0;
    pub.y_set_sub = 0;

    //初始化参考消息
    ref.vx_set_sub = 0;
    ref.vy_set_sub = 0;
    ref.vw_set_sub = 0;
    ref.x_set_sub = 0;
    ref.y_set_sub = 0;

    //traj自定义航点轨迹规划

    //创建轨迹生成对象
    wPs.emplace_back(0.0,0.0,0.0);
    wPs.emplace_back(1.0,0.3,0.0);
    wPs.emplace_back(2.0,0.6,0.0);
    wPs.emplace_back(4.0,0.6,0.0);
    wPs.emplace_back(5.0,0.6,0.0);
    wPs.emplace_back(6.0,0.0,0.0);

    AmTraj amTrajOpt(1024.0, 32.0, 1.0, 3, 1.5, 32, 0.02);
    Eigen::Vector3d iV(-0.015, -0.01, 0.0), fV(0.0, 0.0, 0.0);
    Eigen::Vector3d iA(0.0, 0.0, 0.0), fA(0.0, 0.0, 0.0); //规定航点处的速度和加速度

    int time_count = 0;
    Trajectory traj = amTrajOpt.genOptimalTrajDTC(wPs, iV, iA, fV, fA); //生成轨迹
    ros::Time begin = ros::Time::now();

    ROS_WARN("total duration:%lf", traj.getTotalDuration());

    ros::Rate r(50);//Hz
    while (ros::ok)
    {
        if(flag == true)
        {
            begin = ros::Time::now();//初始时刻
            flag = false;
        }

        ros::Duration time = ros::Time::now() - begin;//时间

        pub.vw_set_sub = 0;
        pub.vx_set_sub = traj.getVel(time.toSec())(0);
        pub.vy_set_sub = traj.getVel(time.toSec())(1);
        pub.x_set_sub = traj.getPos(time.toSec())(0);
        pub.y_set_sub = traj.getPos(time.toSec())(1);
/*         pub.vx_set_sub = 0.5;
        pub.vy_set_sub = 0;
        pub.x_set_sub = 0;
        pub.y_set_sub = 0; */

        ROS_WARN("time = %lf, x = %lf ,y = %lf",time.toSec(),pub.x_set_sub,pub.y_set_sub);
        ROS_INFO("vx = %lf , vy = %lf",pub.vx_set_sub,pub.vy_set_sub);
        send_publisher.publish(pub);
        
        if (time.toSec() > traj.getTotalDuration() && time.toSec() < traj.getTotalDuration() + 0.15 )
        {
            pub.vw_set_sub = 0;
            pub.vx_set_sub = 0;
            pub.vy_set_sub = 0;

            send_publisher.publish(pub);
            ROS_INFO("stop!!vx_set = %lf",pub.vx_set_sub);
        }
        else if(time.toSec() > traj.getTotalDuration() && time.toSec() + 0.15)
        {
            pub.vw_set_sub = 0;
            pub.vx_set_sub = 0;
            pub.vy_set_sub = 0;

            send_publisher.publish(pub);
            ROS_INFO("stop!!vx_set = %lf",pub.vx_set_sub);
            break;
        }

        r.sleep();
    }

    return 0;
}