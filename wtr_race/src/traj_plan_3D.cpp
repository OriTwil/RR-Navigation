#include "wtr_race/wtr_am_traj.hpp"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Joy.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <string>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "astar/astar.h"
#include "wtr_mavros_msgs/wtr_posture.h"
#include "wtr_mavros_msgs/wtr_zone.h"
#include "wtr_mavros_msgs/wtr_control.h"

using namespace Eigen;
using namespace std;
using namespace ros;

typedef enum {
    First_Point,
    Second_Point,
    Third_Point,
    Fourth_Point,
    Fifth_Point,
    Sixth_Point,
    Seventh_Point,
    Eighth_Point,
    Ninth_Point,
    Tenth_Point
} CHASSIS_POINT;

std_msgs::Float32 msg;
CHASSIS_POINT Chassis_Point = First_Point;

vector<double> FireZoneA_(3);
vector<double> FireZoneB_(3);
vector<double> FireZoneC_(3);
vector<double> StartMoveZone_(3);
vector<double> PickupZoneA_(3);
vector<double> PickupZoneB_(3);
vector<double> TargetZone_(3);

class Config
{
    public:
        Config(const ros::NodeHandle &nh_priv)
        {
            nh_priv.getParam("TrajectoryTopic", trajectoryTopic);
            nh_priv.getParam("WayPointsTopic", wayPointsTopic);
            nh_priv.getParam("RouteTopic", routeTopic);
            nh_priv.getParam("AccTopic", accTopic);
            nh_priv.getParam("FrameName", frameName);

            nh_priv.getParam("WeightT", weightT);
            nh_priv.getParam("WeightAcc", weightAcc);
            nh_priv.getParam("WeightJerk", weightJerk);
            nh_priv.getParam("MaxAccRate", maxAccRate);
            nh_priv.getParam("MaxVelRate", maxVelRate);
            nh_priv.getParam("Iterations", iterations);
            nh_priv.getParam("Epsilon", epsilon);
        }
    // Advertised Topics
    std::string trajectoryTopic;
    std::string wayPointsTopic;
    std::string routeTopic;
    std::string accTopic;
    // Frame Name
    std::string frameName;
    // Params
    double weightT;
    double weightAcc;
    double weightJerk;
    double maxAccRate;
    double maxVelRate;
    int iterations;
    double epsilon;

};

class TrajPlan_3D : public Astar_planner::AstarPlannerROS
{
    public:
        TrajPlan_3D();
        void mavrosZoneCallback(const mavros_msgs::wtr_zone::ConstPtr& mavros_zone_msg);
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg);
        void ControlPub();
    private:
        ros::NodeHandle traj_nh_ ;
        ros::Subscriber pos_sub;
        
        Trajectory traj;
        std::vector<Eigen::Vector3d> wPs;

        bool protectFlag;

        ros::Publisher motion_pub;  
        mavros_msgs::wtr_control motion_msg;

        ros::Subscriber odom_sub; 
        tf2_ros::Buffer buffer; 

        //grid_map---------
        ros::Subscriber map_sub;

        ros::Timer timer;
        ros::Time start_time;

};

TrajPlan_3D::TrajPlan_3D()
{
    start_time = ros::Time::now();
    msg.data = 60;
    motion_pub = traj_nh_.advertise<mavros_msgs::wtr_control>("/mavros/speed_control/send_topic",10);
    pos_sub = traj_nh_.subscribe<mavros_msgs::wtr_zone>("/mavros/speed_control/wtr_zone",10,&TrajPlan_3D::mavrosZoneCallback,this);
    map_sub = traj_nh_.subscribe<nav_msgs::OccupancyGrid>("map", 10, &TrajPlan_3D::map_callback,this);

    traj_nh_.getParam("/traj_plan_3D/StartMoveZone",StartMoveZone_);
    traj_nh_.getParam("/traj_plan_3D/PichUpZoneA",PickupZoneA_);
    traj_nh_.getParam("/traj_plan_3D/PickUpZoneB",PickupZoneB_);
    traj_nh_.getParam("/traj_plan_3D/FireZoneA",FireZoneA_);
    traj_nh_.getParam("/traj_plan_3D/FireZoneB",FireZoneB_);
    traj_nh_.getParam("/traj_plan_3D/FireZoneC",FireZoneC_);
}

void TrajPlan_3D::ControlPub()
{
    ros::NodeHandle nh_priv("~");
    Config config(nh_priv);
    ros::Rate rate(50);
    Eigen::Vector3d iV(-0.015, -0.01, 0.0), fV(0.0, 0.0, 0.0);
    Eigen::Vector3d iA(0.0, 0.0, 0.0), fA(0.0, 0.0, 0.0); //规定航点处的速度和加速度

    AmTraj amTrajOpt(config.weightT, config.weightAcc, config.weightJerk,config.maxVelRate, config.maxAccRate, config.iterations, config.epsilon);

    traj = amTrajOpt.genOptimalTrajDTC(wPs, iV, iA, fV, fA);
    ROS_INFO("Draw trail start");
    ros::Time begin = ros::Time::now();
    while (ros::ok())
    {                
        ros::Duration time_diff = ros::Time::now() - begin;
        //实际x为0,y为1
        motion_msg.vx_set = traj.getVel(time_diff.toSec())(0);
        motion_msg.vy_set = traj.getVel(time_diff.toSec())(1);
        motion_msg.x_set =  traj.getPos(time_diff.toSec())(0);
        motion_msg.y_set =  traj.getPos(time_diff.toSec())(1);
        ROS_INFO("total time = %f",traj.getTotalDuration());

        if(time_diff.toSec()>traj.getTotalDuration() && time_diff.toSec()< traj.getTotalDuration()+0.15)
        {

            motion_msg.vx_set = 0;
            motion_msg.vy_set = 0;
            motion_msg.vw_set = 0;
        }
        else if(time_diff.toSec()> traj.getTotalDuration()+0.15)
        {
            motion_msg.vx_set = 0;
            motion_msg.vy_set = 0;
            motion_msg.vw_set = 0;
            ROS_WARN("Stop!");
            break;
        }
        motion_pub.publish(motion_msg);
        ROS_INFO("time = %f,vx = %f,vy = %f,x=%f,y=%f",time_diff.toSec(), motion_msg.vx_set, motion_msg.vy_set,motion_msg.x_set,motion_msg.y_set);
        ROS_INFO("total time = %f",traj.getTotalDuration());
        rate.sleep();
    }
}

void TrajPlan_3D::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    initialize(map_msg);
}    

void TrajPlan_3D::mavrosZoneCallback(const mavros_msgs::wtr_zone::ConstPtr& mavros_zone_msg)
{
    switch (Chassis_Point)
    {
    case First_Point:
        /* code */
        switch (mavros_zone_msg->point)
        {
            case 0:
                break;
            case 1:
                wPs.emplace_back(StartMoveZone_[0],StartMoveZone_[1],StartMoveZone_[2]);
                wPs.emplace_back(PickupZoneB_[0],PickupZoneB_[1],PickupZoneB_[2]);
                wPs.emplace_back(PickupZoneA_[0],PickupZoneA_[1],PickupZoneA_[2]);
                Chassis_Point = Second_Point;
                ControlPub();
                wPs.clear();
                break;
            case 2:
                wPs.emplace_back(StartMoveZone_[0],StartMoveZone_[1],StartMoveZone_[2]);
                wPs.emplace_back(FireZoneA_[0],FireZoneA_[1],FireZoneA_[2]);
                Chassis_Point = Third_Point;
                ControlPub();
                wPs.clear();
                break;
            case 3:
                wPs.emplace_back(StartMoveZone_[0],StartMoveZone_[1],StartMoveZone_[2]);
                wPs.emplace_back(FireZoneB_[0],FireZoneB_[1],FireZoneB_[2]);
                Chassis_Point = Fourth_Point;
                ControlPub();
                wPs.clear();
                break;
            default:
                break;
        }
        break;
    case Second_Point:
        switch (mavros_zone_msg->point)
        {
            case 0:
                wPs.emplace_back(PickupZoneA_[0],PickupZoneA_[1],PickupZoneA_[2]);
                wPs.emplace_back(PickupZoneB_[0],PickupZoneB_[1],PickupZoneB_[2]);
                wPs.emplace_back(StartMoveZone_[0],StartMoveZone_[1],StartMoveZone_[2]);
                Chassis_Point = First_Point;
                motion_msg.w_set = 0;
                ControlPub();
                wPs.clear();
                break;
            case 1:
                break;
            case 2:
                wPs.emplace_back(PickupZoneA_[0],PickupZoneA_[1],PickupZoneA_[2]);
                wPs.emplace_back(PickupZoneB_[0],PickupZoneB_[1],PickupZoneB_[2]);
                wPs.emplace_back(FireZoneA_[0],FireZoneA_[1],FireZoneA_[2]);
                Chassis_Point = Third_Point;
                motion_msg.w_set = 0;
                ControlPub();
                wPs.clear();
                break;
            case 3:
                wPs.emplace_back(PickupZoneA_[0],PickupZoneA_[1],PickupZoneA_[2]);
                wPs.emplace_back(PickupZoneB_[0],PickupZoneB_[1],PickupZoneB_[2]);
                wPs.emplace_back(FireZoneB_[0],FireZoneB_[1],FireZoneB_[2]);
                Chassis_Point = Fourth_Point;
                motion_msg.w_set = 0;
                ControlPub();
                wPs.clear();
                break;
            default:
                break;
        }
        break;
    case Third_Point:
        switch (mavros_zone_msg->point)
        {
            case 0:
                wPs.emplace_back(FireZoneA_[0],FireZoneA_[1],FireZoneA_[2]);
                wPs.emplace_back(StartMoveZone_[0],StartMoveZone_[1],StartMoveZone_[2]);
                motion_msg.w_set = 0;
                Chassis_Point = First_Point;
                ControlPub();
                wPs.clear();
                break;
            case 1:
                wPs.emplace_back(FireZoneA_[0],FireZoneA_[1],FireZoneA_[2]);
                wPs.emplace_back(PickupZoneB_[0],PickupZoneB_[1],PickupZoneB_[2]);
                wPs.emplace_back(PickupZoneA_[0],PickupZoneA_[1],PickupZoneA_[2]);
                Chassis_Point = Second_Point;
                ControlPub();
                wPs.clear();
                break;
            case 2:
                break;
            case 3:
                wPs.emplace_back(FireZoneA_[0],FireZoneA_[1],FireZoneA_[2]);
                wPs.emplace_back(FireZoneB_[0],FireZoneB_[1],FireZoneB_[2]);
                Chassis_Point = Fourth_Point;
                motion_msg.w_set = 0;
                ControlPub();
                wPs.clear();
                break;
            default:
                break;
        }
        break;
    case Fourth_Point:
        switch (mavros_zone_msg->point)
        {
            case 0:
                wPs.emplace_back(FireZoneB_[0],FireZoneB_[1],FireZoneB_[2]);
                wPs.emplace_back(StartMoveZone_[0],StartMoveZone_[1],StartMoveZone_[2]);
                Chassis_Point = First_Point;
                motion_msg.w_set = 0;
                ControlPub();
                wPs.clear();
                break;
            case 1:
                wPs.emplace_back(FireZoneB_[0],FireZoneB_[1],FireZoneB_[2]);
                wPs.emplace_back(PickupZoneB_[0],PickupZoneB_[1],PickupZoneB_[2]);
                wPs.emplace_back(PickupZoneA_[0],PickupZoneA_[1],PickupZoneA_[2]);
                Chassis_Point = Second_Point;
                ControlPub();
                wPs.clear();
                break;
            case 2:
                wPs.emplace_back(FireZoneB_[0],FireZoneB_[1],FireZoneB_[2]);
                wPs.emplace_back(FireZoneA_[0],FireZoneA_[1],FireZoneA_[2]);
                Chassis_Point = Third_Point;
                motion_msg.w_set = 0;
                ControlPub();
                wPs.clear();
                break;
            case 3:
                break;
            default:
                break;
        }
        break;
    case Fifth_Point:
        break;
    case Sixth_Point:
        break;
    case Seventh_Point:
        /* code */
        break;
    case Eighth_Point:
        break;
    case Ninth_Point:
        break;
    case Tenth_Point:
        break;
    default:
        break;
    }     
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "traj_node");
    
    TrajPlan_3D tp_3D;

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}