#include "my_planner.h"
#include <pluginlib/class_list_macros.h>
// #include <opencv2/higui/highgui.hpp>


PLUGINLIB_EXPORT_CLASS( my_planner::MyPlanner ,nav_core::BaseLocalPlanner)
namespace my_planner
{
           MyPlanner:: MyPlanner()
           {    
            setlocale(LC_ALL,"");//将字符串编码本地化，目的是显示中文
           }

           MyPlanner::~MyPlanner()
           {

           }

           void  MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS* costmap_ros)
           {
            ROS_WARN("导航运行中！");
           }

           bool  MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
           {
                 return true;
           }

           bool  MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
           {
                 return false;
           }

           bool  MyPlanner::isGoalReached()
           {
                 return false;
           }
} 