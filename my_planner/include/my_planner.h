#ifndef MY_PLANNER_H_
#define MY_PLANNER_H_

#include <ros/ros.h>   //依赖项 ros函数接口 
#include <nav_core/base_local_planner.h> //依赖项原始规划器
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>

// 定义命名空间 my_planner：用于隔离代码，避免类名/函数名冲突
namespace my_planner
{
    // 在 my_planner 命名空间下定义一个类 MyPlanner
    class MyPlanner  : public nav_core::BaseLocalPlanner
                // “： public”表示继承     nav_core::BaseLocalPlanner
    {
        // 任何属性、方法，
        public:
        //表示类对外公开的接口函数

           MyPlanner(); //构造函数，与类的名字一模一样
           //初始化类的成员变量（如设置默认值、分配内存、初始化指针为 nullptr），避免野指针、未初始化变量导致的程序崩溃。
           ~MyPlanner();//析构函数
           //放构造函数中分配的资源（如动态内存、打开的文件 / 句柄、订阅的 ROS 话题等），避免内存泄漏。
           
           //局部规划区的固定接口
           void initialize(std::string name, tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS* costmap_ros);
           //void initialize (...)：规划器的 “初始化入口”
           //这是规划器的初始化函数，在 MyPlanner 对象被创建后、开始工作前调用，负责完成所有 “运行前准备”—— 
           //比如绑定 ROS 资源、初始化参数、关联代价地图 / TF 变换等，是规划器的 “启动配置阶段”。
           //类型                            参数             作用     
           //std::string                    name             规划器的唯一名称 命名
           //tf2_ros::Buffer*                tf           机器人各坐标系（如基坐标系、激光坐标系）的转换 “字典”，规划时需要用它换算坐标（TF 变换缓冲区指针）
           //costmap_2d::Costmap2DROS*   costmap_ros      机器人感知到的 “环境地图”（哪里有障碍物、哪里可通行），是避障的核心依据（代价地图指针）
           

           bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
           //bool setPlan (...)：给规划器 “下达全局路径任务”
           //向局部规划器传入全局路径（由全局规划器如 A*、Dijkstra 计算出的 “从起点到终点的大致路线”），局部规划器需要保存这条路径，作为后续计算局部速度指令的 “导航目标”。
           //类型                                              参数             作用   
           //const std::vector<geometry_msgs::PoseStamped>&   plan    一系列 “路点” 的集合，以及每个路点包含位置（x/y/z）和姿态（四元数），是机器人要走的 “总路线”
           //


           bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
           //bool computeVelocityCommands (...)：规划器的 “核心决策函数”
           //类型                         参数             作用   
           //eometry_msgs::Twist&      cmd_vel      规划器计算出的结果，会赋值给这个变量，move_base 拿到后发给机器人底盘
           bool isGoalReached();
           //判断机器人是否已经到达全局路径的终点，move_base 会持续调用这个函数，一旦返回 true，就会停止机器人运动，结束本次导航任务。
    };
}
#endif// MY_PLANNER_H_