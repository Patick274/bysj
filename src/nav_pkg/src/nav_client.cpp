#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
//包含的消息包类型
//move_base_msgs/MoveBaseActionGoal     action_goal
//move_base_msgs/MoveBaseActionResult   action_result
//move_base_msgs/MoveBaseActionFeedback action_feedback

#include <actionlib/client/simple_action_client.h>

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//             /------对象类型-----/    <通讯使用的消息包类型>                 别名
//             用别名MoveBaseClient 取代

int main( int argc , char** argv)
{
    ros::init(argc,argv,"nav_client");
    //初始化
    MoveBaseClient ac("move_base",true);
    //生成一个Action客户端对象 ac=ActionClient 构造函数 （ "name" spin_thread ）
    //"move_base"是要连接的Action Server 的名字 
    // true 是让ac在与move_base通讯过程中自动阻塞等待结果

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("waitting for the move_base action sever to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id ="map";                     //坐标地图
   // goal.target_pose.header.frame_id ="base_link";               //以自己为参考坐标系进行移动
   // goal.target_pose.header.frame_id ="base_footprint";            //机器人底座足迹坐标系
    //base_link的 z 轴可能不为 0（比如机器人底盘有高度，原点在底盘中心，z=0.1 米）；
    //base_footprint强制 z=0，是 ROS 导航栈（move_base）默认的 “机器人地面参考点”

    goal.target_pose.header.stamp = ros::Time::now();

  //移动去的空间位置
    goal.target_pose.pose.position.x = -3.0;
    goal.target_pose.pose.position.y = 2.0;
  //goal.target_pose.pose.position.z =    0; (默认为0)

  //导航姿态的朝向
  //四元数
    goal.target_pose.pose.orientation.w =  1.0;


    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    //调用Action客户端对象的sendGoal(）函数 将消息包发送给move_base
    
    //等待导航结果
    
    ac.waitForResult();
    //waitForResult()会阻塞卡住，直到move_base返回导航结果
    
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
       ROS_INFO("Mission complete!");
    else
       ROS_INFO("Mission failed ...");
    
    return 0;

}