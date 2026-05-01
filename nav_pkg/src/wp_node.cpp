#include <ros/ros.h>         // ROS核心头文件，包含节点初始化、发布/订阅等基础功能
#include <std_msgs/String.h> // ROS标准字符串消息类型头文件，用于定义字符串消息
#include <string>
#include <vector>

// ==================== 全局变量 ====================
ros::Publisher nav_pub;           // 发布者需要全局访问，或在回调中使用
std::vector<std::string> waypoints = {"1", "2", "3", "4", "5"};  // 航点列表
int current_waypoint_index = 0;  // 当前航点索引
bool waiting_for_result = false; // 标志位：是否正在等待导航结果


void NavResultCallback(const std_msgs::String &msg)//回调函数
//回调函数必须符合这个格式：
// void 函数名(const 消息类型 &msg);
{
    ROS_WARN(" [Received navi_waypoint = %s ]",msg.data.c_str());
    //msg.data.c_str()：将 C++ 的std::string转换为 C 风格字符串，适配ROS_WARN的格式化输出；

    std::string result = msg.data;
    
    // 如果导航成功完成，发送下一个航点
    if (result == "done" && waiting_for_result)
      
    {
        current_waypoint_index++;  // 移动到下一个航点
        
        if (current_waypoint_index < waypoints.size())
        {
            // 发送下一个航点
            std_msgs::String nav_msg;
            nav_msg.data = waypoints[current_waypoint_index];
            nav_pub.publish(nav_msg);
            ROS_INFO("Sending waypoint: %s", nav_msg.data.c_str());
            waiting_for_result = true;
        }
        else
        {
            ROS_INFO("All waypoints completed!");
            waiting_for_result = false;
        }
    }

}


int main(int argc ,char** argv)
{
    //初始化ros节点
    ros::init(argc,argv,"wp_node");
    //ros大管家NodeHandle n
    ros::NodeHandle n;
    
    //建立发布者
    /*ros::Publisher*/  
    nav_pub =  n.advertise<std_msgs::String>("/waterplus/navi_waypoint", 10);//目标航点名称的话题发布
    //去掉局部定义 
    //建立订阅者
    ros::Subscriber res_sub =  n.subscribe("/waterplus/navi_result",10,NavResultCallback);//订阅

    //语法 ros::Publisher  xxx_pub = n.advertise<std_msgs::String>("话题名字"，缓冲区大小)；
    //语法 ros::Subscriber xxx_sub = n.subscriber<"话题名字"，缓冲区大小，回调函数>；


    //休眠1s，确保发布者与ros主节点完成连接     
    //sleep(1);
    ros::Duration(1.0).sleep();// 推荐用ROS的Duration，而非sleep（更兼容ROS时间）

    // 检查航点列表非空
    if (waypoints.empty())
    {
        ROS_ERROR("航点列表为空！");
        return 1;
    }

    std_msgs::String nav_msg;
    nav_msg.data = "1";
    nav_pub.publish(nav_msg);
    ROS_INFO("start navigation,first waypoint %s",nav_msg.data.c_str());
    waiting_for_result = true;
    ros::spin();
    // 构造并发布导航指令消息
    // std_msgs::String nav_msg;   //创建字符串消息对象
    // nav_msg.data= "1";          //消息类型内容为“1”
    // nav_pub.publish (nav_msg);  //发布消息到/waterplus/navi_waypoint话题
    // 语法： xxx_pub.publish(消息对象)；

   // ros::Rate rate(10);
   // while(ros::ok())
   // {
   //     ros::spinOnce();
   //     rate.sleep();
   // }

    return 0;

}
