#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>
#include <mutex>      // 【新增】包含互斥锁头文件
#include <algorithm>  // 【新增】包含 std::min 和 std::max 用于限幅

serial::Serial ser;
std::mutex serial_mutex; // 【安全锁 1】定义全局串口互斥锁

// ==========================================
// 导航下行速度回调函数
// ==========================================
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // 【安全锁 2】下行速度限幅 (Hard Limit)
    // 即使 move_base 规划出错，或者键盘遥控乱按，底层也能兜底
    // 假设你的小车物理极限是: 线速度 0.8m/s, 角速度 2.0rad/s
    const double MAX_LINEAR_VEL = 0.8; 
    const double MAX_ANGULAR_VEL = 2.0;

    double vx = std::max(-MAX_LINEAR_VEL, std::min(MAX_LINEAR_VEL, msg->linear.x));
    double vy = std::max(-MAX_LINEAR_VEL, std::min(MAX_LINEAR_VEL, msg->linear.y));
    double vth = std::max(-MAX_ANGULAR_VEL, std::min(MAX_ANGULAR_VEL, msg->angular.z));

    // 转换为 STM32 协议单位 (假设放大 1000 倍转化为 mm/s)
    int16_t send_vx = (int16_t)(vx * 1000);
    int16_t send_vy = (int16_t)(vy * 1000);
    int16_t send_vth = (int16_t)(vth * 1000);

    // 打包下行协议 (9字节: 帧头2 + Vx2 + Vy2 + Vth2 + 校验1)
    uint8_t send_buf[9];
    send_buf[0] = 0xAA; 
    send_buf[1] = 0xAA;
    send_buf[2] = (send_vx >> 8) & 0xFF;
    send_buf[3] = send_vx & 0xFF;
    send_buf[4] = (send_vy >> 8) & 0xFF;
    send_buf[5] = send_vy & 0xFF;
    send_buf[6] = (send_vth >> 8) & 0xFF;
    send_buf[7] = send_vth & 0xFF;

    uint8_t checksum = 0;
    for(int i=0; i<8; i++) checksum += send_buf[i];
    send_buf[8] = checksum;

    // 【安全锁 1】多线程并发写保护
    // 确保在往串口写数据时，主循环的读取操作或其他回调不会抢占资源
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        if(ser.isOpen()){
            ser.write(send_buf, 9);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "base_controller_nav");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 串口配置参数读取
    std::string port;
    private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
    int baudrate;
    private_nh.param<int>("baudrate", baudrate, 115200);

    try {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR("Unable to open port: %s", port.c_str());
        return -1;
    }

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 100);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    tf2_ros::TransformBroadcaster odom_broadcaster;

    ros::Subscriber cmd_sub = nh.subscribe("/cmd_vel", 50, cmdVelCallback);

    double x = 0.0, y = 0.0, th = 0.0;
    ros::Time current_time, last_time;
    bool is_first = true;
    std::vector<uint8_t> data_buffer;
    ros::Rate loop_rate(200);

    while(ros::ok()) {
        // 【安全锁 1】读串口时也加锁，防止与写入冲突
        size_t n = 0;
        {
            std::lock_guard<std::mutex> lock(serial_mutex);
            if(ser.available()) {
                n = ser.available();
                std::vector<uint8_t> temp_buf(n);
                ser.read(temp_buf.data(), n);
                data_buffer.insert(data_buffer.end(), temp_buf.begin(), temp_buf.end());
            }
        }

        if (data_buffer.size() > 1024) data_buffer.clear(); 

        while(data_buffer.size() >= 21) { 
            if (data_buffer[0] != 0x5A || data_buffer[1] != 0x5A) {
                data_buffer.erase(data_buffer.begin());
                continue;
            }
            uint8_t ck = 0;
            for(int i=0; i<20; i++) ck += data_buffer[i];
            if(ck != data_buffer[20]) {
                data_buffer.erase(data_buffer.begin());
                continue;
            }

            current_time = ros::Time::now();
            if(is_first) { last_time = current_time; is_first = false; }
            
            // ... (解析 IMU 和 Odom，代码保持不变) ...

            data_buffer.erase(data_buffer.begin(), data_buffer.begin() + 21);
            last_time = current_time;
        }
        
        ros::spinOnce(); 
        loop_rate.sleep();
    }
    return 0;
}