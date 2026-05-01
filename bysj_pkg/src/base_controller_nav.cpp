#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>
#include <mutex>
#include <algorithm>

// ==========================================
// 全局串口与线程锁
// ==========================================
serial::Serial ser;
std::mutex serial_mutex;

// ==========================================
// 物理单位转换系数（与下位机固件对齐）
// ==========================================
const float ACC_COEF = 0.01f;   // 加速度：原始值 × 0.01 = m/s²
const float GYR_COEF = 0.001f;  // 角速度：原始值 × 0.001 = rad/s
const float VEL_COEF = 0.001f;  // 轮速：原始值 × 0.001 = m/s 或 rad/s

// ==========================================
// 速度限幅常量（兜底保护）
// ==========================================
const double MAX_LINEAR_VEL = 0.8;   // m/s
const double MAX_ANGULAR_VEL = 2.0;  // rad/s

// ==========================================
// 协方差矩阵常量
// ==========================================
const boost::array<double, 9> IMU_COVARIANCE = {{
    0.001, 0.0,   0.0,
    0.0,   0.001, 0.0,
    0.0,   0.0,   0.001
}};

// orientation 不可用标记（ROS 规范：首元素 -1）
const boost::array<double, 9> ORIENTATION_COV_UNKNOWN = {{
    -1.0, 0, 0,
    0, 0, 0,
    0, 0, 0
}};

// Odom pose：x,y 可信；z,roll,pitch 不可信（1e6）；yaw 可信度降低（轮速积分漂移）
const boost::array<double, 36> ODOM_POSE_COVARIANCE = {{
    1e-3, 0, 0, 0, 0, 0,
    0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-2
}};

// Odom twist：速度可信度比 pose 略低（轮速噪声/打滑）
const boost::array<double, 36> ODOM_TWIST_COVARIANCE = {{
    1e-2, 0, 0, 0, 0, 0,
    0, 1e-2, 0, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-2
}};

// ==========================================
// 导航下行速度回调（/cmd_vel → STM32）
// ==========================================
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // 速度硬限幅（防止上层规划器或键盘遥控发出超限指令）
    double vx = std::max(-MAX_LINEAR_VEL, std::min(MAX_LINEAR_VEL, msg->linear.x));
    double vy = std::max(-MAX_LINEAR_VEL, std::min(MAX_LINEAR_VEL, msg->linear.y));
    double vth = std::max(-MAX_ANGULAR_VEL, std::min(MAX_ANGULAR_VEL, msg->angular.z));

    // 转换为 STM32 协议单位（放大 1000 倍，下位机收到后需 /1000 还原）
    int16_t send_vx = static_cast<int16_t>(vx * 1000);
    int16_t send_vy = static_cast<int16_t>(vy * 1000);
    int16_t send_vth = static_cast<int16_t>(vth * 1000);

    // 打包下行协议：帧头(2) + Vx(2) + Vy(2) + Vth(2) + 校验(1) = 9字节
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
    for(int i = 0; i < 8; i++) checksum += send_buf[i];
    send_buf[8] = checksum;

    std::lock_guard<std::mutex> lock(serial_mutex);
    if(ser.isOpen()) {
        ser.write(send_buf, 9);
    }
}

// ==========================================
// 主函数
// ==========================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "base_controller_nav");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // -------------------- 参数读取 --------------------
    std::string port;
    private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
    int baudrate;
    private_nh.param<int>("baudrate", baudrate, 115200);

    // 【核心】实验模式控制参数：通过 launch 文件切换实验 1/2/3
    bool publish_imu;
    private_nh.param<bool>("publish_imu", publish_imu, true);
    bool publish_odom;
    private_nh.param<bool>("publish_odom", publish_odom, true);

    std::string frame_id_imu;
    private_nh.param<std::string>("frame_id_imu", frame_id_imu, "imu_link");
    std::string frame_id_odom;
    private_nh.param<std::string>("frame_id_odom", frame_id_odom, "odom");
    std::string child_frame_id;
    private_nh.param<std::string>("child_frame_id", child_frame_id, "base_link");

    // -------------------- 串口初始化 --------------------
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
    ROS_INFO("Serial port opened: %s", port.c_str());
    ROS_INFO("Experiment mode: publish_imu=%s, publish_odom=%s",
             publish_imu ? "true" : "false",
             publish_odom ? "true" : "false");

    // -------------------- ROS 发布器与订阅器 --------------------
    ros::Publisher imu_pub;
    ros::Publisher odom_pub;
    tf2_ros::TransformBroadcaster odom_broadcaster;

    if(publish_imu) {
        imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 100);
    }
    if(publish_odom) {
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    }

    ros::Subscriber cmd_sub = nh.subscribe("/cmd_vel", 50, cmdVelCallback);

    // -------------------- Odom 状态变量 --------------------
    double x = 0.0, y = 0.0, th = 0.0;
    ros::Time current_time, last_time;
    bool is_first = true;
    std::vector<uint8_t> data_buffer;
    ros::Rate loop_rate(200);

    // ==========================================
    // 主循环
    // ==========================================
    while(ros::ok()) {
        // -------------------- 串口读取（加锁） --------------------
        size_t n = 0;
        std::vector<uint8_t> temp_buf;
        {
            std::lock_guard<std::mutex> lock(serial_mutex);
            if(ser.available()) {
                n = ser.available();
                temp_buf.resize(n);
                ser.read(temp_buf.data(), n);
            }
        }
        if(n > 0) {
            data_buffer.insert(data_buffer.end(), temp_buf.begin(), temp_buf.end());
        }

        // -------------------- 缓冲区溢出保护 --------------------
        if(data_buffer.size() > 1024) {
            ROS_WARN("Buffer overflow (%zu bytes), recovering to next frame head...", data_buffer.size());
            bool found = false;
            for(size_t i = 1; i + 1 < data_buffer.size(); ++i) {
                if(data_buffer[i] == 0x5A && data_buffer[i+1] == 0x5A) {
                    data_buffer.erase(data_buffer.begin(), data_buffer.begin() + i);
                    found = true;
                    break;
                }
            }
            if(!found) data_buffer.clear();
        }

        // -------------------- 解析上行协议（21 字节固定帧） --------------------
        // 帧头(2) + Acc(6) + Gyro(6) + Vel(6) + 校验(1)
        while(data_buffer.size() >= 21) {
            if(data_buffer[0] != 0x5A || data_buffer[1] != 0x5A) {
                data_buffer.erase(data_buffer.begin());
                continue;
            }

            uint8_t checksum = 0;
            for(int i = 0; i < 20; i++) checksum += data_buffer[i];
            if(checksum != data_buffer[20]) {
                data_buffer.erase(data_buffer.begin());
                continue;
            }

            // 【时间戳】每帧独立取 stamp，保证该帧 IMU 与 Odom 严格同步
            current_time = ros::Time::now();

            // -------------------- 解析 IMU（字节 2~13）--------------------
            int16_t ax = static_cast<int16_t>((data_buffer[2]  << 8) | data_buffer[3]);
            int16_t ay = static_cast<int16_t>((data_buffer[4]  << 8) | data_buffer[5]);
            int16_t az = static_cast<int16_t>((data_buffer[6]  << 8) | data_buffer[7]);
            int16_t gx = static_cast<int16_t>((data_buffer[8]  << 8) | data_buffer[9]);
            int16_t gy = static_cast<int16_t>((data_buffer[10] << 8) | data_buffer[11]);
            int16_t gz = static_cast<int16_t>((data_buffer[12] << 8) | data_buffer[13]);

            // -------------------- 解析轮速（字节 14~19）--------------------
            int16_t raw_vx  = static_cast<int16_t>((data_buffer[14] << 8) | data_buffer[15]);
            int16_t raw_vy  = static_cast<int16_t>((data_buffer[16] << 8) | data_buffer[17]);
            int16_t raw_vth = static_cast<int16_t>((data_buffer[18] << 8) | data_buffer[19]);

            // -------------------- 发布 IMU（实验 2/3） --------------------
            if(publish_imu) {
                sensor_msgs::Imu imu_msg;
                imu_msg.header.stamp = current_time;
                imu_msg.header.frame_id = frame_id_imu;

                imu_msg.linear_acceleration.x = ax * ACC_COEF;
                imu_msg.linear_acceleration.y = ay * ACC_COEF;
                imu_msg.linear_acceleration.z = az * ACC_COEF;

                imu_msg.angular_velocity.x = gx * GYR_COEF;
                imu_msg.angular_velocity.y = gy * GYR_COEF;
                imu_msg.angular_velocity.z = gz * GYR_COEF;

                tf2::Quaternion q;
                q.setRPY(0, 0, 0);
                imu_msg.orientation = tf2::toMsg(q);
                imu_msg.orientation_covariance = ORIENTATION_COV_UNKNOWN;

                imu_msg.angular_velocity_covariance = IMU_COVARIANCE;
                imu_msg.linear_acceleration_covariance = IMU_COVARIANCE;

                imu_pub.publish(imu_msg);
            }

            // -------------------- 发布 Odom + TF（仅实验 3） --------------------
            if(publish_odom) {
                if(is_first) {
                    last_time = current_time;
                    is_first = false;
                }

                double vx  = raw_vx  * VEL_COEF;
                double vy  = raw_vy  * VEL_COEF;
                double vth = raw_vth * VEL_COEF;

                double dt = (current_time - last_time).toSec();

                // dt 保护：防止串口卡顿或时间回退导致位置飞点
                if(dt > 0.0 && dt < 1.0) {
                    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
                    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
                    double delta_th = vth * dt;

                    x += delta_x;
                    y += delta_y;
                    th += delta_th;
                }

                tf2::Quaternion odom_quat;
                odom_quat.setRPY(0, 0, th);
                geometry_msgs::Quaternion quat_msg = tf2::toMsg(odom_quat);

                // TF: odom -> base_link
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = frame_id_odom;
                odom_trans.child_frame_id = child_frame_id;
                odom_trans.transform.translation.x = x;
                odom_trans.transform.translation.y = y;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = quat_msg;
                odom_broadcaster.sendTransform(odom_trans);

                // /odom 话题
                nav_msgs::Odometry odom;
                odom.header.stamp = current_time;
                odom.header.frame_id = frame_id_odom;
                odom.child_frame_id = child_frame_id;

                odom.pose.pose.position.x = x;
                odom.pose.pose.position.y = y;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = quat_msg;
                odom.pose.covariance = ODOM_POSE_COVARIANCE;

                odom.twist.twist.linear.x = vx;
                odom.twist.twist.linear.y = vy;
                odom.twist.twist.angular.z = vth;
                odom.twist.covariance = ODOM_TWIST_COVARIANCE;

                odom_pub.publish(odom);

                last_time = current_time;
            }

            // 移除已处理的 21 字节
            data_buffer.erase(data_buffer.begin(), data_buffer.begin() + 21);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    // 关闭串口
    std::lock_guard<std::mutex> lock(serial_mutex);
    if(ser.isOpen()) ser.close();
    return 0;
}