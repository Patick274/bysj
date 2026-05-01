#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <serial/serial.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

serial::Serial ser;

int main(int argc, char** argv) {
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

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
    ROS_INFO("Serial port opened successfully: %s", port.c_str());

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 100);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    tf2_ros::TransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    
    ros::Time current_time, last_time;
    // 【修复 3】初始时间戳设置：引入标志位，确保第一次 dt 计算准确
    bool is_first_packet = true; 

    std::vector<uint8_t> data_buffer;
    ros::Rate loop_rate(200);

    // 【修复 1】定义完整的 IMU 协方差矩阵 (9维)
    // 如果没有姿态解算，方向协方差第一项设为 -1 代表忽略，或赋予合理值
    boost::array<double, 9> imu_covariance = {
        0.001, 0.0,   0.0,
        0.0,   0.001, 0.0,
        0.0,   0.0,   0.001
    };

    // 【修复 2】定义完整的 Odom 协方差矩阵 (36维)
    // 假设是 2D 移动底盘，x, y, yaw 赋予较小协方差，z, roll, pitch 赋予极大协方差(1e6)代表不信任
    boost::array<double, 36> odom_covariance = {
        1e-3, 0, 0, 0, 0, 0,
        0, 1e-3, 0, 0, 0, 0,
        0, 0, 1e6, 0, 0, 0,
        0, 0, 0, 1e6, 0, 0,
        0, 0, 0, 0, 1e6, 0,
        0, 0, 0, 0, 0, 1e-3
    };

    while(ros::ok()) {
        if(ser.available()) {
            size_t n = ser.available();
            std::vector<uint8_t> temp_buf(n);
            ser.read(temp_buf.data(), n);
            data_buffer.insert(data_buffer.end(), temp_buf.begin(), temp_buf.end());

            // 【修复 4】防止长期运行导致缓冲区无限溢出导致内存爆炸/卡死
            if (data_buffer.size() > 1024) {
                ROS_WARN("Serial buffer overflow! Clearing buffer...");
                data_buffer.clear();
                continue;
            }

            // 假设新协议为 21 字节: 帧头(2) + Acc(6) + Gyro(6) + Vel(6) + 校验(1)
            while(data_buffer.size() >= 21) { 
                if (data_buffer[0] != 0x5A || data_buffer[1] != 0x5A) {
                    data_buffer.erase(data_buffer.begin());
                    continue;
                }

                uint8_t checksum = 0;
                for(int i=0; i<20; i++) checksum += data_buffer[i];
                if(checksum != data_buffer[20]) {
                    data_buffer.erase(data_buffer.begin());
                    continue;
                }

                // 成功抓取有效数据包，更新当前时间
                current_time = ros::Time::now();

                // 【修复 3】精准控制初始时间，防止第一次位置跳变
                if (is_first_packet) {
                    last_time = current_time;
                    is_first_packet = false;
                }

                // ==========================================
                // 第一部分：解析并发布 IMU 数据
                // ==========================================
                int16_t ax = (int16_t)(data_buffer[2] << 8 | data_buffer[3]);
                int16_t ay = (int16_t)(data_buffer[4] << 8 | data_buffer[5]);
                int16_t az = (int16_t)(data_buffer[6] << 8 | data_buffer[7]);

                int16_t gx = (int16_t)(data_buffer[8] << 8 | data_buffer[9]);
                int16_t gy = (int16_t)(data_buffer[10] << 8 | data_buffer[11]);
                int16_t gz = (int16_t)(data_buffer[12] << 8 | data_buffer[13]);

                float acc_coef = 0.01f;   
                float gyr_coef = 0.001f;  

                sensor_msgs::Imu imu_msg;
                imu_msg.header.stamp = current_time;
                imu_msg.header.frame_id = "imu_link";
                imu_msg.angular_velocity.x = gx * gyr_coef;
                imu_msg.angular_velocity.y = gy * gyr_coef;
                imu_msg.angular_velocity.z = gz * gyr_coef;
                imu_msg.linear_acceleration.x = ax * acc_coef;
                imu_msg.linear_acceleration.y = ay * acc_coef;
                imu_msg.linear_acceleration.z = az * acc_coef;

                // 【修复 1】赋予完整的 IMU 协方差矩阵
                imu_msg.orientation_covariance = imu_covariance;
                imu_msg.angular_velocity_covariance = imu_covariance;
                imu_msg.linear_acceleration_covariance = imu_covariance;

                tf2::Quaternion q_imu;
                q_imu.setRPY(0, 0, 0); 
                imu_msg.orientation = tf2::toMsg(q_imu);
                
                imu_pub.publish(imu_msg);

                // ==========================================
                // 第二部分：解析并推算 Odom 航迹
                // ==========================================
                int16_t raw_vx = (int16_t)(data_buffer[14] << 8 | data_buffer[15]);
                int16_t raw_vy = (int16_t)(data_buffer[16] << 8 | data_buffer[17]);
                int16_t raw_vth = (int16_t)(data_buffer[18] << 8 | data_buffer[19]);

                double vx = raw_vx * 0.001; 
                double vy = raw_vy * 0.001; 
                double vth = raw_vth * 0.001; 

                double dt = (current_time - last_time).toSec();
                
                // 航迹推算 (如果 dt 极小或异常大，予以忽略防飞点)
                if (dt > 0.0 && dt < 1.0) {
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

                // ==========================================
                // 第三部分：发布 TF 变换
                // ==========================================
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_link"; 

                odom_trans.transform.translation.x = x;
                odom_trans.transform.translation.y = y;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = quat_msg;

                odom_broadcaster.sendTransform(odom_trans);

                // ==========================================
                // 第四部分：发布 /odom 话题
                // ==========================================
                nav_msgs::Odometry odom;
                odom.header.stamp = current_time;
                odom.header.frame_id = "odom";
                odom.child_frame_id = "base_link";

                odom.pose.pose.position.x = x;
                odom.pose.pose.position.y = y;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = quat_msg;
                // 【修复 2】赋予完整的 Odom 位置协方差
                odom.pose.covariance = odom_covariance;

                odom.twist.twist.linear.x = vx;
                odom.twist.twist.linear.y = vy;
                odom.twist.twist.angular.z = vth;
                // 【修复 2】赋予完整的 Odom 速度协方差
                odom.twist.covariance = odom_covariance;

                odom_pub.publish(odom);

                // ==========================================
                // 更新与清理
                // ==========================================
                last_time = current_time;
                data_buffer.erase(data_buffer.begin(), data_buffer.begin() + 21);
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    ser.close();
    return 0;
}