#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <serial/serial.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>

serial::Serial ser;

int main(int argc, char** argv) {
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // ==================== 参数读取 ====================
    std::string port;
    private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
    int baudrate;
    private_nh.param<int>("baudrate", baudrate, 115200);

    // ==================== 串口初始化 ====================
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

    // ==================== 实验3：发布 IMU + Odom + TF ====================
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 100);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    tf2_ros::TransformBroadcaster odom_broadcaster;

    // ==================== Odom 状态变量 ====================
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    ros::Time current_time, last_time;
    bool is_first_packet = true;

    std::vector<uint8_t> data_buffer;
    ros::Rate loop_rate(200);

    // ==================== 协方差矩阵 ====================
    boost::array<double, 9> imu_covariance = {
        0.001, 0.0,   0.0,
        0.0,   0.001, 0.0,
        0.0,   0.0,   0.001
    };

    // Odom pose：x,y,yaw可信；z,roll,pitch不可信（1e6）
    boost::array<double, 36> odom_pose_covariance = {
        1e-3, 0, 0, 0, 0, 0,
        0, 1e-3, 0, 0, 0, 0,
        0, 0, 1e6, 0, 0, 0,
        0, 0, 0, 1e6, 0, 0,
        0, 0, 0, 0, 1e6, 0,
        0, 0, 0, 0, 0, 1e-3
    };

    // Odom twist：速度可信度比pose略低（轮速噪声/打滑）
    boost::array<double, 36> odom_twist_covariance = {
        1e-2, 0, 0, 0, 0, 0,
        0, 1e-2, 0, 0, 0, 0,
        0, 0, 1e6, 0, 0, 0,
        0, 0, 0, 1e6, 0, 0,
        0, 0, 0, 0, 1e6, 0,
        0, 0, 0, 0, 0, 1e-2
    };
    //======================系数======================
                float acc_coef = 0.01f;
                float gyr_coef = 0.001f;
                float vel_coef = 0.001f;
    // ==================== 主循环 ====================
    while(ros::ok()) {
        if(ser.available()) {
            size_t n = ser.available();
            std::vector<uint8_t> temp_buf(n);
            ser.read(temp_buf.data(), n);
            data_buffer.insert(data_buffer.end(), temp_buf.begin(), temp_buf.end());

            // 缓冲区溢出保护
            if (data_buffer.size() > 1024) {
                ROS_WARN("Serial buffer overflow! Clearing buffer...");
                data_buffer.clear();
                continue;
            }

            // ==================== 21 字节固定帧解析 ====================
            // 帧头(2) + Acc(6) + Gyro(6) + Vel(6) + 校验(1)
            while(data_buffer.size() >= 21) {
                // 帧头检查
                if (data_buffer[0] != 0x5A || data_buffer[1] != 0x5A) {
                    data_buffer.erase(data_buffer.begin());
                    continue;
                }

                // 校验和：前20字节累加和 == 第21字节
                uint8_t checksum = 0;
                for(int i = 0; i < 20; i++) checksum += data_buffer[i];
                if(checksum != data_buffer[20]) {
                    data_buffer.erase(data_buffer.begin());
                    continue;
                }

                // -------------------- 解析 IMU（字节 2~13）--------------------
                int16_t ax = (int16_t)(data_buffer[2]  << 8 | data_buffer[3]);
                int16_t ay = (int16_t)(data_buffer[4]  << 8 | data_buffer[5]);
                int16_t az = (int16_t)(data_buffer[6]  << 8 | data_buffer[7]);
                int16_t gx = (int16_t)(data_buffer[8]  << 8 | data_buffer[9]);
                int16_t gy = (int16_t)(data_buffer[10] << 8 | data_buffer[11]);
                int16_t gz = (int16_t)(data_buffer[12] << 8 | data_buffer[13]);

                // -------------------- 解析轮速（字节 14~19）--------------------
                int16_t raw_vx  = (int16_t)(data_buffer[14] << 8 | data_buffer[15]);//前后
                int16_t raw_vy  = (int16_t)(data_buffer[16] << 8 | data_buffer[17]);//左右
                int16_t raw_vth = (int16_t)(data_buffer[18] << 8 | data_buffer[19]);//旋转

                // -------------------- 发布 IMU 消息 --------------------
                sensor_msgs::Imu imu_msg;
                imu_msg.header.stamp = ros::Time::now();
                imu_msg.header.frame_id = "imu_link";
                //加速度
                imu_msg.linear_acceleration.x = ax * acc_coef;
                imu_msg.linear_acceleration.y = ay * acc_coef;
                imu_msg.linear_acceleration.z = az * acc_coef;
                //陀螺仪
                imu_msg.angular_velocity.x = gx * gyr_coef;
                imu_msg.angular_velocity.y = gy * gyr_coef;
                imu_msg.angular_velocity.z = gz * gyr_coef;

                tf2::Quaternion q_imu;
                q_imu.setRPY(0, 0, 0);
                imu_msg.orientation = tf2::toMsg(q_imu);
                imu_msg.orientation_covariance[0] = -1.0;

                imu_msg.angular_velocity_covariance = imu_covariance;
                imu_msg.linear_acceleration_covariance = imu_covariance;

                imu_pub.publish(imu_msg);

                // -------------------- Odom 航迹推算 --------------------
                current_time = ros::Time::now();

                if (is_first_packet) {
                    last_time = current_time;
                    is_first_packet = false;
                }

                double vx  = raw_vx  * vel_coef;
                double vy  = raw_vy  * vel_coef;
                double vth = raw_vth * vel_coef;

                double dt = (current_time - last_time).toSec();

                // dt 保护：防止串口卡顿或时间回退导致位置飞点
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

                // -------------------- 发布 TF: odom -> base_link --------------------
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_link";

                odom_trans.transform.translation.x = x;
                odom_trans.transform.translation.y = y;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = quat_msg;

                odom_broadcaster.sendTransform(odom_trans);

                // -------------------- 发布 /odom 话题 --------------------
                nav_msgs::Odometry odom;
                odom.header.stamp = current_time;
                odom.header.frame_id = "odom";
                odom.child_frame_id = "base_link";

                odom.pose.pose.position.x = x;
                odom.pose.pose.position.y = y;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = quat_msg;
                odom.pose.covariance = odom_pose_covariance;

                odom.twist.twist.linear.x = vx;
                odom.twist.twist.linear.y = vy;
                odom.twist.twist.angular.z = vth;
                odom.twist.covariance = odom_twist_covariance;

                odom_pub.publish(odom);

                // -------------------- 更新与清理 --------------------
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