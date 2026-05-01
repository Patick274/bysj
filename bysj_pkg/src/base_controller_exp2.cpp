#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <serial/serial.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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

    // ==================== 实验2：只发布 IMU ====================
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 100);
    ros::Rate loop_rate(200);

    std::vector<uint8_t> data_buffer;

    // BMI088 原始数据转换系数（±24g / ±2000°/s 量程）
    // 若你的 STM32 下位机已做单位转换（如发的是 mm/s² 或 0.01 倍数），
    // 请与固件系数对齐，把下面两行改回 0.01f / 0.001f
    float acc_coef = 0.01f;    // 加速度计校准系数
    float gyr_coef = 0.001f;   // 陀螺仪校准系数  

    // 完整的 3×3 协方差对角矩阵
    boost::array<double, 9> imu_covariance = {
        0.001, 0.0,   0.0,
        0.0,   0.001, 0.0,
        0.0,   0.0,   0.001
    };

    while(ros::ok()) {
        if(ser.available()) {
            size_t n = ser.available();
            std::vector<uint8_t> temp_buf(n);
            ser.read(temp_buf.data(), n);
            data_buffer.insert(data_buffer.end(), temp_buf.begin(), temp_buf.end());

            // ==================== 缓冲区溢出保护 ====================
            if (data_buffer.size() > 1024) {
                ROS_WARN("Serial buffer overflow! Clearing buffer...");
                data_buffer.clear();
                continue;
            }

            // ==================== 实验2：21 字节固定帧解析 ====================
            //协议为 21 字节: 帧头(2) + Acc(6) + Gyro(6) + Vel(6) + 校验(1)
            while(data_buffer.size() >= 21) {
                // 帧头检查
                if (data_buffer[0] != 0x5A || data_buffer[1] != 0x5A) {
                    data_buffer.erase(data_buffer.begin());
                    continue;
                }

                // 校验和：只校验前20字节（IMU+Vel）== 第21字节
                uint8_t checksum = 0;
                for(int i = 0; i < 20; i++) checksum += data_buffer[i];
                if(checksum != data_buffer[20]) {
                    data_buffer.erase(data_buffer.begin());
                    continue;
                }

                // -------------------- 只解析 IMU 部分（字节 2~13）--------------------
                int16_t ax = (int16_t)(data_buffer[2]  << 8 | data_buffer[3]);
                int16_t ay = (int16_t)(data_buffer[4]  << 8 | data_buffer[5]);
                int16_t az = (int16_t)(data_buffer[6]  << 8 | data_buffer[7]);
                int16_t gx = (int16_t)(data_buffer[8]  << 8 | data_buffer[9]);
                int16_t gy = (int16_t)(data_buffer[10] << 8 | data_buffer[11]);
                int16_t gz = (int16_t)(data_buffer[12] << 8 | data_buffer[13]);


                // -------------------- 组装并发布 IMU 消息 --------------------
                sensor_msgs::Imu imu_msg;
                imu_msg.header.stamp = ros::Time::now();
                imu_msg.header.frame_id = "imu_link";

                // 线加速度
                imu_msg.linear_acceleration.x = ax * acc_coef;
                imu_msg.linear_acceleration.y = ay * acc_coef;
                imu_msg.linear_acceleration.z = az * acc_coef;

                // 角速度
                imu_msg.angular_velocity.x = gx * gyr_coef;
                imu_msg.angular_velocity.y = gy * gyr_coef;
                imu_msg.angular_velocity.z = gz * gyr_coef;

                // 四元数：实验2无姿态解算，orientation 不可用，按 ROS 规范标记为 -1
                tf2::Quaternion q;
                q.setRPY(0, 0, 0);
                imu_msg.orientation = tf2::toMsg(q);
                // 直接整个数组赋值，明确告诉所有人：orientation 完全没观测
                boost::array<double, 9> orientation_cov_unknown = 
                {
                 -1.0, 0, 0,
                    0, 0, 0,
                    0, 0, 0
                };
                imu_msg.orientation_covariance = orientation_cov_unknown; //imu_msg.orientation_covariance[0] = -1.0;  // 告知融合算法忽略

                // 角速度与线加速度协方差（完整 3×3 对角阵）
                imu_msg.angular_velocity_covariance = imu_covariance;
                imu_msg.linear_acceleration_covariance = imu_covariance;

                imu_pub.publish(imu_msg);

                // 移除已处理的 21 字节
                data_buffer.erase(data_buffer.begin(), data_buffer.begin() + 21);
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    ser.close();
    return 0;
}