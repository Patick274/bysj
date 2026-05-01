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

    // 从 launch 读取参数 ✅ 正确
    std::string port;
    private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
    int baudrate;
    private_nh.param<int>("baudrate", baudrate, 115200);

    // 打开串口
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
    ros::Rate loop_rate(200);

    // 粘包分包缓冲区 ✅ 正确
    std::vector<uint8_t> data_buffer;

    while(ros::ok()) {
        if(ser.available()) {
            size_t n = ser.available();
            std::vector<uint8_t> temp_buf(n);
            ser.read(temp_buf.data(), n);
            data_buffer.insert(data_buffer.end(), temp_buf.begin(), temp_buf.end());

            // 固定协议 15 字节：帧头2 + 加速度6 + 角速度6 + 校验1
            while(data_buffer.size() >= 15) { 
                // 帧头检查
                if (data_buffer[0] != 0x5A || data_buffer[1] != 0x5A) {
                    data_buffer.erase(data_buffer.begin());
                    continue;
                }

                // 校验和检查 ✅ 正确
                uint8_t checksum = 0;
                for(int i=0; i<14; i++) checksum += data_buffer[i];
                if(checksum != data_buffer[14]) {
                    data_buffer.erase(data_buffer.begin());
                    continue;
                }

                // ======================
                // 在这里解析数据！
                // ======================
                int16_t ax = (int16_t)(data_buffer[2]  << 8 | data_buffer[3]);
                int16_t ay = (int16_t)(data_buffer[4]  << 8 | data_buffer[5]);
                int16_t az = (int16_t)(data_buffer[6]  << 8 | data_buffer[7]);

                int16_t gx = (int16_t)(data_buffer[8]  << 8 | data_buffer[9]);
                int16_t gy = (int16_t)(data_buffer[10] << 8 | data_buffer[11]);
                int16_t gz = (int16_t)(data_buffer[12] << 8 | data_buffer[13]);

                // 单位转换（根据你的STM32程序修改）
                float acc_coef = 0.01f;
                float gyr_coef = 0.001f;

                // 发布 IMU 消息
                sensor_msgs::Imu imu_msg;
                imu_msg.header.stamp = ros::Time::now();
                imu_msg.header.frame_id = "imu_link";

                // 角速度
                imu_msg.angular_velocity.x = gx * gyr_coef;
                imu_msg.angular_velocity.y = gy * gyr_coef;
                imu_msg.angular_velocity.z = gz * gyr_coef;

                // 线加速度
                imu_msg.linear_acceleration.x = ax * acc_coef;
                imu_msg.linear_acceleration.y = ay * acc_coef;
                imu_msg.linear_acceleration.z = az * acc_coef;

                // 四元数（先用 0，你之后可以加 yaw）
                tf2::Quaternion q;
                q.setRPY(0, 0, 0);   // 修复：不再使用未定义变量
                imu_msg.orientation = tf2::toMsg(q);

                // 协方差（完整填写）
                for(int i=0; i<9; i+=4)
                {
                imu_msg.orientation_covariance[i] = 0.001;
                imu_msg.angular_velocity_covariance[i] = 0.0001;
                imu_msg.linear_acceleration_covariance[i] = 0.001;
                }
               
                // 发布
                imu_pub.publish(imu_msg);

                // 移除已处理数据
                data_buffer.erase(data_buffer.begin(), data_buffer.begin() + 15);
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    ser.close();
    return 0;
}
