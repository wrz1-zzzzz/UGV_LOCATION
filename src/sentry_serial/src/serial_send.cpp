#include "ros/ros.h"
#include "serial_send.h"

#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <cstring>
#include <iomanip>

// 全局定义
serial::Serial sentry_ser;
const size_t data_len = sizeof(Serial_Package); // 41
std::string cmd_vel_topic = "/cmd_vel";
Serial_Package serial_package{}; // 初始化时所有位默认为0


// CRC16 (Modbus) implementation
static uint16_t Calculate_CRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

// 删除了 callback_odom，因为不需要接收里程计数据了

void callback(const geometry_msgs::Twist& cmd_vel)
{
    // 打印调试信息
    ROS_INFO("Receive cmd_vel: x=%.2f, y=%.2f, yaw=%.2f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    serial_package.fields.header = 0x7A;
    serial_package.fields.header2 = 0x7B;
    
    // 1. 填入速度信息
    serial_package.fields.linear_x = cmd_vel.linear.x;
    serial_package.fields.linear_y = cmd_vel.linear.y;
    serial_package.fields.angular_z = cmd_vel.angular.z;

    // 2. 强制将位置信息置为 0 (模拟里程计数据)
    serial_package.fields.pos_x = 0.0;
    serial_package.fields.pos_y = 0.0;
    serial_package.fields.yaw   = 0.0;

    // 3. 计算 CRC (范围不变，仍然包含那些为0的位置数据)
    const uint8_t* crc_start = serial_package.Send_Buffer + 2; 
    const size_t crc_len = sizeof(serial_package.fields.linear_x)
                         + sizeof(serial_package.fields.linear_y)
                         + sizeof(serial_package.fields.angular_z)
                         + sizeof(serial_package.fields.pos_x)
                         + sizeof(serial_package.fields.pos_y)
                         + sizeof(serial_package.fields.yaw);
                         
    uint16_t crc = Calculate_CRC16(crc_start, crc_len);
    serial_package.Send_Buffer[38] = static_cast<uint8_t>(crc & 0xFF);        // checksum low
    serial_package.Send_Buffer[39] = static_cast<uint8_t>((crc >> 8) & 0xFF); // checksum high

    serial_package.fields.footer = 0x7D;

    // 发送
    try {
        if (!sentry_ser.isOpen()) {
            ROS_ERROR("Serial port not open when trying to send");
            return;
        }
        
        // 关键修改：删除了 "如果pos_x, pos_y, yaw全为0则不发送" 的判断
        // 现在即使全是0，也会发送数据包，保证下位机能收到速度指令
        
        sentry_ser.flush();
        size_t written = sentry_ser.write(serial_package.Send_Buffer, data_len);
        
        // 简化日志，避免刷屏太快
        // ROS_INFO_STREAM("Sent " << written << " bytes. CRC=" << crc);
        
    } catch (const serial::IOException& e) {
        ROS_ERROR_STREAM("Serial write failed: " << e.what());
    }
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "sentry_send_teleop"); // 改了个节点名区分
    ros::NodeHandle n("~");

    n.param<std::string>("cmd_vel_topic", cmd_vel_topic, cmd_vel_topic);

    try
    {
        std::string serial_port;
        if (argc == 2) serial_port = argv[1];
        else serial_port = "/dev/ttyUSB0"; // 确保这里的端口号是对的

        sentry_ser.setPort(serial_port);
        sentry_ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        sentry_ser.setTimeout(to);
        sentry_ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port: " << e.what());
        return -1;
    }

    if (sentry_ser.isOpen()) ROS_INFO_STREAM("Serial Port opened");
    else {
        ROS_ERROR_STREAM("Serial Port failed to open");
        return -1;
    }

    ROS_INFO_STREAM("Init Finished! packet_size=" << sizeof(Serial_Package));
    
    // 删除了 odom_sub 订阅
    // 只保留 cmd_vel 订阅
    ros::Subscriber sub = n.subscribe(cmd_vel_topic, 1000, callback);
    
    ROS_INFO("Ready for Teleop Twist Keyboard...");
    
    ros::spin();
    return 0;
}
