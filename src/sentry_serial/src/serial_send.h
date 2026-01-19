#ifndef SERIAL_SEND_H
#define SERIAL_SEND_H

#include <serial/serial.h>
#include <cstdint>
#include <string>

extern serial::Serial sentry_ser;
extern const size_t data_len;
extern std::string cmd_vel_topic;

#pragma pack(push, 1)
union Serial_Package
{
    struct
    {
        uint8_t  header;     // 0x7A
        uint8_t  header2;    // 0x7B
        float    linear_x;   // 4
        float    linear_y;   // 4
        float    angular_z;  // 4
        double   pos_x;
        double   pos_y;
        double   yaw;
        uint16_t checksum;   // 2 (CRC16)
        uint8_t  footer;     // 0x7D
    } fields;
    uint8_t Send_Buffer[41]; // 必须与 struct 大小一致
};
#pragma pack(pop)

static_assert(sizeof(Serial_Package) == 41, "Serial_Package must be 41 bytes");

#endif // SERIAL_SEND_H
