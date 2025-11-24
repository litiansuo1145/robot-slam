#ifndef __PAVO2S_COMMON_H__
#define __PAVO2S_COMMON_H__

#define BUFFER_SIZE (0x8000)   	//32K
#define FRAME_CAPCITY (0x1) //1


#define CMD_SET_GET_INDEX (2)
#define CMD_TYPE_INDEX (3)
#define CMD_SET (0xA0)
#define CMD_GET (0xB0)
#define CMD_LEN (0x4)

#define ANGLE_RANGLE_MIN (4000)
#define ANGLE_RANGLE_MAX (32000)

#define FIRING_PER_PKT 12

#include <cstdint>


enum DATA_MODE
{
    INTENSITY_DISTANCE=0x0000,
    CHECK_MODE = 0x0001,
    DISTANCE=0x0002,
};
enum PAVO2S_CMD_INDEX {
    S_CMD_DEST_IP_READ = 0,
    S_CMD_DEVICE_IP_READ = 1,
    S_CMD_FW_VER,
    S_CMD_OS_ERROR,
    S_CMD_PN = 4,

    S_CMD_SN, //5
    S_CMD_ENABLE_DATA = 6,  //6
    S_CMD_ANGLE_RANGLE,
    S_CMD_ANGLE_RESOLUTION,
    S_CMD_MOTOR_SPEED = 9,

    S_CMD_INTENSITY_MODE, //10 INTENSITY
    S_CMD_SLEEP_RESTART = 11,
    S_CMD_RESET,
    S_CMD_ECHO_MODE,
    S_CMD_MAC,

    S_CMD_DEV_IP,       //15
    S_CMD_DEST_IP,
    S_CMD_DEV_PORT,
    S_CMD_DEST_PORT,
    S_CMD_NET_UNLOCK,

    S_CMD_ANTI_DISTURB, //20
    S_CMD_PORT_READ,
    S_CMD_SPEED_ENABLE,
    S_CMD_RESOLUTION_ENABLE,
    S_CMD_UNLOCK,

    S_CMD_INDEX_LEN
};

static const unsigned char PAVO2S_CMD[][S_CMD_INDEX_LEN] = {
    { 0xEB, 0x90, 0x00, 0x00 },  //目的ip地址(只读)
    { 0xEB, 0x90, 0x00, 0x01 },  //源ip地址(只读)
    { 0xEB, 0x90, 0x00, 0x7F },  //FPGA版本
    { 0xEB, 0x90, 0x00, 0xD1 },  //系统异常
    { 0xEB, 0x90, 0x00, 0x28 },  //PN

    { 0xEB, 0x90, 0x00, 0x29 },  //SN //5
    { 0xEB, 0x90, 0x00, 0xA0 },  //ENABLE_DATA
    { 0xEB, 0x90, 0x00, 0x27 },  //ANGLE_RANGE
    { 0xEB, 0x90, 0x00, 0x3  },  //ANGLE_RESOLUTION
    { 0xEB, 0x90, 0x00, 0x10 },  //MOTOR_SPEED

    { 0xEB, 0x90, 0x00, 0xAA },  //INTENSITY 10
    { 0xEB, 0x90, 0x00, 0xE5 },  //SLEEP_RESTART
    { 0xEB, 0x90, 0x00, 0xAC },  //RESET
    { 0xEB, 0x90, 0x00, 0x11 },  //echo
    { 0xEB, 0x90, 0x00, 0xE0 },  //mac

    { 0xEB, 0x90, 0x00, 0xE1 },  //SET_DEV_IP  //15
    { 0xEB, 0x90, 0x00, 0xE0 },  //SET_DEST_IP
    { 0xEB, 0x90, 0x00, 0xE3 },  //SET_DEV_PORT
    { 0xEB, 0x90, 0x00, 0xE2 },  //SET_DEST_PORT
    { 0xEB, 0x90, 0x00, 0xEF },  //NET_UNLOCK

    { 0xEB, 0x90, 0x00, 0x15 },  //S_CMD_ANTI_DISTURB
    { 0xEB, 0x90, 0x00, 0x02 },  //目的端口，源端口(只读)
    { 0xEB, 0x90, 0x00, 0xD4 },  //转速范围寄存器
    { 0xEB, 0x90, 0x00, 0xD5 },  //角分辨率范围寄存器
    { 0xEB, 0x90, 0x00, 0x40 },  //解锁指令
};

static const uint8_t PAVO2S_CMD_ARG_LEN[S_CMD_INDEX_LEN] = {
    0x4,
    0x4,
    0x4,
    0x4,
    0x4,

    0x4,
    0x4,
    0x4,
    0x4,
    0x4,

    0x4,
    0x4,
    0x4,
    0x4,
    0x8,

    0x4,
    0x4,
    0x4,
    0x4,
    0x4,

    0x4,
    0x4,
    0x4,
    0x4,
    0x4,
};

namespace pavo2s {

    const unsigned char PROTOCOL_HEAD[8] = { 0xFF,0x53,0x4D,0x49,0x4E,0x49,0x43,0x53 };
    const int PROTOCOL_HEAD_LEN = 8;

    const int PROTOCOL_DATA_LEN = 4;
    const unsigned char PROTOCOL_END[4] = { 0xFE,0xFE,0xFE,0xFE };
    const int PROTOCOL_END_LEN = 4;


    //以下为udp部分
    const unsigned char PROTOCOL_HEAD_S[8] = { 0x00,0x00,0x00,0x08,0x00,0x01,0x30,0x02 };
    const int PROTOCOL_RESERVED_S_LEN = 26;
    const unsigned char PROTOCOL_RESERVED_S[26] = {0};
}

typedef struct CmdData
{
    unsigned char szCmd[128];
    uint16_t cmdSize;
}CmdDataDef;

#endif

