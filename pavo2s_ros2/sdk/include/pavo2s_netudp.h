#ifndef __PAVO2_NETUDP_H__
#define __PAVO2_NETUDP_H__

#include <map>
#include <mutex>
#include <thread>
#include <condition_variable>

#include "pavo2s_common.h"
#include "pavo2s_types.h"
#include "utils.h"
#include "pavo2s_filter.h"

#ifdef _WIN32
#include <winsock2.h>
#include <WS2tcpip.h>
#include <iostream>
#pragma comment(lib, "ws2_32.lib")
#else
#include "sys/types.h"
#include "sys/socket.h"
#include "arpa/inet.h"
#include "unistd.h"
#endif


#pragma pack(push)
#pragma pack(1)
typedef struct packet_head
{
    uint16_t ScanSerial()
    {
        return ntohs(scan_serial);
    }
    uint16_t PacketSerial()
    {
        return ntohs(packet_serial);
    }
    uint32_t TimeStamp()
    {
        return ntohl(time_stamp);
    }
    uint16_t ScanPointsNum()
    {
        return ntohs(scan_points_num);
    }
    uint16_t PacketPointsNum()
    {
        return ntohs(packet_points_num);
    }
    uint16_t FirstAngle()
    {
        return ntohs(first_angle);
    }
    uint16_t AngularIncrement()
    {
        return ntohs(angular_increment);
    }
    uint16_t FirstIndex()
    {
        return ntohs(first_index);
    }

    uint32_t start_sign1;
    uint32_t start_sign2;
    uint8_t version;
    uint8_t packet_type;
    uint16_t reserved1;
    uint16_t packet_size;
    uint16_t head_size;
    uint16_t scan_serial;
    uint16_t packet_serial;
    uint16_t total_packet_num;
    uint16_t reserved2;
    uint32_t time_stamp;
    uint32_t status_flag;
    uint32_t scan_frequency;
    uint16_t scan_points_num;
    uint16_t packet_points_num;
    uint16_t first_index;
    uint16_t reserved3;
    uint16_t first_angle;
    uint16_t angular_increment;
}packet_head_t;
#pragma pack(pop)

typedef struct scan_base_info
{
    uint16_t first_angle;
    uint16_t angular_increment;
    uint16_t size;
    uint32_t time_stamp;
}scan_base_info_t;

namespace pavo2s
{

#define MAX_PACKET_QUE_SIZE (100)
class NetConnectUDP
{
public:
    static void ReceiveThread(NetConnectUDP* pUdpConnect);
public:
    bool recvData(arr_pavo_packet_info_scan_t& arr_packet_info, uint16_t time_out = 0);
    bool recvData(vec_pavo_packet_info_scan_t& vec_packet_info, uint16_t time_out = 0);
    bool recvCMD(PAVO2S_CMD_INDEX cmd, unsigned char * content, uint16_t & count, uint16_t time_out = 0);
    bool recvCMDNoWait(PAVO2S_CMD_INDEX cmd,unsigned char * content, uint16_t & count);
    bool sendCMD(unsigned char * content, uint16_t size);
    bool enable_tail_filter(uint16_t method = 0);
    bool enable_front_filter(uint16_t iLevel = 0);
public:
    NetConnectUDP(uint16_t dest_port);
    ~NetConnectUDP();
    bool open(std::string lidar_ip, uint16_t lidar_port);
    bool isOpen();
    bool udpClose();
    int  getErrorCode();
    void parsePacket(char* data, uint16_t len);
    bool enable_Unlock(bool val);

    bool is_receiving_;

private:
    bool readPoint(vec_pavo_packet_info_scan_t& vtr_packet_info, uint16_t time_out = 0);
    bool write(unsigned char * content, uint16_t size);
    bool noMatchingCmd(PAVO2S_CMD_INDEX cmd);
    bool clearCMD(uint16_t cmdType);
    void resetReceiveState();

    static std::mutex socket_mutx_;

#ifdef _WIN32
    SOCKET m_iSocket;
    int m_iWSACleanupCount;
    SOCKADDR_IN sendAddr;
#else
    int m_iSocket;
    struct sockaddr_in sendAddr;
#endif
    std::thread thread_receive;

    uint16_t m_dest_port;
    std::string m_strLidarIP;

    //packet_head_t m_first_packet_header;
    uint16_t m_scan_serial_now;
    uint16_t m_points_expected_now;
    uint16_t m_recved_points_num;

    vec_pavo_packet_info_scan_t m_current_scan;
    SynchronizedQueue<vec_pavo_packet_info_scan_t> m_scans_que;

    std::mutex m_cmd_op_mutx_;
    std::condition_variable m_cmd_op_cond_;
    std::map<uint16_t,CmdDataDef*> m_mapCmd;

    MADataFilter* m_pMADataFilter;
};
}
#endif

