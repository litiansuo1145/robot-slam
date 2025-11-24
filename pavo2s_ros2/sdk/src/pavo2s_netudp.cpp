#include <time.h>
#include <chrono>
#include <fstream>
#include <sstream>

#include "../include/pavo2s_netudp.h"
#include "../include/pavo2s_common.h"

std::mutex pavo2s::NetConnectUDP::socket_mutx_;

char g_packet_start_sign[8] = { 0x58,0x4D,0x47,0x44,0x31,0x32,0x30,0x31};

inline uint16_t UInt16Val2HostOrder(const void* ptr)
{
    auto char_ptr = reinterpret_cast<const char*>(ptr);
    return (((uint16_t(*char_ptr) << 8) & 0xFF00)
    | ((uint16_t(*(char_ptr + 1))) & 0x00FF));
}

bool pavo2s::NetConnectUDP::recvData(arr_pavo_packet_info_scan_t& arr_packet_info, uint16_t time_out)
{
    vec_pavo_packet_info_scan_t vec_packet_info;
    bool ret = this->readPoint(vec_packet_info, time_out);
    if(ret)
    {
        for(uint32_t cnt = 0; cnt < vec_packet_info.vec_buff.size() && cnt < arr_packet_info.count; cnt++)
        {
            arr_packet_info.data_buffer[cnt].angle = vec_packet_info.vec_buff[cnt].angle;
            arr_packet_info.data_buffer[cnt].distance = vec_packet_info.vec_buff[cnt].distance;
            arr_packet_info.data_buffer[cnt].intensity = vec_packet_info.vec_buff[cnt].intensity;
        }
        arr_packet_info.time_stamp = vec_packet_info.time_stamp;
        if(m_pMADataFilter && (arr_packet_info.count > 0))
        {
            m_pMADataFilter->Filter(arr_packet_info.data_buffer, arr_packet_info.count);
        }
        return true;
    }
    return false;
}

bool pavo2s::NetConnectUDP::recvData(vec_pavo_packet_info_scan_t& vec_packet_info, uint16_t time_out)
{
    bool ret = this->readPoint(vec_packet_info, time_out);
    if(ret)
    {
        if(m_pMADataFilter && (vec_packet_info.vec_buff.size() > 0))
        {
            m_pMADataFilter->Filter(&(vec_packet_info.vec_buff[0]), int(vec_packet_info.vec_buff.size()));
        }
        return true;
    }
    return false;
}

bool pavo2s::NetConnectUDP::recvCMD(PAVO2S_CMD_INDEX cmd,unsigned char* content,uint16_t & count,uint16_t time_out)
{
    if(time_out <= 0)
       time_out = 1000;
   
    {
        std::unique_lock<std::mutex> guard(this->m_cmd_op_mutx_);
        if(time_out == 0)
        {
            while(this->noMatchingCmd(cmd) && this->is_receiving_)
            {
                this->m_cmd_op_cond_.wait(guard);
            }

        }
        else //timeout > 0
        {
            while(this->noMatchingCmd(cmd) && this->is_receiving_)
            {
                if(m_cmd_op_cond_.wait_for(guard,std::chrono::milliseconds(time_out)) == std::cv_status::timeout)
                    return false;
            }
        }

        if(!this->is_receiving_)
            return false;

        bool bFind = false;
        unsigned char contentTemp[128] = { 0 };
        auto it = m_mapCmd.find(PAVO2S_CMD[cmd][CMD_TYPE_INDEX]);
        if(it != m_mapCmd.end())
        {
            count = it->second->cmdSize;
            memcpy(contentTemp,it->second->szCmd,count);
            contentTemp[CMD_SET_GET_INDEX] = 0x00;
            if((count >= CMD_LEN) && (memcmp(contentTemp,PAVO2S_CMD[cmd],CMD_LEN) == 0))
            {
                memcpy(content,it->second->szCmd,count);
                CmdDataDef* cmdTemp = it->second;
                m_mapCmd.erase(it);
                delete cmdTemp;
                bFind = true;
            }
        }
        if(bFind == false)
            return false;
    }
    return true;
}

bool pavo2s::NetConnectUDP::recvCMDNoWait(PAVO2S_CMD_INDEX cmd,unsigned char * content,uint16_t& count)
{
    std::unique_lock<std::mutex> guard(this->m_cmd_op_mutx_);

    bool bFind = false;
    unsigned char contentTemp[128] = { 0 };
    auto it = m_mapCmd.find(PAVO2S_CMD[cmd][CMD_TYPE_INDEX]);
    if(it != m_mapCmd.end())
    {
        count = it->second->cmdSize;
        memcpy(contentTemp,it->second->szCmd,count);
        contentTemp[CMD_SET_GET_INDEX] = 0x00;
        if((count >= CMD_LEN) && (memcmp(contentTemp,PAVO2S_CMD[cmd],CMD_LEN) == 0))
        {
            memcpy(content,it->second->szCmd,count);
            CmdDataDef* cmdTemp = it->second;
            m_mapCmd.erase(it);
            delete cmdTemp;
            bFind = true;
        }
    }
    return bFind;
}

bool pavo2s::NetConnectUDP::sendCMD(unsigned char * content, uint16_t size)
{
    bool ret;
    if(content[CMD_SET_GET_INDEX] == CMD_SET)
    {
        ret = enable_Unlock(true);
        if(!ret) return false;
        ret = this->write(content, size);
    }
    else if(content[CMD_SET_GET_INDEX] == CMD_GET)
    {
        this->clearCMD(content[CMD_TYPE_INDEX]);
        content[CMD_SET_GET_INDEX] = 0xB1;
        ret = this->write(content, size);
        if(!ret)
        {
            return false;
        }
    }
    return true;
}

bool pavo2s::NetConnectUDP::enable_tail_filter(uint16_t method)
{
    if(m_pMADataFilter)
    {
        m_pMADataFilter->EnableTailFilter(method);
        return true;
    }
    return false;
}

bool pavo2s::NetConnectUDP::enable_front_filter(uint16_t iLevel)
{
    if(m_pMADataFilter)
    {
        m_pMADataFilter->EnableFrontFilter(iLevel);
        return true;
    }
    return false;
}

pavo2s::NetConnectUDP::NetConnectUDP(uint16_t dest_port) :
    is_receiving_(false),
    m_dest_port(dest_port),
    m_scans_que(MAX_PACKET_QUE_SIZE)
{
    m_pMADataFilter = new MADataFilter();

#ifdef _WIN32
    m_iSocket = INVALID_SOCKET;
    m_iWSACleanupCount = 0;
#else
    m_iSocket = 0;
#endif
}

pavo2s::NetConnectUDP::~NetConnectUDP()
{
    udpClose();
}


bool pavo2s::NetConnectUDP::open(std::string lidar_ip, uint16_t lidar_port)
{
    std::unique_lock<std::mutex> guard(this->socket_mutx_);

    m_strLidarIP = lidar_ip;
#ifdef _WIN32

#ifndef WSA_START_UP_READY
    WSADATA wsadata;
    m_iWSACleanupCount++;
    int nError = WSAStartup(MAKEWORD(2,2),&wsadata);
    if(nError != 0)
    {
        return false;
    }
#endif // !WSA_START_UP_READY

    SOCKADDR_IN receiveAddr;
    receiveAddr.sin_family = AF_INET;
    receiveAddr.sin_port = htons(m_dest_port);
    receiveAddr.sin_addr.S_un.S_addr = htonl(INADDR_ANY);

    m_iSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if(m_iSocket == INVALID_SOCKET)
    {
        closesocket(m_iSocket);
#ifndef WSA_START_UP_READY
        if(m_iWSACleanupCount > 0)
        {
            m_iWSACleanupCount--;
            WSACleanup();
        }
#endif // !WSA_START_UP_READY
        return false;
    }
    int retVal = ::bind((SOCKET)m_iSocket,(sockaddr*)(&receiveAddr),sizeof(SOCKADDR));
    if(retVal == SOCKET_ERROR)
    {
        closesocket(m_iSocket);
#ifndef WSA_START_UP_READY
        if(m_iWSACleanupCount > 0)
        {
            m_iWSACleanupCount--;
            WSACleanup();
        }
#endif // !WSA_START_UP_READY
        return false;
    }
    memset(&sendAddr,0,sizeof(SOCKADDR_IN));
    sendAddr.sin_family = AF_INET;
    sendAddr.sin_port = htons(lidar_port);
    sendAddr.sin_addr.S_un.S_addr = inet_addr(lidar_ip.c_str());
#else
    m_iSocket = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    if(m_iSocket == -1)
    {
        return false;
    }
    struct sockaddr_in receiveAddr;
    memset(&receiveAddr,0,sizeof(receiveAddr));
    receiveAddr.sin_family = AF_INET;
    receiveAddr.sin_port = htons(m_dest_port);
    receiveAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    if(bind(m_iSocket,(struct sockaddr*)&receiveAddr,sizeof(receiveAddr)) == -1)
    {
        return false;
    }
    memset(&sendAddr, 0, sizeof(sendAddr));
    sendAddr.sin_family = AF_INET;
    sendAddr.sin_port = htons(lidar_port);
    sendAddr.sin_addr.s_addr = inet_addr(lidar_ip.c_str());
#endif
    is_receiving_ = true;
    m_strLidarIP = lidar_ip;

    resetReceiveState();

    thread_receive = std::thread(&pavo2s::NetConnectUDP::ReceiveThread, this);
    return true;
}

bool pavo2s::NetConnectUDP::isOpen()
{
    return  this->is_receiving_;
}

bool pavo2s::NetConnectUDP::udpClose()
{
    std::unique_lock<std::mutex> guard(this->socket_mutx_);
#ifdef _WIN32
    closesocket(m_iSocket);
    m_iSocket = INVALID_SOCKET;
#ifndef WSA_START_UP_READY
    if(m_iWSACleanupCount > 0)
    {
        m_iWSACleanupCount--;
        WSACleanup();
    }
#endif //WSA_START_UP_READY
#else
    shutdown(m_iSocket, SHUT_RDWR);
    close(m_iSocket);
#endif //_WIN32

    is_receiving_ = false;
    if(thread_receive.joinable())
        thread_receive.join();
    return true;
}

bool pavo2s::NetConnectUDP::readPoint(vec_pavo_packet_info_scan_t& vec_packet_info,uint16_t time_out)
{
    return m_scans_que.dequeue(vec_packet_info, time_out);
}

bool pavo2s::NetConnectUDP::write(unsigned char * content, uint16_t size)
{
    uint16_t send_len;
    unsigned char send_buffer[BUFFER_SIZE];
    memset(send_buffer,0, BUFFER_SIZE);
    //add the protocol fixed head
    send_len = 0;
    memcpy(send_buffer + send_len, PROTOCOL_HEAD_S, PROTOCOL_HEAD_LEN);
    send_len = send_len + PROTOCOL_HEAD_LEN;
    //add the version and reserver
    memcpy(send_buffer + send_len, PROTOCOL_RESERVED_S, PROTOCOL_RESERVED_S_LEN);
    send_len = send_len + PROTOCOL_RESERVED_S_LEN;
    //add the content
    memcpy(send_buffer + send_len, content, size);
    send_len = send_len + size;
    int iRt = 0;

#ifdef _WIN32
    iRt = sendto(m_iSocket,(char*)send_buffer,send_len,0,(sockaddr*)&sendAddr, sizeof(sendAddr));
#else
    iRt = sendto(m_iSocket, send_buffer, send_len, 0, (const struct sockaddr *)&sendAddr, sizeof(sendAddr));
#endif
    return (iRt > 0);
}

int pavo2s::NetConnectUDP::getErrorCode()
{
    return 0;
}

void pavo2s::NetConnectUDP::resetReceiveState()
{
    m_points_expected_now = 0;
    m_recved_points_num = 0;

    m_current_scan.time_stamp = 0;
    m_current_scan.vec_buff.clear();
}

void pavo2s::NetConnectUDP::parsePacket(char* data, uint16_t len)
{
    if(memcmp(g_packet_start_sign,data,8))
    {
        std::unique_lock<std::mutex> guard(this->m_cmd_op_mutx_);

        CmdDataDef* cmdData = new CmdDataDef();
        cmdData->cmdSize = len - 34;
        memset(cmdData->szCmd,0,sizeof(cmdData->szCmd));
        memcpy(cmdData->szCmd,data + 34,len - 34);
        m_mapCmd[cmdData->szCmd[CMD_TYPE_INDEX]] = cmdData;

        this->m_cmd_op_cond_.notify_one();
    }
    else
    {
        packet_head_t* pPacketHead = (packet_head*)data;
        uint16_t scan_serial = pPacketHead->ScanSerial();
        uint16_t packet_serial = pPacketHead->PacketSerial();
        uint16_t packet_points_num = pPacketHead->PacketPointsNum();
        uint16_t scan_points_num = pPacketHead->ScanPointsNum();
        uint16_t first_index = pPacketHead->FirstIndex();
/*
 * may receive data
 * 1. packet lost (lost first, middle, or last)
 * 2. receive packet from the middle of the scan
 * 3. receive packet from the first packet of the scan(packet 0)
 */
        if(packet_serial == 0) //new scan
        {
            //push previous scan. filter the initial state
            if((m_points_expected_now > 0)
                    && (m_current_scan.vec_buff.size() == m_points_expected_now))
            {
                m_scans_que.enqueue(std::move(m_current_scan));
            }
            resetReceiveState();

            m_scan_serial_now = scan_serial;
            m_points_expected_now = scan_points_num;
            m_current_scan.time_stamp = ntohl(pPacketHead->time_stamp);
            m_current_scan.vec_buff.resize(scan_points_num);
        }
        else if(m_scan_serial_now != scan_serial)//continued scan: filter receive halfway
        {
            resetReceiveState();
            return;
        }

       m_recved_points_num += packet_points_num;
       auto pcl_ptr = data + sizeof(packet_head);
       auto first_angle = pPacketHead->FirstAngle();
       auto agular_increment = pPacketHead->AngularIncrement();
       //for(int i = first_index; i < first_index + packet_points_num; i++)
       for(uint16_t i = 0; i < packet_points_num; i++)
       {
           pavo_response_scan_t& point = m_current_scan.vec_buff[i + first_index];
           point.angle = first_angle + agular_increment * i;
           //point.intensity = charToShort(pcl_ptr + i * 4);
           //point.distance = charToShort(pcl_ptr + 2 + i * 4);
           point.intensity = UInt16Val2HostOrder(pcl_ptr);
           point.distance = UInt16Val2HostOrder(pcl_ptr + 2);
           pcl_ptr += 4;
       }

    }
}

void pavo2s::NetConnectUDP::ReceiveThread(NetConnectUDP* pUdpConnect)
{
    char recv_buffer[BUFFER_SIZE];
#ifdef _WIN32
    SOCKADDR_IN peer;
    int len = sizeof(peer);
    while(pUdpConnect->is_receiving_)
    {
        int iRt = recvfrom(pUdpConnect->m_iSocket, recv_buffer,BUFFER_SIZE - 1,0,(struct sockaddr*)&peer, &len);
        if(iRt == SOCKET_ERROR)
            continue;

        std::string from_ip = inet_ntoa(peer.sin_addr);
        if(from_ip.compare(pUdpConnect->m_strLidarIP) == 0)
        {
            pUdpConnect->parsePacket(recv_buffer,iRt);
        }
    }
#else
    struct sockaddr_in peer;
    socklen_t len = sizeof(peer);
    while(pUdpConnect->is_receiving_)
    {
        memset(recv_buffer, 0, BUFFER_SIZE);
        ssize_t s = recvfrom(pUdpConnect->m_iSocket, recv_buffer, BUFFER_SIZE - 1, 0, (struct sockaddr*)&peer, &len);
        if(s > 0)
        {
            std::string from_ip = inet_ntoa(peer.sin_addr);
            if(from_ip.compare(pUdpConnect->m_strLidarIP) == 0)
            {
                pUdpConnect->parsePacket(recv_buffer, s);
            }
        }
        else
        {
            continue;
        }
    }
#endif
}

inline bool pavo2s::NetConnectUDP::noMatchingCmd(PAVO2S_CMD_INDEX cmd)
{
    return (m_mapCmd.find(PAVO2S_CMD[cmd][CMD_TYPE_INDEX]) == m_mapCmd.end());
}

bool pavo2s::NetConnectUDP::clearCMD(uint16_t cmdType)
{
    m_mapCmd.erase(cmdType);
    return true;
}

bool pavo2s::NetConnectUDP::enable_Unlock(bool val)
{
    bool ret;
    unsigned char send_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_UNLOCK;;
    memset(send_cmd,0,8);
    memcpy(send_cmd,PAVO2S_CMD[cmd_type],CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
    if(val) send_cmd[7] = 0x01;
    ret = this->write(send_cmd,8);
    return ret;
}

