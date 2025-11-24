#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <iostream>

#include "../include/pavo2s_driver.h"
#include "../include/pavo2s_netudp.h"
#include "../include/pavo2s_common.h"



pavo2s::pavo2s_driver::pavo2s_driver(uint16_t dest_port)
{
    conn_ptr_ = new NetConnectUDP(dest_port);
}

pavo2s::pavo2s_driver::~pavo2s_driver()
{
    if (conn_ptr_)delete conn_ptr_;
}

bool pavo2s::pavo2s_driver::pavo2s_open(std::string lidar_ip, uint16_t lidar_port)
{
    return conn_ptr_->open(lidar_ip, lidar_port);
}

bool pavo2s::pavo2s_driver::pavo2s_is_open()
{
    return conn_ptr_->isOpen();
}

bool pavo2s::pavo2s_driver::pavo2s_close()
{
    return conn_ptr_->udpClose();
}

bool pavo2s::pavo2s_driver::get_scanned_data(arr_pavo_packet_info_scan_t& arr_packet_info,uint16_t timeout)
{
    return conn_ptr_->recvData(arr_packet_info,timeout);
}

bool pavo2s::pavo2s_driver::get_scanned_data(vec_pavo_packet_info_scan_t& vtr_packet_info,uint16_t timeout)
{
    vtr_packet_info.vec_buff.clear();
    return conn_ptr_->recvData(vtr_packet_info,timeout);
}

bool pavo2s::pavo2s_driver::get_scanned_data(arr_pavo_packet_info_pcd_t& arr_packet_info,uint16_t timeout)
{
    vec_pavo_packet_info_scan_t vec_packet_scan_info;
    bool ret = conn_ptr_->recvData(vec_packet_scan_info,timeout);
    if (ret)
    {
        for (uint32_t cnt = 0; cnt < vec_packet_scan_info.vec_buff.size() && (cnt < arr_packet_info.count); cnt++)
        {
            arr_packet_info.data_buffer[cnt].intensity = vec_packet_scan_info.vec_buff[cnt].intensity;
            arr_packet_info.data_buffer[cnt].x = vec_packet_scan_info.vec_buff[cnt].distance*cos(Degree2Radians(vec_packet_scan_info.vec_buff[cnt].angle/100.0f));
            arr_packet_info.data_buffer[cnt].y = vec_packet_scan_info.vec_buff[cnt].distance*sin(Degree2Radians(vec_packet_scan_info.vec_buff[cnt].angle/100.0f));
            arr_packet_info.data_buffer[cnt].z = 0;
        }
        arr_packet_info.count = uint16_t(vec_packet_scan_info.vec_buff.size());
        arr_packet_info.time_stamp = vec_packet_scan_info.time_stamp;
    }
    return ret;
}

bool pavo2s::pavo2s_driver::get_scanned_data(vec_pavo_packet_info_pcd_t& vec_packet_info,uint16_t timeout)
{
    vec_packet_info.vec_buff.clear();

    pavo_response_pcd_t pcd;
    vec_pavo_packet_info_scan_t vec_packet_scan_info;
    bool ret = conn_ptr_->recvData(vec_packet_scan_info,timeout);
    if (ret)
    {
        for (uint32_t cnt = 0; cnt < vec_packet_scan_info.vec_buff.size(); cnt++)
        {
            pcd.intensity = vec_packet_scan_info.vec_buff[cnt].intensity;
            pcd.x = vec_packet_scan_info.vec_buff[cnt].distance*cos(Degree2Radians(vec_packet_scan_info.vec_buff[cnt].angle/100.0f));
            pcd.y = vec_packet_scan_info.vec_buff[cnt].distance*sin(Degree2Radians(vec_packet_scan_info.vec_buff[cnt].angle/100.0f));
            pcd.z = 0;
            vec_packet_info.vec_buff.push_back(pcd);
        }
        vec_packet_info.time_stamp = vec_packet_scan_info.time_stamp;
    }
    return ret;
}

bool pavo2s::pavo2s_driver::get_device_sn(std::string & sn)
{
    bool ret;
    unsigned char send_cmd[8];
    unsigned char recv_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_SN;
    uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, CMD_LEN);
    if (!ret) return false;
    //recvice the cmd
    ret = conn_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
    if (!ret) return false;
    //check the type and arg len
    ret = check_cmd_s(cmd_type, recv_cmd, recv_len);
    if (!ret) return false;
    //check the value
    std::stringstream ss;
    for (uint8_t i = CMD_LEN; i < recv_len; i++)
    {
        ss << std::hex << std::setw(2) << std::setfill('0')
           << std::setiosflags(std::ios::uppercase) << (short)recv_cmd[i];
        if (i == (recv_len - 1)) continue;
    }
    sn = ss.str();
    return ret;
}

bool pavo2s::pavo2s_driver::get_device_pn(std::string & pn)
{
    bool ret;
    unsigned char send_cmd[8];
    unsigned char recv_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_PN;
    uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, CMD_LEN);
    if (!ret) return false;
    //recvice the cmd
    ret = conn_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
    if (!ret) return false;
    //check the type and arg len
    ret = check_cmd_s(cmd_type, recv_cmd, recv_len);
    if (!ret) return false;
    //check the value
    std::stringstream ss;
    for (uint8_t i = CMD_LEN; i < recv_len; i++)
    {
        ss << std::hex << std::setw(2) << std::setfill('0')
           << std::setiosflags(std::ios::uppercase) << (short)recv_cmd[i];
        if (i == (recv_len - 1)) continue;
    }
    pn = ss.str();
    return ret;
}

bool pavo2s::pavo2s_driver::get_sdk_ver(std::string & ver)
{
    ver = std::string(SDK_VER);
    return true;
}

bool pavo2s::pavo2s_driver::get_firmware_ver(std::string & ver)
{
    bool ret;
    unsigned char send_cmd[8];
    unsigned char recv_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_FW_VER;
    uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, CMD_LEN);
    if (!ret) return false;
    //recvice the cmd
    ret = conn_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
    if (!ret) return false;
    //check the type and arg len
    ret = check_cmd_s(cmd_type, recv_cmd, recv_len);
    if (!ret) return false;
    //check the value
    std::stringstream ss;
    for (uint8_t i = CMD_LEN; i < recv_len; i++)
    {
        ss << (short)recv_cmd[i];
        if (i == (recv_len - 1)) continue;
        ss << ".";
    }
    ver = ss.str();
    return ret;
}

bool pavo2s::pavo2s_driver::get_os_error(uint32_t& errCode,uint8_t iIndex/* = 0*/)
{
    bool ret;
    unsigned char send_cmd[8];
    unsigned char recv_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_OS_ERROR;
    uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
    memcpy(send_cmd,PAVO2S_CMD[cmd_type],CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd,CMD_LEN);
    if(!ret) return false;
    //recvice the cmd
    ret = conn_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
    if(!ret) return false;
    //check the type and arg len
    ret = check_cmd_s(cmd_type,recv_cmd,recv_len);
    if(!ret) return false;
    //check the value
    errCode = 0;
    if(iIndex > 0 && iIndex <= 4)
    {
        errCode = (recv_cmd[CMD_LEN + 3]) & (1 << (iIndex - 1));
    }
    else
    {
        errCode = ((uint32_t)recv_cmd[CMD_LEN + 3]);
    }
    return ret;
}

int  pavo2s::pavo2s_driver::get_error_code()
{
    return conn_ptr_->getErrorCode();
}

bool pavo2s::pavo2s_driver::enable_data(bool en)
{
    bool ret;
    unsigned char send_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_ENABLE_DATA;
    uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
    memset(send_cmd, 0, send_len);
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
    send_len = CMD_LEN + PAVO2S_CMD_ARG_LEN[cmd_type];
    if (en) send_cmd[send_len - 1] = 0x01;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, send_len);
    if (!ret) return false;
    return ret;
}

bool pavo2s::pavo2s_driver::set_angle_range(uint16_t start, uint16_t end)
{
    if(start < ANGLE_RANGLE_MIN || end > ANGLE_RANGLE_MAX || start >= end)
        return false;

    bool ret;
    unsigned char send_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_ANGLE_RANGLE;
    uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
    memset(send_cmd, 0, send_len);
    //set the content
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
    send_cmd[CMD_LEN] = 0xFF & (start >> 8);
    send_cmd[CMD_LEN + 1] = 0xFF & (start);
    send_cmd[CMD_LEN + 2] = 0xFF & (end >> 8);
    send_cmd[CMD_LEN + 3] = 0xFF & (end);
    //set the send len
    send_len = CMD_LEN + PAVO2S_CMD_ARG_LEN[cmd_type];
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, send_len);
    if (!ret) return false;
    return ret;
}

bool pavo2s::pavo2s_driver::get_angle_range(uint16_t & start, uint16_t & end)
{
    bool ret;
    unsigned char send_cmd[8];
    unsigned char recv_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_ANGLE_RANGLE;
    uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, CMD_LEN);
    if (!ret) return false;
    //recvice the cmd
    ret = conn_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
    if (!ret) return false;
    //check the type and arg len
    ret = check_cmd_s(cmd_type, recv_cmd, recv_len);
    if (!ret) return false;
    //check the value
    start = (uint16_t)recv_cmd[CMD_LEN] * 0x100 + (uint16_t)recv_cmd[CMD_LEN + 1];
    end = (uint16_t)recv_cmd[CMD_LEN + 2] * 0x100 + (uint16_t)recv_cmd[CMD_LEN + 3];
    return ret;
}

bool pavo2s::pavo2s_driver::get_motor_speed_enable(uint32_t& val)
{
    bool ret;
    unsigned char send_cmd[8];
    unsigned char recv_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_SPEED_ENABLE;
    uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, CMD_LEN);
    if(!ret) return false;
    //recvice the cmd
    ret = conn_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
    if(!ret) return false;
    //check the type and arg len
    ret = check_cmd_s(cmd_type, recv_cmd, recv_len);
    if(!ret) return false;
    //check the value
    val = ((uint32_t)recv_cmd[CMD_LEN + 3]) + (((uint32_t)recv_cmd[CMD_LEN + 2]) << 8) + (((uint32_t)recv_cmd[CMD_LEN + 1]) << 16) + (((uint32_t)recv_cmd[CMD_LEN]) << 24);
    return ret;
}

bool pavo2s::pavo2s_driver::set_motor_speed(uint8_t val)
{
    uint32_t enableValue;
    bool ret = get_motor_speed_enable(enableValue);
    if(!ret)
        return false;

    std::set<uint8_t> setSpeedEnable;
    for(int a = 0; a < 8; a++)
    {
        if(enableValue & 0x1)
        {
            setSpeedEnable.insert(15 + a * 5);
        }
        enableValue = enableValue >> 1;
    }

    if(setSpeedEnable.find(val) == setSpeedEnable.end())
    {
        return false;
    }

    unsigned char send_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_MOTOR_SPEED;
    uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
    memset(send_cmd, 0, send_len);
    //set the content
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
    send_cmd[CMD_LEN + 3] = 0xFF & (val);
    //set the send len
    send_len = CMD_LEN + PAVO2S_CMD_ARG_LEN[cmd_type];
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, send_len);
    if(!ret) return false;
    return ret;
}

bool pavo2s::pavo2s_driver::get_motor_speed(uint8_t & val)
{
    bool ret;
    unsigned char send_cmd[8];
    unsigned char recv_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_MOTOR_SPEED;
    uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, CMD_LEN);
    if (!ret) return false;
    //recvice the cmd
    ret = conn_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
    if (!ret) return false;
    //check the type and arg len
    ret = check_cmd_s(cmd_type, recv_cmd, recv_len);
    if (!ret) return false;
    //check the value
    val = (uint16_t)recv_cmd[CMD_LEN + 3];
    return ret;
}

bool pavo2s::pavo2s_driver::get_angle_resolution_enable(uint32_t & val)
{
    bool ret;
    unsigned char send_cmd[8];
    unsigned char recv_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_RESOLUTION_ENABLE;
    uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, CMD_LEN);
    if(!ret) return false;
    //recvice the cmd
    ret = conn_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
    if(!ret) return false;
    //check the type and arg len
    ret = check_cmd_s(cmd_type, recv_cmd, recv_len);
    if(!ret) return false;
    //check the value
    val = ((uint32_t)recv_cmd[CMD_LEN + 3]) + (((uint32_t)recv_cmd[CMD_LEN + 2]) << 8) + (((uint32_t)recv_cmd[CMD_LEN + 1]) << 16) + (((uint32_t)recv_cmd[CMD_LEN]) << 24);
    return ret;
}

bool pavo2s::pavo2s_driver::set_angle_resolution(uint16_t val)
{
    uint32_t enableValue;
    bool ret = get_angle_resolution_enable(enableValue);
    if(!ret)
        return false;

    uint16_t resBase = 8;
    std::set<uint16_t> setResEnable;
    for(int a = 0; a < 3; a++)
    {
        if(enableValue & 0x1)
        {
            setResEnable.insert(resBase);
        }
        enableValue = enableValue >> 1;
        resBase *= 2;
    }

    if(setResEnable.find(val) == setResEnable.end())
    {
        return false;
    }

    unsigned char send_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_ANGLE_RESOLUTION;
    uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
    memset(send_cmd, 0, send_len);
    //set the content
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
    send_cmd[CMD_LEN + 3] = 0xFF & (val);
    //set the send len
    send_len = CMD_LEN + PAVO2S_CMD_ARG_LEN[cmd_type];
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, send_len);
    if(!ret) return false;
    return ret;
}

bool pavo2s::pavo2s_driver::get_angle_resolution(uint16_t & val)
{
    bool ret;
    unsigned char send_cmd[8];
    unsigned char recv_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_ANGLE_RESOLUTION;
    uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, CMD_LEN);
    if (!ret) return false;
    //recvice the cmd
    ret = conn_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
    if (!ret) return false;
    //check the type and arg len
    ret = check_cmd_s(cmd_type, recv_cmd, recv_len);
    if (!ret) return false;
    //check the value
    val = (uint16_t)recv_cmd[CMD_LEN + 2] * 0x100 + (uint16_t)recv_cmd[CMD_LEN + 3];
    return ret;
}

bool pavo2s::pavo2s_driver::set_device_ip(std::string ip)
{
    bool ret;
    unsigned char send_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_DEV_IP;
    uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
    memset(send_cmd, 0, send_len);
    //set the content
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
    ret = string2Byte(ip, send_cmd + CMD_LEN);
    if (!ret) return false;
    //set the send len
    send_len = CMD_LEN + PAVO2S_CMD_ARG_LEN[cmd_type];
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, send_len);
    if (!ret) return false;
    ret = enable_Net_Unlock();
    if(!ret) return false;
    return ret;
}

bool pavo2s::pavo2s_driver::get_device_ip(std::string & ip)
{
    bool ret;
    unsigned char send_cmd[8];
    unsigned char recv_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_DEVICE_IP_READ;
    uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, CMD_LEN);
    if (!ret) return false;
    //recvice the cmd
    ret = conn_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
    if (!ret) return false;
    //check the type and arg len
    ret = check_cmd_s(cmd_type, recv_cmd, recv_len);
    if (!ret) return false;
    //check the value
    ret = bytes2String(ip, recv_cmd + CMD_LEN);
    return ret;
}

bool pavo2s::pavo2s_driver::set_dest_ip(std::string ip)
{
    bool ret;
    unsigned char send_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_DEST_IP;
    uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
    memset(send_cmd, 0, send_len);
    //set the content
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
    ret = string2Byte(ip, send_cmd + CMD_LEN);
    if(!ret) return false;
    //set the send len
    send_len = CMD_LEN + PAVO2S_CMD_ARG_LEN[cmd_type];
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, send_len);
    if(!ret) return false;
    ret = enable_Net_Unlock();
    if(!ret) return false;
    return ret;
}

bool pavo2s::pavo2s_driver::get_dest_ip(std::string & ip)
{
    bool ret;
    unsigned char send_cmd[8];
    unsigned char recv_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_DEST_IP_READ;
    uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, CMD_LEN);
    if(!ret) return false;
    //recvice the cmd
    ret = conn_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
    if(!ret) return false;
    //check the type and arg len
    ret = check_cmd_s(cmd_type, recv_cmd, recv_len);
    if(!ret) return false;
    //check the value
    ret = bytes2String(ip, recv_cmd + CMD_LEN);
    return ret;
}

bool pavo2s::pavo2s_driver::set_device_udp_port(uint16_t val)
{
    bool ret;
    unsigned char send_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_DEV_PORT;
    uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);
    memset(send_cmd, 0, send_len);
    //set the content
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
    send_cmd[CMD_LEN + 2] = 0xFF & (val >> 8);
    send_cmd[CMD_LEN + 3] = 0xFF & (val);
    //set the send len
    send_len = CMD_LEN + PAVO2S_CMD_ARG_LEN[cmd_type];
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, send_len);
    if (!ret) return false;
    ret = enable_Net_Unlock();
    if(!ret) return false;
    return ret;
}

bool pavo2s::pavo2s_driver::get_device_udp_port(uint16_t & val)
{
    bool ret;
    unsigned char send_cmd[8];
    unsigned char recv_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_PORT_READ;
    uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, CMD_LEN);
    if (!ret) return false;
    //recvice the cmd
    ret = conn_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
    if (!ret) return false;
    //check the type and arg len
    ret = check_cmd_s(cmd_type, recv_cmd, recv_len);
    if (!ret) return false;
    //check the value
    val = (uint16_t)recv_cmd[CMD_LEN + 0] * 0x100 + (uint16_t)recv_cmd[CMD_LEN + 1];
    return ret;
}

bool pavo2s::pavo2s_driver::set_dest_udp_port(uint16_t val)
{
    bool ret;
    unsigned char send_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_DEST_PORT;
    uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
    memset(send_cmd, 0, send_len);
    //set the content
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
    send_cmd[CMD_LEN + 2] = 0xFF & (val >> 8);
    send_cmd[CMD_LEN + 3] = 0xFF & (val);
    //set the send len
    send_len = CMD_LEN + PAVO2S_CMD_ARG_LEN[cmd_type];
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, send_len);
    if(!ret) return false;
    ret = enable_Net_Unlock();
    if(!ret) return false;
    return ret;
}

bool pavo2s::pavo2s_driver::get_dest_udp_port(uint16_t & val)
{
    bool ret;
    unsigned char send_cmd[8];
    unsigned char recv_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_PORT_READ;
    uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, CMD_LEN);
    if(!ret) return false;
    //recvice the cmd
    ret = conn_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
    if(!ret) return false;
    //check the type and arg len
    ret = check_cmd_s(cmd_type, recv_cmd, recv_len);
    if(!ret) return false;
    //check the value
    val = (uint16_t)recv_cmd[CMD_LEN + 2] * 0x100 + (uint16_t)recv_cmd[CMD_LEN + 3];
    return ret;
}

bool pavo2s::pavo2s_driver::reset_device_default()
{
    return false;
}

bool pavo2s::pavo2s_driver::set_os_restart()
{
    bool ret;
    unsigned char send_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_SLEEP_RESTART;
    uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
    memset(send_cmd, 0, send_len);
    //set the content
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
    send_cmd[CMD_LEN + 3] = 0x01;
    //set the send len
    send_len = CMD_LEN + PAVO2S_CMD_ARG_LEN[cmd_type];
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, send_len);
    if (!ret) return false;
    return ret;
}

bool pavo2s::pavo2s_driver::enable_tail_filter(uint16_t method)
{
    return conn_ptr_->enable_tail_filter(method);
}

bool pavo2s::pavo2s_driver::enable_front_filter(uint16_t iLevel)
{
    return conn_ptr_->enable_front_filter(iLevel);
}

bool pavo2s::pavo2s_driver::get_device_mac(std::string& mac)
{
    mac = "";
    return false;
}

bool pavo2s::pavo2s_driver::set_anti_disturb_level(uint8_t val)
{
    bool ret;
    unsigned char send_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_ANTI_DISTURB;
    uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
    memset(send_cmd, 0, send_len);
    //set the content
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
    send_cmd[CMD_LEN + 3] = 0xFF & (val);
    //set the send len
    send_len = CMD_LEN + PAVO2S_CMD_ARG_LEN[cmd_type];
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, send_len);
    if(!ret) return false;
    return ret;
}

bool pavo2s::pavo2s_driver::get_anti_disturb_level(uint8_t& val)
{
    bool ret;
    unsigned char send_cmd[8];
    unsigned char recv_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_ANTI_DISTURB;
    uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, CMD_LEN);
    if(!ret) return false;
    //recvice the cmd
    ret = conn_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
    if(!ret) return false;
    //check the type and arg len
    ret = check_cmd_s(cmd_type, recv_cmd, recv_len);
    if(!ret) return false;
    //check the value
    val = (uint16_t)recv_cmd[CMD_LEN + 3];
    return ret;
}

bool pavo2s::pavo2s_driver::set_echo_mode(bool en)
{
    bool ret;
    unsigned char send_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_ECHO_MODE;
    uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);;
    memset(send_cmd,0,send_len);
    memcpy(send_cmd,PAVO2S_CMD[cmd_type],CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
    send_len = CMD_LEN + PAVO2S_CMD_ARG_LEN[cmd_type];
    if(en) send_cmd[send_len - 1] = 0x01;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd,send_len);
    if(!ret) return false;
    return ret;
}

bool pavo2s::pavo2s_driver::get_echo_mode(bool& en)
{
   bool ret;
   unsigned char send_cmd[8];
   unsigned char recv_cmd[8];
   PAVO2S_CMD_INDEX cmd_type = S_CMD_ECHO_MODE;
   uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
   memcpy(send_cmd,PAVO2S_CMD[cmd_type],CMD_LEN);
   send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
   //send the cmd
   ret = conn_ptr_->sendCMD(send_cmd,CMD_LEN);
   if(!ret) return false;
   //recvice the cmd
   ret = conn_ptr_->recvCMD(cmd_type,recv_cmd,recv_len);
   if(!ret) return false;
   //check the type and arg len
   ret = check_cmd_s(cmd_type,recv_cmd,recv_len);
   if(!ret) return false;
   //check the value
   if(recv_cmd[recv_len - 1])
   {
    en = true;
   }
   else
   {
    en = false;
   }
   return ret;
}

std::string & pavo2s::pavo2s_driver::TrimSpace(std::string & str)
{
    // TODO: 在此处插入 return 语句
    if (str.empty())
    {
        return str;
    }
    str.erase(0, str.find_first_not_of(" "));  //trim white space
    str.erase(str.find_last_not_of(" ") + 1);

    str.erase(0, str.find_first_not_of("\t"));  //trim tab
    str.erase(str.find_last_not_of("\t") + 1);

    return str;
}

bool pavo2s::pavo2s_driver::string2Byte(const std::string & ip_str, unsigned char * ip_bytes)
{
    if (ip_str.empty())  //empty string
        return false;

    if (count(ip_str.begin(), ip_str.end(), '.') != 3)
        return false;

    std::string ip_str_copy(ip_str);
    TrimSpace(ip_str_copy); //remove leading and tail space

    std::string str_tmp;
    std::size_t prev_pos = 0;

    int dot_num = 0;
    int byte_single = 0;
    char tmp_bytes[4];

    for (std::size_t i = 0; i < ip_str_copy.length(); i++)
    {
        if (ip_str_copy[i] == '.')
        {
            if (dot_num++ > 3)
                return false;

            str_tmp = ip_str_copy.substr(prev_pos, i - prev_pos);
            prev_pos = i + 1;

            byte_single = atoi(str_tmp.c_str());  //invalid value
            if (byte_single > 255)
                return false;
            tmp_bytes[dot_num - 1] = static_cast<char>(byte_single);

            continue;
        }
        if ((ip_str_copy[i] < '0') || (ip_str_copy[i] > '9')) //invalid character
            return false;

    }

    str_tmp = ip_str_copy.substr(prev_pos);
    byte_single = atoi(str_tmp.c_str());
    tmp_bytes[dot_num] = static_cast<char>(byte_single);

    memcpy(ip_bytes, tmp_bytes, 4);

    return true;
}

bool pavo2s::pavo2s_driver::bytes2String(std::string & ip_str, const unsigned char * ip_bytes)
{
    if (ip_bytes == 0)
        return false;

    ip_str = std::to_string(static_cast<unsigned char>(ip_bytes[0])) + "." +
        std::to_string(static_cast<unsigned char>(ip_bytes[1])) + "." +
        std::to_string(static_cast<unsigned char>(ip_bytes[2])) + "." +
        std::to_string(static_cast<unsigned char>(ip_bytes[3]));
    return true;
}

inline bool pavo2s::pavo2s_driver::check_cmd_s(PAVO2S_CMD_INDEX cmd, unsigned char * content, uint16_t size)
{
    content[CMD_SET_GET_INDEX] = 0x00;
    if(size < CMD_LEN) return false;
    if(memcmp(content, PAVO2S_CMD[cmd], CMD_LEN))return false;
    uint16_t arg_len;
    arg_len = size - CMD_LEN;
    if(arg_len != PAVO2S_CMD_ARG_LEN[cmd]) return false;
    return true;
}

bool pavo2s::pavo2s_driver::enable_Net_Unlock()
{
    bool ret;
    unsigned char send_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_NET_UNLOCK;;
    memset(send_cmd, 0, 8);
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_SET;
    send_cmd[7] = 0x01;
    uint16_t send_len = sizeof(send_cmd) / sizeof(unsigned char);
    ret = conn_ptr_->sendCMD(send_cmd,send_len);
    return ret;
}

bool pavo2s::pavo2s_driver::get_heartbeat()
{
    bool ret;
    unsigned char send_cmd[8];
    unsigned char recv_cmd[8];
    PAVO2S_CMD_INDEX cmd_type = S_CMD_SN;
    uint16_t recv_len = sizeof(recv_cmd) / sizeof(unsigned char);
    memcpy(send_cmd, PAVO2S_CMD[cmd_type], CMD_LEN);
    send_cmd[CMD_SET_GET_INDEX] = CMD_GET;
    //send the cmd
    ret = conn_ptr_->sendCMD(send_cmd, CMD_LEN);
    if(!ret) return false;
    //recvice the cmd
    ret = conn_ptr_->recvCMD(cmd_type,recv_cmd,recv_len,1000);
    if(!ret) return false;
    //check the type and arg len
    ret = check_cmd_s(cmd_type, recv_cmd, recv_len);
    if(!ret) return false;
    return ret;
}

bool pavo2s::pavo2s_driver::send_cmd_asynch(unsigned char* content, uint16_t size)
{
    bool ret = false;
    ret = conn_ptr_->sendCMD(content,size);
    return ret;
}

bool pavo2s::pavo2s_driver::recv_cmd_asynch(PAVO2S_CMD_INDEX cmd,unsigned char* content, uint16_t& size)
{
    bool ret = false;
    ret = conn_ptr_->recvCMDNoWait(cmd,content,size);
    if(!ret) return false;
    ret = check_cmd_s(cmd,content,size);
    return ret;
}
