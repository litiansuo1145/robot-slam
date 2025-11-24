#ifndef __PAVO2S_DRIVER_H__
#define __PAVO2S_DRIVER_H__

#include <string>
#include <set>
#include <vector>

#include "pavo2s_types.h"
#include "pavo2s_netudp.h"

#define SDK_VER "2.1.0"

namespace pavo2s
{
class NetConnect;
class pavo2s_driver
{
public:
    pavo2s_driver(uint16_t dest_port);
    ~pavo2s_driver();
    bool pavo2s_open(std::string lidar_ip, uint16_t lidar_port);
    bool pavo2s_is_open();
    bool pavo2s_close();
    bool get_scanned_data(arr_pavo_packet_info_scan_t& arr_packet_info,uint16_t timeout = 0);
    bool get_scanned_data(vec_pavo_packet_info_scan_t& vtr_packet_info,uint16_t timeout = 0);
    bool get_scanned_data(arr_pavo_packet_info_pcd_t& arr_packet_info,uint16_t timeout = 0);
    bool get_scanned_data(vec_pavo_packet_info_pcd_t& vtr_packet_info,uint16_t timeout = 0);
    bool get_device_sn(std::string& sn);
    bool get_device_pn(std::string& pn);
    bool get_sdk_ver(std::string& ver);
    bool get_firmware_ver(std::string& ver);
    bool get_os_error(uint32_t& errCode,uint8_t iIndex = 0);
    int  get_error_code();
    bool enable_data(bool en);
    bool set_angle_range(uint16_t start, uint16_t end);
    bool get_angle_range(uint16_t & start, uint16_t & end);
    bool get_motor_speed_enable(uint32_t& val);
    bool set_motor_speed(uint8_t val);
    bool get_motor_speed(uint8_t & val);
    bool get_angle_resolution_enable(uint32_t & val);
    bool set_angle_resolution(uint16_t val);
    bool get_angle_resolution(uint16_t& val);
    bool set_device_ip(std::string ip);
    bool get_device_ip(std::string & ip);
    bool set_dest_ip(std::string ip);
    bool get_dest_ip(std::string & ip);
    bool set_device_udp_port(uint16_t val);
    bool get_device_udp_port(uint16_t & val);
    bool set_dest_udp_port(uint16_t val);
    bool get_dest_udp_port(uint16_t & val);
    bool reset_device_default();
    bool set_os_restart();
    bool enable_tail_filter(uint16_t method);
    bool get_device_mac(std::string& mac);
    bool set_anti_disturb_level(uint8_t val);
    bool get_anti_disturb_level(uint8_t& val);
    bool set_echo_mode(bool en);
    bool get_echo_mode(bool& en);
    bool enable_front_filter(uint16_t iLevel);
    bool get_heartbeat();
    bool send_cmd_asynch(unsigned char* content,uint16_t size);
    bool recv_cmd_asynch(PAVO2S_CMD_INDEX cmd,unsigned char* content,uint16_t& size);
protected:
        NetConnectUDP * conn_ptr_;
private:
    std::string& TrimSpace(std::string &str);
    bool string2Byte(const std::string& ip_str, unsigned char* ip_bytes);
    bool bytes2String(std::string& ip_str, const unsigned char* ip_bytes);
    bool enable_Net_Unlock();
    inline bool check_cmd_s(PAVO2S_CMD_INDEX cmd, unsigned char * content, uint16_t size);
};
}
#endif
