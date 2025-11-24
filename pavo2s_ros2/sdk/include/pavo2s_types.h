#ifndef __PAVO2S_TYPES_H__
#define __PAVO2S_TYPES_H__

#include <cmath>
#include <vector>
#include <cstdint>

#if defined(_WIN32)
#pragma pack(1)
#define __DATA_ALIGN__  
#else
#define __DATA_ALIGN__ __attribute__((packed))
#endif


typedef struct pavo_response_scan
{
	uint16_t angle;
	uint16_t distance;
	uint16_t  intensity;
} __DATA_ALIGN__ pavo_response_scan_t;

typedef struct pavo_response_pcd
{
	double x;
	double y;
	double z;  //fixed as 0 for single-laser device
	uint16_t  intensity;
} __DATA_ALIGN__ pavo_response_pcd_t;

typedef struct {
	uint32_t data_tag;
	uint32_t packet_id;
	uint32_t reserver;
	uint16_t data_type;
	uint16_t angle_resolution;
	uint16_t start_angle;
	uint16_t end_angle;
	uint32_t time_stamp;
} __DATA_ALIGN__ data_info_t;

typedef struct arr_pavo_packet_info_scan
{
    pavo_response_scan_t* data_buffer;
    uint16_t count;
    uint32_t time_stamp;
} __DATA_ALIGN__ arr_pavo_packet_info_scan_t;

typedef struct vec_pavo_packet_info_scan
{

    vec_pavo_packet_info_scan()
    {

    }
    vec_pavo_packet_info_scan(vec_pavo_packet_info_scan&& other)
    {
        this->vec_buff = std::move(other.vec_buff);
        this->time_stamp = other.time_stamp;
    }
    vec_pavo_packet_info_scan& operator=(vec_pavo_packet_info_scan&& other)
    {
        this->vec_buff = std::move(other.vec_buff);
        this->time_stamp = other.time_stamp;
        return *this;
    }
    std::vector<pavo_response_scan_t> vec_buff;
    uint32_t time_stamp;
}vec_pavo_packet_info_scan_t;

typedef struct arr_pavo_packet_info_pcd
{
    pavo_response_pcd_t* data_buffer;
    uint16_t count;
    uint32_t time_stamp;
} __DATA_ALIGN__ arr_pavo_packet_info_pcd_t;

typedef struct vec_pavo_packet_info_pcd
{
    std::vector<pavo_response_pcd_t> vec_buff;
    uint32_t time_stamp;
}vec_pavo_packet_info_pcd_t;

//when the distance is 0 ,set the output value is 0 or 50000(100m) that is out of the valid range 
#define DISTANCE_ZERO 0
#if DISTANCE_ZERO
const int distance_max = 50000; //100m
#else
const int distance_max = 0; //0m
#endif

#if defined(_WIN32)
#pragma pack()
#endif

#endif //__PAVO2S_TYPES_H__

