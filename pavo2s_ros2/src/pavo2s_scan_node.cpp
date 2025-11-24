#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"

#include "pavo2s_driver.h"
#include "pavo2s_utils.h"

#define DISTANCES_UINT (0.002f)

#define DISTANCE_RANGE_MIN (0.10f)
#define DISTANCE_RANGE_MAX (35.0f)

#define SCAN_PUB_BUFFER_SIZE (10)
#define DEF_TIMEOUT (180)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif // !M_PI

#define DEG2RAD(x) ((x) * M_PI / 180)


void publish_msg(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub,
                 std::vector<pavo_response_scan_t> &nodes_vec, rclcpp::Time start, double scan_time,
                 std::string frame_id, bool inverted, double min_range, double max_range)
{
    sensor_msgs::msg::LaserScan scanMsg;
    std::size_t counts = nodes_vec.size();
    scanMsg.ranges.resize(counts);
    scanMsg.intensities.resize(counts);

    float range = 0.0;
    float intensity = 0.0;

    for (std::size_t i = 0; i < counts; i++)
    {
       range = nodes_vec[i].distance * DISTANCES_UINT;
       intensity = nodes_vec[i].intensity;
       if ((range > max_range) || (range < min_range))
       {
           range = 0.0;
           intensity = 0.0;
       }
       if (!inverted)
       {
           scanMsg.ranges[i] = range;
           scanMsg.intensities[i] = intensity;
       }
       else
       {
           scanMsg.ranges[counts - 1 - i] = range;
           scanMsg.intensities[counts - 1 - i] = intensity;
       }
  }
  scanMsg.header.stamp = start;
  scanMsg.header.frame_id = frame_id;
  scanMsg.angle_min = DEG2RAD(nodes_vec[0].angle / 100.0f) - M_PI;
  scanMsg.angle_max = DEG2RAD(nodes_vec[counts - 1].angle / 100.0f) - M_PI;
  scanMsg.angle_increment = (scanMsg.angle_max - scanMsg.angle_min) / (counts - 1);
  scanMsg.scan_time = scan_time;
  scanMsg.time_increment = scan_time / (counts - 1);
  scanMsg.range_min = min_range;
  scanMsg.range_max = max_range;
  scan_pub->publish(scanMsg);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node_handle = rclcpp::Node::make_shared("pavo2s_scan_node");
    std::string frame_id, scan_topic;

    bool inverted;

    double range_max, range_min;
    int timeout;

    //--------------------declare prameters----------------------
    node_handle->declare_parameter<std::string>("frame_id", "pavo2s_frame");
    node_handle->declare_parameter<std::string>("topic", "pavo2s_scan");

    node_handle->declare_parameter<double>("range_min", DISTANCE_RANGE_MIN);
    node_handle->declare_parameter<double>("range_max", DISTANCE_RANGE_MAX);


    node_handle->declare_parameter<bool>("inverted", false);
    node_handle->declare_parameter<int>("timeout", DEF_TIMEOUT);


    //---------------------get parameters-----------------------------
    node_handle->get_parameter_or<std::string>("frame_id", frame_id, "pavo2s_frame");
    node_handle->get_parameter_or<std::string>("topic", scan_topic, "pavo2s_scan");

    node_handle->get_parameter_or<double>("range_min", range_min, DISTANCE_RANGE_MIN);
    node_handle->get_parameter_or<double>("range_max", range_max, DISTANCE_RANGE_MAX);

    node_handle->get_parameter_or<bool>("inverted", inverted, false);
    node_handle->get_parameter_or<int>("timeout", timeout, DEF_TIMEOUT);


    //---------------Initialize Lidar-----------------------------------
    std::unique_ptr<pavo2s::pavo2s_driver> drv = init_lidar(node_handle);
    if(!drv)
    {
        RCLCPP_FATAL(node_handle->get_logger(), "Initialize Lidar Error. Exit!");
        return -1;
    }

    //------------------frame id, topic---------------------------
    RCLCPP_INFO(node_handle->get_logger(), "Using frame_id [%s].", frame_id.c_str());
    RCLCPP_INFO(node_handle->get_logger(), "Using scan_topic [%s].", scan_topic.c_str());


    //--------------------Range Check-----------------------------
    if(!bCheckMin(range_min, DISTANCE_RANGE_MIN))
    {
        RCLCPP_WARN(node_handle->get_logger(), "Invalid parameter range_min [%f], use default value [%f].",
                    range_min, DISTANCE_RANGE_MIN);
        range_min = DISTANCE_RANGE_MIN;
    }

    if(!bCheckMax(range_max, DISTANCE_RANGE_MAX))
    {
        RCLCPP_WARN(node_handle->get_logger(), "Invalid parameter range_max [%f], use default value [%f].",
                    range_max, DISTANCE_RANGE_MAX);
        range_max = DISTANCE_RANGE_MAX;
    }

    if(range_min >= range_max)
    {
        RCLCPP_WARN(node_handle->get_logger(), "range_min[%f] is greater than range_max [%f]. \
                    Use default value: [%f, %f].",
                    range_min, range_max, DISTANCE_RANGE_MIN, DISTANCE_RANGE_MAX);
        range_min = DISTANCE_RANGE_MIN;
        range_max = DISTANCE_RANGE_MAX;
    }
    RCLCPP_INFO(node_handle->get_logger(), "Using range [%f, %f] for distance.", range_min, range_max);


    //----------------inverted------------
    const char* inverted_str = inverted ? "true" : "false";
    RCLCPP_INFO(node_handle->get_logger(), "Using inverted [%s].", inverted_str);


    //---------------------timeout--------------
    if(timeout < 0)
    {
        RCLCPP_WARN(node_handle->get_logger(), "Invalid parameter timeout [%d], use default value [%d].",
                      timeout, DEF_TIMEOUT);
        timeout = DEF_TIMEOUT;
    }
    else
    {
        RCLCPP_INFO(node_handle->get_logger(), "Using timeout [%d].", timeout);
    }

   
    //---------------------prepare for publish------------
    uint8_t motor_speed;
    if(!drv->get_motor_speed(motor_speed))
    {
        RCLCPP_FATAL(node_handle->get_logger(), "Read motor_speed error. Please Check Link. Exit!");
        return -1;
    }

    if(!drv->enable_data(true))
    {
        RCLCPP_FATAL(node_handle->get_logger(), "Enable data sending error. Exit!");
        return -1;
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
            scan_pub  = node_handle->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, SCAN_PUB_BUFFER_SIZE);

    double scan_duration = 1.0f / motor_speed;

    while (rclcpp::ok())
    {
        rclcpp::Time start_scan_time = node_handle->now();
        vec_pavo_packet_info_scan_t scanInfo;

        if(!drv->get_scanned_data(scanInfo, timeout))
        {
            RCLCPP_ERROR(node_handle->get_logger(), "Get scanned data error!");
            continue;
        }

        if(scanInfo.vec_buff.size() < 2)
        {
            RCLCPP_ERROR(node_handle->get_logger(), "Scan data empty!");
            continue;
        }

        publish_msg(scan_pub,
                    scanInfo.vec_buff, start_scan_time, scan_duration,
                    frame_id, inverted, range_min, range_max);
        rclcpp::spin_some(node_handle);
    }

    return 0;
}

