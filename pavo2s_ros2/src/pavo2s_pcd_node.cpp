#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_srvs/srv/empty.hpp"

#include "pavo2s_driver.h"
#include "pavo2s_utils.h"

#define DISTANCES_UINT (0.002f)

#define SCAN_PUB_BUFFER_SIZE (10)
#define DEF_TIMEOUT (180)


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node_handle = rclcpp::Node::make_shared("pavo2s_pcd_node");
    std::string frame_id;
    std::string topic;

    bool inverted;
    int timeout;

    //----------------------------declare parameters----------------------
    node_handle->declare_parameter<std::string>("frame_id", "pavo2s_frame");
    node_handle->declare_parameter<std::string>("topic", "pavo2s_pcd");

    node_handle->declare_parameter<bool>("inverted", false);
    node_handle->declare_parameter<int>("timeout", DEF_TIMEOUT);

    //------------------------------get parameters---------------------------------
    node_handle->get_parameter_or<std::string>("frame_id", frame_id, "pavo2s_frame");
    node_handle->get_parameter_or<std::string>("topic", topic, "pavo2s_pcd");

    node_handle->get_parameter_or<bool>("inverted", inverted, false);
    node_handle->get_parameter_or<int>("timeout", timeout, DEF_TIMEOUT);

    //---------------Initialize Lidar-----------------------------------
    std::unique_ptr<pavo2s::pavo2s_driver> drv = init_lidar(node_handle);
    if(!drv)
    {
        RCLCPP_FATAL(node_handle->get_logger(), "Initialize Lidar Error. Exit!");
        return -1;
    }


    //------------------frame id, topic---------------
    RCLCPP_INFO(node_handle->get_logger(), "Using frame_id [%s].", frame_id.c_str());
    RCLCPP_INFO(node_handle->get_logger(), "Using topic [%s].", topic.c_str());


    //--------------------inverted----------------------------------
    const char* inverted_str = inverted ? "true" : "false";
    RCLCPP_INFO(node_handle->get_logger(), "Using inverted [%s].", inverted_str);


    //---------------------------timeout------------------------
    if(timeout < 0)
    {
        RCLCPP_WARN(node_handle->get_logger(), "Invalid parameter timeout [%d], use default value[%d].",
                    timeout, DEF_TIMEOUT);
        timeout = DEF_TIMEOUT;
    }
    else
    {
        RCLCPP_INFO(node_handle->get_logger(), "Using timeout [%d].", timeout);
    }


    //----------------------prepare for publish----------------------------------
    if(!drv->enable_data(true))
    {
        RCLCPP_FATAL(node_handle->get_logger(), "Enable data sending error. Exit!");
        return -1;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr
            cloud_pub = node_handle->create_publisher<sensor_msgs::msg::PointCloud>(topic, SCAN_PUB_BUFFER_SIZE);

    while (rclcpp::ok())
    {
        sensor_msgs::msg::PointCloud cloud;
        vec_pavo_packet_info_pcd_t pcdInfo;

        if(!drv->get_scanned_data(pcdInfo, timeout))
        {
            RCLCPP_ERROR(node_handle->get_logger(), "Get scanned data error!");
            continue;
        }

        std::size_t num_points = pcdInfo.vec_buff.size();
        if(num_points == 0)
        {
            RCLCPP_ERROR(node_handle->get_logger(), "Scan data empty!");
            continue;
        }

        cloud.header.stamp = node_handle->now();
        cloud.header.frame_id = frame_id;
        cloud.points.resize(num_points);
        cloud.channels.resize(1);
        cloud.channels[0].name = "intensities";
        cloud.channels[0].values.resize(num_points);
        if (!inverted)
        {
            for(std::size_t i = 0; i < num_points; i++)
            {
                cloud.points[i].x = -pcdInfo.vec_buff[i].x * DISTANCES_UINT;
                cloud.points[i].y = -pcdInfo.vec_buff[i].y * DISTANCES_UINT;
                cloud.points[i].z = pcdInfo.vec_buff[i].z;
                cloud.channels[0].values[i] = pcdInfo.vec_buff[i].intensity;
            }
        }
        else
        {
            for(std::size_t i = 0; i < num_points; i++)
            {
                std::size_t inverted_index = num_points-1-i;
                cloud.points[inverted_index].x = -pcdInfo.vec_buff[i].x * DISTANCES_UINT;
                cloud.points[inverted_index].y = pcdInfo.vec_buff[i].y * DISTANCES_UINT;
                cloud.points[inverted_index].z = pcdInfo.vec_buff[i].z;
                cloud.channels[0].values[inverted_index] = pcdInfo.vec_buff[i].intensity;
            }
        }
        cloud_pub->publish(cloud);
        rclcpp::spin_some(node_handle);
    }
    return 0;
}

