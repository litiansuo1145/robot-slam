#include "pavo2s_utils.h"

bool bCheckMin(int sample, int standard)
{
    return (sample >= standard);
}

bool bCheckMax(int sample, int standard)
{
    return (sample <= standard);
}

bool bCheckMin(double sample, double standard)
{
    return (sample  >= standard - 0.0001);
}

bool bCheckMax(double sample, double standard)
{
    return (sample <= standard + 0.0001);
}

bool bCheckPort(int iPort)
{
    return ((iPort >= 1024) && (iPort < 65535));
}

#define IP_ERROR_DIGIT (1)
#define IP_ERROR_SEGMENT (2)
#define IP_ERROR_ILLEGAL (3)

bool bCheckIP(std::string strIP, int& iErrorType)
{
    for(std::size_t m = 0; m < strIP.length(); m++) //invalid character
    {
        if((strIP[m] == '.') || ((strIP[m] >= '0') && (strIP[m] <= '9')))
            continue;
        
        iErrorType = IP_ERROR_DIGIT;
        return false;
    }
    char delim = '.';
    std::size_t previous = 0;
    std::size_t current = strIP.find(delim);
    std::vector<std::string> elems;
    while (current != std::string::npos)
    {
        if (current > previous)
        {
            elems.push_back(strIP.substr(previous, current - previous));
        }
        previous = current + 1;
        current = strIP.find(delim, previous);
    }
    if (previous != strIP.size())
    {
        elems.push_back(strIP.substr(previous));
    }

    if(elems.size() != 4) //segment
    {
        iErrorType = IP_ERROR_SEGMENT;
        return false;
    }
    
    unsigned int iIPTemp[4] = {0};
    for(int m = 0; m < 4; m++)
    {
        iIPTemp[m] = atoi(elems[m].c_str());
    }

    if((iIPTemp[0] == 0) || (iIPTemp[0] == 127)
        || ((iIPTemp[0] >= 224) && (iIPTemp[0] <= 255))) //Illegal
    {
        iErrorType = IP_ERROR_ILLEGAL;
        return false;
    }

    if((iIPTemp[0] == 128) && (iIPTemp[1] == 0))
    {
        iErrorType = IP_ERROR_ILLEGAL;
        return false;
    }
    
    if((iIPTemp[0] == 191) && (iIPTemp[1] == 255))
    {
        iErrorType = IP_ERROR_ILLEGAL;
        return false;
    }
    
    if((iIPTemp[0] == 169) && (iIPTemp[1] == 254))
    {
        iErrorType = IP_ERROR_ILLEGAL;
        return false;
    }
    
    if((iIPTemp[0] == 192) && (iIPTemp[1] == 0) && (iIPTemp[2] == 0))
    {
        iErrorType = IP_ERROR_ILLEGAL;
        return false;
    }
    
    if((iIPTemp[0] == 223) && (iIPTemp[1] == 255) && (iIPTemp[2] == 255))
    {
        iErrorType = IP_ERROR_ILLEGAL;
        return false;
    }
    
    if(iIPTemp[3] == 255)
    {
        iErrorType = IP_ERROR_ILLEGAL;
        return false;
    }

    return true;
}

#define DEF_ANGLE_START (4000)
#define DEF_ANGLE_END (32000)

#define ANGLE_RANGE_MIN (0)
#define ANGLE_RANGE_MAX (32000)

#define TAIL_FILTER_LEVEL_MIN (0)
#define TAIL_FILTER_LEVEL_MAX (3)

#define FRONT_FILTER_LEVEL_MIN (0)
#define FRONT_FILTER_LEVEL_MAX (3)

#define ANTI_DISTURB_LEVEL_MIN (0)
#define ANTI_DISTURB_LEVEL_MAX (2)

#define DEF_MOTOR_SPEED (25)
#define DEF_RESOLUTION (16)

std::unique_ptr<pavo2s::pavo2s_driver> init_lidar(rclcpp::Node::SharedPtr node_handle)
{
    //--------------node parameters----------------------------------
    std::string lidar_ip;
    int lidar_port;
    int dest_port;

    int angle_start, angle_end;
    int motor_speed;
    int resolution;
    bool echo_mode;

    int tail_filter, front_filter, anti_disturb;

    //----------------------declare parameters----------------------------------
    node_handle->declare_parameter<std::string>("lidar_ip", "10.10.10.101");
    node_handle->declare_parameter<int>("lidar_port", 2368);
    node_handle->declare_parameter<int>("dest_port", 2368);

    node_handle->declare_parameter<int>("angle_start", DEF_ANGLE_START);
    node_handle->declare_parameter<int>("angle_end",  DEF_ANGLE_END);

    node_handle->declare_parameter<int>("motor_speed", DEF_MOTOR_SPEED);
    node_handle->declare_parameter<int>("resolution", DEF_RESOLUTION);

    node_handle->declare_parameter<bool>("echo_mode", true);
    node_handle->declare_parameter<int>("tail_filter", TAIL_FILTER_LEVEL_MIN);
    node_handle->declare_parameter<int>("front_filter", FRONT_FILTER_LEVEL_MIN);
    node_handle->declare_parameter<int>("anti_disturb", ANTI_DISTURB_LEVEL_MIN);


    //------------------------get parameters----------------------------------------
    node_handle->get_parameter_or<std::string>("lidar_ip", lidar_ip, "10.10.10.121");
    node_handle->get_parameter_or<int>("lidar_port", lidar_port, 2368);
    node_handle->get_parameter_or<int>("dest_port", dest_port, 2368);

    node_handle->get_parameter_or<int>("angle_start", angle_start, DEF_ANGLE_START);
    node_handle->get_parameter_or<int>("angle_end", angle_end, DEF_ANGLE_END);

    node_handle->get_parameter_or<int>("motor_speed", motor_speed, DEF_MOTOR_SPEED);
    node_handle->get_parameter_or<int>("resolution", resolution, DEF_RESOLUTION);
    node_handle->get_parameter_or<bool>("echo_mode", echo_mode, true);

    node_handle->get_parameter_or<int>("tail_filter", tail_filter, TAIL_FILTER_LEVEL_MIN);
    node_handle->get_parameter_or<int>("front_filter", front_filter, FRONT_FILTER_LEVEL_MIN);
    node_handle->get_parameter_or<int>("anti_disturb", anti_disturb, ANTI_DISTURB_LEVEL_MIN);

    bool set_ret = false;
    //----------------ip, port, udp server----------------------
    int iErrorType = 0;
    if(!bCheckIP(lidar_ip, iErrorType))
    {
        if((iErrorType == IP_ERROR_DIGIT) || (iErrorType == IP_ERROR_SEGMENT))
        {
            RCLCPP_FATAL(node_handle->get_logger(), "Incorrect IP address format! lidar_ip [%s]. Exit!",
                         lidar_ip.c_str());
        }
        else if(iErrorType == IP_ERROR_ILLEGAL)
        {
            RCLCPP_FATAL(node_handle->get_logger(), "IP addresses cannot be special addresses! lidar_ip [%s]. Exit",
                         lidar_ip.c_str());
        }
        return nullptr;
    }

    if(!bCheckPort(lidar_port))
    {
        RCLCPP_FATAL(node_handle->get_logger(), "Invalid parameter lidar_port [%d]. Exit!",
                     lidar_port);
        return nullptr;
    }

    if(!bCheckPort(dest_port))
    {
        RCLCPP_FATAL(node_handle->get_logger(), "Invalid parameter dest_port [%d]. Exit!",
                     lidar_port);
        return nullptr;
    }

    std::unique_ptr<pavo2s::pavo2s_driver> drv(new pavo2s::pavo2s_driver(dest_port));
    if(!drv)
    {
        RCLCPP_FATAL(node_handle->get_logger(), "Create pavo2s driver error!");
        return nullptr;
    }
    else
    {
        RCLCPP_INFO(node_handle->get_logger(), "Create pavo2s driver success.");
    }


    RCLCPP_INFO(node_handle->get_logger(), "Start UDP Server for Lidar: IP[%s], Port[%d].",
                lidar_ip.c_str(), lidar_port);
    if(drv->pavo2s_open(lidar_ip, lidar_port))
    {
        RCLCPP_INFO(node_handle->get_logger(),
                    "  UDP Server Started successfully, ready to receive data.");
    }
    else
    {
        RCLCPP_FATAL(node_handle->get_logger(),
                     "  UDP Server Started incorrectly, Exit!");
        return nullptr;
    }


    //--------------------------------angle_range------------------------------
    uint16_t angle_start_get, angle_end_get;
    bool angle_range_func_en = drv->get_angle_range(angle_start_get, angle_end_get);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if(angle_range_func_en)
    {
        if(!bCheckMin(angle_start, ANGLE_RANGE_MIN))
        {
            RCLCPP_WARN(node_handle->get_logger(),
                        "Invalid parameter angle_start [%d], use default value [%d].",
                        angle_start, DEF_ANGLE_START);
            angle_start = DEF_ANGLE_START;
        }

        if(!bCheckMax(angle_end, ANGLE_RANGE_MAX))
        {
            RCLCPP_WARN(node_handle->get_logger(),
                        "Invalid parameter angle_end [%d], use default value [%d].",
                        angle_end, DEF_ANGLE_END);
            angle_end = DEF_ANGLE_END;
        }

        if(angle_start >= angle_end)
        {
            RCLCPP_WARN(node_handle->get_logger(),
                        "Invalid parameter angle_start [%d] is greater angle_end [%d], use default value.",
                        angle_start, angle_end);
            angle_start = DEF_ANGLE_START;
            angle_end = DEF_ANGLE_END;
        }

        set_ret = drv->set_angle_range(uint16_t(angle_start), uint16_t(angle_end));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if(!drv->get_angle_range(angle_start_get, angle_end_get))
        {
            RCLCPP_FATAL(node_handle->get_logger(),
                         "Read angle range error. Please Check Link. Exit!");
            return nullptr;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if(set_ret && (angle_start_get == uint16_t(angle_start)) && (angle_end_get == uint16_t(angle_end)))
        {
            RCLCPP_INFO(node_handle->get_logger(),
                        "Set angle range [%d, %d] success.",
                        angle_start, angle_end);
        }
        else
        {
            RCLCPP_WARN(node_handle->get_logger(),
                        "Set angle range [%d, %d] failed! Continue with values previously set in device [%d, %d].",
                        angle_start, angle_end, angle_start_get, angle_end_get);
        }
    }


    //--------------------------------motor speed-------------------------------
    uint8_t motor_speed_prev = 0;

    if(!drv->get_motor_speed(motor_speed_prev))
    {
        RCLCPP_FATAL(node_handle->get_logger(),
                     "Read motor_speed error. Please Check Link. Exit!");
        return nullptr;
    }

    if(motor_speed_prev != uint8_t(motor_speed))
    {
        set_ret = drv->set_motor_speed(uint8_t(motor_speed));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        uint8_t motor_speed_get = 0;
        if(!drv->get_motor_speed(motor_speed_get))
        {
            RCLCPP_FATAL(node_handle->get_logger(),
                         "Read motor_speed error. Please Check Link. Exit!");
            return nullptr;
        }

        if(set_ret && (motor_speed_get == uint8_t(motor_speed)))
        {
            RCLCPP_INFO(node_handle->get_logger(),
                        "Set motor_speed [%d] success.",
                        motor_speed);
            RCLCPP_INFO(node_handle->get_logger(),
                        "    motor_speed updated. Wait for stablization of motor speed....");
            std::this_thread::sleep_for(std::chrono::seconds(100));
        }
        else
        {
            RCLCPP_WARN(node_handle->get_logger(),
                        "Set motor_speed [%d] failed. Continue with value [%d] previously set in device.",
                        motor_speed, motor_speed_prev);
        }
    }
    else
    {
        RCLCPP_INFO(node_handle->get_logger(),
                    "Set motor_speed [%d] success.",
                    motor_speed);
    }


    //--------------------------------------resolution--------------------------------------------
    uint16_t resolution_get;
    set_ret = drv->set_angle_resolution(uint16_t(resolution));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if(!drv->get_angle_resolution(resolution_get))
    {
        RCLCPP_FATAL(node_handle->get_logger(),
                     "Read resolution error. Please Check Link. Exit!");
        return nullptr;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if(set_ret && (resolution_get == uint16_t(resolution)))
    {
        RCLCPP_INFO(node_handle->get_logger(),
                    "Set resolution [%d] success.",
                    resolution);
    }
    else
    {
        RCLCPP_WARN(node_handle->get_logger(),
                    "Set resolution [%d] failed. Continue with value [%d] previously set in device.",
                    resolution, resolution_get);
    }


    //add in Protocol v2.3, not exit if read back error
    //----------------echo mode----------------------------
    bool echo_mode_get;
    bool echo_mode_func_en = drv->get_echo_mode(echo_mode_get);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if(echo_mode_func_en)
    {
        const char* echo_mode_str = echo_mode ? "true" : "false";

        set_ret = drv->set_echo_mode(echo_mode);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(!drv->get_echo_mode(echo_mode_get))
        {
            RCLCPP_FATAL(node_handle->get_logger(),
                         "Read echo_mode error. Please Check Link. Exit!");
            return nullptr;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if(set_ret && (echo_mode_get == echo_mode))
        {
            RCLCPP_INFO(node_handle->get_logger(), "Set echo_mode [%s] success.",
                        echo_mode_str);
        }
        else
        {
            const char* echo_mode_get_str = echo_mode_get ? "true" : "false";
            RCLCPP_WARN(node_handle->get_logger(),
                        "Set echo_mode [%s] failed! Continue with value [%s] previously set in device.",
                        echo_mode_str, echo_mode_get_str);
        }
    }


    //-----------------filters --------------------
    if((tail_filter > TAIL_FILTER_LEVEL_MAX) || (tail_filter < TAIL_FILTER_LEVEL_MIN))
    {
        RCLCPP_WARN(node_handle->get_logger(),
                    "Invalid parameter tail_filter [%d], use default value [%d].",
                    tail_filter, TAIL_FILTER_LEVEL_MIN);
        tail_filter = TAIL_FILTER_LEVEL_MIN;
    }
    else
    {
        RCLCPP_INFO(node_handle->get_logger(), "Using tail_filter [%d].", tail_filter);
    }
    drv->enable_tail_filter(tail_filter);

    if((front_filter > FRONT_FILTER_LEVEL_MAX) || (front_filter < FRONT_FILTER_LEVEL_MIN))
    {
        RCLCPP_WARN(node_handle->get_logger(),
                    "Invalid parameter front_filter [%d], use default value [%d].",
                    front_filter, FRONT_FILTER_LEVEL_MIN);
        front_filter = FRONT_FILTER_LEVEL_MIN;
    }
    else
    {
        RCLCPP_INFO(node_handle->get_logger(), "Using front_filter [%d].",
                    front_filter);
    }
    drv->enable_front_filter(front_filter);

    if((anti_disturb > ANTI_DISTURB_LEVEL_MAX) || (anti_disturb < ANTI_DISTURB_LEVEL_MIN))
    {
        RCLCPP_WARN(node_handle->get_logger(),
                    "Invalid parameter hr_filter [%d], use default value [%d].",
                    anti_disturb, ANTI_DISTURB_LEVEL_MIN);
        anti_disturb = ANTI_DISTURB_LEVEL_MIN;
    }
    else
    {
        RCLCPP_INFO(node_handle->get_logger(), "Using hr_filter [%d].", anti_disturb);
    }
    drv->set_anti_disturb_level(anti_disturb);

    return drv;
}

