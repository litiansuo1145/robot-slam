#ifndef  _PAVO2S_UTILS_H_
#define  _PAVO2S_UTILS_H_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "pavo2s_driver.h"


bool bCheckMin(int sample, int standard);

bool bCheckMax(int sample, int standard);

bool bCheckMin(double sample, double standard);

bool bCheckMax(double sample, double standard);

bool bCheckPort(int iPort);

bool bCheckIP(std::string strIP, int& iErrorType);

std::unique_ptr<pavo2s::pavo2s_driver> init_lidar(rclcpp::Node::SharedPtr node_handle);

#endif   //_PAVO2S_UTILS_H_
