/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-11-20
 ****************************************************************/

#ifndef HEX_EXAMPLE_DATA_INTERFACE_DATA_INTERFACE_H_
#define HEX_EXAMPLE_DATA_INTERFACE_DATA_INTERFACE_H_

#if HEX_ROS_VERSION == 1
#include "hex_example/data_interface/ros1_interface.h"
#elif HEX_ROS_VERSION == 2
#include "hex_example/data_interface/ros2_interface.h"
#else
#warning "HEX_ROS_VERSION is unknown"
#endif

#endif  // HEX_EXAMPLE_DATA_INTERFACE_DATA_INTERFACE_H_

