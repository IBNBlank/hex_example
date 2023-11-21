/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-11-20
 ****************************************************************/

#include "hex_example/data_interface/ros1_interface.h"

namespace hex {
namespace example {

void DataInterface::Log(LogLevel level, const char* format, ...) {
  char* buffer;

  va_list args;
  va_start(args, format);
  int32_t len = vasprintf(&buffer, format, args);
  va_end(args);

  if (len < 0) {
    ROS_FATAL("### Wrong Log Message ###");
    return;
  }

  switch (level) {
    case LogLevel::kDebug: {
      ROS_DEBUG("%s", buffer);
      break;
    }
    case LogLevel::kInfo: {
      ROS_INFO("%s", buffer);
      break;
    }
    case LogLevel::kWarn: {
      ROS_WARN("%s", buffer);
      break;
    }
    case LogLevel::kError: {
      ROS_ERROR("%s", buffer);
      break;
    }
    case LogLevel::kFatal: {
      ROS_FATAL("%s", buffer);
      break;
    }
    default: {
      ROS_FATAL("### Wrong Log Level ###");
      ROS_FATAL("%s", buffer);
      break;
    }
  }

  free(buffer);
}

void DataInterface::Init(int argc, char* argv[], std::string name,
                         double period, void (*handle)()) {
  ros::init(argc, argv, name);
  static ros::NodeHandle nh;
  static ros::NodeHandle nh_local("~");
  nh_ptr_ = &nh;
  nh_local_ptr_ = &nh_local;

  ParameterInit();
  VariableInit();
  PublisherInit();
  SubscriberInit();
  TimerInit(period, handle);

  Log(LogLevel::kInfo,
      "\033[1;32m %s: ### data interface init finish ### \033[0m", name.data());
}

void DataInterface::Deinit() { Shutdown(); }

void DataInterface::ParameterInit() {
  nh_local_ptr_->param<std::string>("out_string", kout_string_, "hello");
  nh_local_ptr_->param<int32_t>("max_count", kmax_count_, 10);
}

void DataInterface::VariableInit() {
  in_string_flag_ = false;
  in_string_ = "";
}

void DataInterface::PublisherInit() {
  out_string_pub_ = nh_ptr_->advertise<std_msgs::String>("out_string", 1);
}

void DataInterface::SubscriberInit() {
  in_string_sub_ =
      nh_ptr_->subscribe("in_string", 1, &DataInterface::InStringHandle, this);
}

void DataInterface::TimerInit(double period, void (*handle)()) {
  timer_handle_ = handle;
  timer_ = nh_ptr_->createTimer(ros::Duration(period * 0.001),
                                &DataInterface::TimerHandle, this);
}

void DataInterface::PublishOutString(const std::string& out_string) {
  std_msgs::StringPtr out_string_ptr(new std_msgs::String);

  out_string_ptr->data = out_string;

  out_string_pub_.publish(out_string_ptr);
}

void DataInterface::InStringHandle(const std_msgs::StringPtr& data) {
  if (!in_string_flag_) {
    in_string_ = data->data;
    in_string_flag_ = true;
  }
}

}  // namespace example
}  // namespace hex
