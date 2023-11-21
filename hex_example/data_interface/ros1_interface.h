/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-11-20
 ****************************************************************/

#ifndef HEX_EXAMPLE_DATA_INTERFACE_ROS1_INTERFACE_H_
#define HEX_EXAMPLE_DATA_INTERFACE_ROS1_INTERFACE_H_

#include <memory>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

namespace hex {
namespace example {

enum class LogLevel { kDebug = 0, kInfo, kWarn, kError, kFatal };

class DataInterface {
 public:
  static DataInterface& GetDataInterface() {
    static DataInterface singleton;
    return singleton;
  }

  // Interface Handle
  void Log(LogLevel, const char*, ...);
  inline void Work() { ros::spin(); }
  inline void Shutdown() { ros::shutdown(); }
  inline bool Ok() { return ros::ok(); }
  inline double GetTime() { return ros::Time::now().toSec(); }

  // Initialization Handle
  void Init(int, char*[], std::string, double, void (*)());
  void Deinit();

  // Parameter Handle
  inline const std::string& GetOutString() { return kout_string_; }
  inline int32_t const GetMaxCount() { return kmax_count_; }

  // Publisher Handle
  void PublishOutString(const std::string&);

  // Subscriber Handle
  inline bool GetInStringFlag() { return in_string_flag_; }
  inline void ResetInStringFlag() { in_string_flag_ = false; }
  inline const std::string& GetInString() { return in_string_; }

 protected:
  // Timer Handle
  inline void TimerHandle(const ros::TimerEvent&) { timer_handle_(); }

  // Subscriber Handle
  void InStringHandle(const std_msgs::StringPtr&);

 private:
  DataInterface() = default;
  virtual ~DataInterface() = default;

  // Initialization Handle
  void ParameterInit();
  void VariableInit();
  void PublisherInit();
  void SubscriberInit();
  void TimerInit(double, void (*)());

  // Node Handle
  ros::NodeHandle* nh_ptr_;
  ros::NodeHandle* nh_local_ptr_;

  // Timer Handle
  ros::Timer timer_;
  void (*timer_handle_)();

  // Publisher Handle
  ros::Publisher out_string_pub_;

  // Subscriber Handle
  ros::Subscriber in_string_sub_;

  // Parameters Handle
  std::string kout_string_;
  int32_t kmax_count_;

  // Variable Handle
  bool in_string_flag_;
  std::string in_string_;
};

}  // namespace example
}  // namespace hex

#endif  // HEX_EXAMPLE_DATA_INTERFACE_ROS1_INTERFACE_H_
