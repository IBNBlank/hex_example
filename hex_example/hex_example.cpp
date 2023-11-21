/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-11-21
 ****************************************************************/

#include "hex_example/hex_example.h"

#include "hex_example/data_interface/data_interface.h"

namespace hex {
namespace example {

bool HexExample::Init() {
  static DataInterface& data_interface = DataInterface::GetDataInterface();

  // Parameter
  kout_string_ = data_interface.GetOutString();
  kmax_count_ = data_interface.GetMaxCount();

  // Variable
  in_string_ = "";
  count_ = 0;

  return true;
}

bool HexExample::Work() {
  SubMessage();
  PubMessage();

  return true;
}

void HexExample::SubMessage() {
  static DataInterface& data_interface = DataInterface::GetDataInterface();

  if (data_interface.GetInStringFlag()) {
    in_string_ = data_interface.GetInString();
    data_interface.ResetInStringFlag();

    data_interface.Log(LogLevel::kInfo, "I hear: %s; count: %d",
                       in_string_.c_str(), count_);

    count_++;
    if (count_ >= kmax_count_) {
      count_ = 0;
    }
  }
}

void HexExample::PubMessage() {
  static DataInterface& data_interface = DataInterface::GetDataInterface();

  data_interface.PublishOutString(kout_string_);
}

}  // namespace example
}  // namespace hex
