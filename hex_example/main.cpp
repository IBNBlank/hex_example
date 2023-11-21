/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-11-21
 ****************************************************************/

#include "hex_example/data_interface/data_interface.h"
#include "hex_example/hex_example.h"

using hex::example::DataInterface;
using hex::example::HexExample;
using hex::example::LogLevel;

const char kNodeName[] = "hex_example";

void TimeHandle() {
  enum class FiniteState { kInitState = 0, kWorkState };
  static FiniteState finite_state_machine_state = FiniteState::kInitState;
  static DataInterface& data_interface = DataInterface::GetDataInterface();
  static HexExample& hex_example = HexExample::GetHexExample();

  switch (finite_state_machine_state) {
    case FiniteState::kInitState: {
      if (hex_example.Init()) {
        data_interface.Log(LogLevel::kInfo, "%s : Init Succeded", kNodeName);
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(LogLevel::kWarn, "%s : Init Failed", kNodeName);
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    case FiniteState::kWorkState: {
      if (hex_example.Work()) {
        // data_interface.Log(LogLevel::kInfo, "%s : Work Succeded", kNodeName);
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(LogLevel::kWarn, "%s : Work Failed", kNodeName);
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    default: {
      data_interface.Log(LogLevel::kError, "%s : Unknown State", kNodeName);
      finite_state_machine_state = FiniteState::kInitState;
      break;
    }
  }
}

int main(int argc, char** argv) {
  DataInterface& data_interface = DataInterface::GetDataInterface();
  data_interface.Init(argc, argv, kNodeName, 20.0, TimeHandle);

  data_interface.Work();

  data_interface.Deinit();
  return 0;
}
