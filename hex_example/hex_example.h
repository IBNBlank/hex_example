/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui blank@invalid.example
 * Date   : 2023-10-31
 ****************************************************************/

#ifndef HEX_EXAMPLE_HEX_EXAMPLE_H_
#define HEX_EXAMPLE_HEX_EXAMPLE_H_

#include <string>

namespace hex {
namespace example {

class HexExample {
 public:
  static HexExample& GetHexExample() {
    static HexExample template_func;
    return template_func;
  }

  // Work Handle
  bool Init();
  bool Work();

 private:
  HexExample() = default;
  virtual ~HexExample() = default;

  // Work Handle
  void SubMessage();
  void PubMessage();

  // Parameters Handle
  std::string kout_string_;
  int32_t kmax_count_;

  // Variable Handle
  std::string in_string_;
  int32_t count_;
};

}  // namespace example
}  // namespace hex

#endif  // HEX_EXAMPLE_HEX_EXAMPLE_H_
