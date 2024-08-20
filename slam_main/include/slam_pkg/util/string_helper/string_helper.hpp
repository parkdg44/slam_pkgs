//
// Created by park on 24. 5. 20.
//

#pragma once

#include <vector>

namespace Slam::util {

inline std::vector<std::string> split_str(std::string str, char delimiter) {
  std::istringstream iss(str);
  std::string buffer;
  std::vector<std::string> result;

  while (getline(iss, buffer, delimiter)) {
    result.push_back(buffer);
  }

  return result;
}

}  // namespace Slam::util