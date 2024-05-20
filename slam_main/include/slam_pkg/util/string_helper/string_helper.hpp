//
// Created by park on 24. 5. 20.
//

#pragma once

#include <vector>

namespace Slam::util {

inline std::vector<std::string> split_str(std::string str, char Delimiter) {
  std::istringstream iss(str);
  std::string buffer;
  std::vector<std::string> result;

  while (getline(iss, buffer, Delimiter)) {
    result.push_back(buffer);
  }

  return result;
}

}  // namespace Slam::util