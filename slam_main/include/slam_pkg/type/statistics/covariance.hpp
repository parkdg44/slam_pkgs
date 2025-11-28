//
// Created by park on 24. 8. 22.
//

#pragma once

#include <Eigen/Dense>

namespace Slam {

template <uint I>
struct Covariance {
  Eigen::Matrix<double, I, I> value{};
};

}  // namespace Slam
