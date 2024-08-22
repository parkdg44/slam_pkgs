//
// Created by park on 24. 8. 22.
//

#pragma once

namespace Slam {

template <uint I>
struct Covariance {
  Eigen::Matrix<double, I, I> value{};
};

}  // namespace Slam
