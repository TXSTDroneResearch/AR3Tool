#include "drone/region.h"

#include "base/math.h"

namespace drone {

Region::Region() = default;
Region::~Region() = default;

Eigen::Vector3d Region::GetLinearDelta(size_t point,
                                       const Eigen::Vector3d& origin) const {
  if (point >= points_.size()) {
    // TODO(justin): Assertion failure here!
    return {0., 0., 0.};
  }

  auto& location = points_[point];
  auto del = CalculateGPSLinearDistance({origin.x(), origin.y()},
                                        {location.x(), location.y()});

  return {del.x(), del.y(), location.z() - origin.z()};
}

}  // namespace drone