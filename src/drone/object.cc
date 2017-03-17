#include "drone/object.h"

#include "base/math.h"

namespace drone {

Object::Object() = default;
Object::~Object() = default;

Eigen::Vector3d Object::GetLinearDelta(const Eigen::Vector2d& origin,
                                       double altitude) const {
  auto del = CalculateGPSLinearDistance(origin, location_);

  return {del.x(), del.y(), altitude - altitude_};
}

}  // namespace drone