#ifndef _OBJECT_H_
#define _OBJECT_H_

#include <Eigen/Dense>
#include <memory>

#include "base/math.h"

namespace drone {

// Represents a foreign object
class Object {
 public:
  Object();
  ~Object();

  // Returns the delta from (origin, altitude) -> our position
  Eigen::Vector3d GetLinearDelta(const Eigen::Vector2d& origin,
                                 double altitude) const;

  // Sets the location in radians (long, lat, alt - meters)
  void set_location(Eigen::Vector2d location, double altitude) {
    location_ = location;
    altitude_ = altitude;
  }

  const Eigen::Vector2d& get_location() const { return location_; }
  double get_altitude() const { return altitude_; }

 private:
  // Location in GPS: long, lat
  Eigen::Vector2d location_;
  double altitude_;
};

}  // namespace drone

#endif  // _OBJECT_H_