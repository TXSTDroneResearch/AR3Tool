#ifndef _OBJECT_H_
#define _OBJECT_H_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "base/math.h"

namespace drone {

// Represents a region defined by GPS points and altitudes.
class Region {
 public:
  Region();
  ~Region();

  // Returns the delta from (origin, altitude) -> our position
  Eigen::Vector3d GetLinearDelta(size_t point,
                                 const Eigen::Vector3d& origin) const;

  // Sets the location in radians (long, lat, alt - meters)
  void add_point(Eigen::Vector3d location) { points_.push_back(location); }

  // Sets the location in degrees (long, lat, alt - meters)
  void add_point_deg(Eigen::Vector3d location) {
    points_.push_back(
        {deg2rad(location.x()), deg2rad(location.y()), location.z()});
  }

  size_t get_num_points() const { return points_.size(); }

 private:
  std::string name_;
  uint8_t color[4];  // RGBA

  // Location in GPS: long, lat
  std::vector<Eigen::Vector3d> points_;
};

}  // namespace drone

#endif  // _OBJECT_H_