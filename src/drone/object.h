#ifndef _OBJECT_H_
#define _OBJECT_H_

#include <Eigen/Dense>
#include <memory>

namespace drone {

// Represents a foreign object
class Object {
 public:
  Object();
  ~Object();

  void set_location(Eigen::Vector3d location) { location_ = location; }
  const Eigen::Vector3d& get_location() const { return location_; }

 private:
  // Location in GPS: lat, long, alt
  // altitude is relative to ground level
  Eigen::Vector3d location_;
};

}  // namespace drone

#endif  // _OBJECT_H_