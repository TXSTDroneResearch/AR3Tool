#ifndef _BASE_MATH_H_
#define _BASE_MATH_H_

#include <Eigen/Dense>
#include <cmath>

constexpr double PI = 3.141592653589793238463;

// Inputs are in radians (longitude, latitude), not degrees!
// Units of the output are in meters.
inline Eigen::Vector2d CalculateGPSLinearDistance(const Eigen::Vector2d& p1,
                                                  const Eigen::Vector2d& p2) {
  // Haversine formula
  Eigen::Vector2d del = p2 - p1;
  double dla2 = sin(del.y() / 2) * sin(del.y() / 2);
  double dlo2 = sin(del.x() / 2) * sin(del.x() / 2);
  double a = dla2 + cos(p1.y()) * cos(p2.y()) * dlo2;
  double c = 2 * asin(sqrt(a));
  double d = c * 6378100;  // 6378100 = radius of earth in meters

  // Calculate the bearing
  double bearing = atan2(
      sin(del.x()) * cos(p2.y()),
      cos(p1.y()) * sin(p2.y()) - sin(p1.y()) * cos(p2.y()) * cos(del.x()));

  bearing = -bearing + PI / 2;

  // Calculate a linear delta
  Eigen::Vector2d linDel;
  linDel[0] = d * cos(bearing);
  linDel[1] = d * sin(bearing);
  return linDel;
}

#endif  // _BASE_MATH_H_