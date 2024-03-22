#ifndef UTILITIES_H
#define UTILITIES_H

#include <math.h>
#include <eigen3/Eigen/Eigen>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <numeric>
#include <tuple>

/* struct Color_t //{ */

struct Color_t
{
  double red;
  double green;
  double blue;
  double alpha;

  bool operator==(const Color_t &other) const {
    return this->red == other.red && this->green == other.green && this->blue == other.blue && this->alpha == other.alpha;
  }

  bool operator!=(const Color_t &other) const {
    return this->red != other.red || this->green != other.green || this->blue != other.blue || this->alpha != other.alpha;
  }
};

//}

/* printVector3d() //{ */

/**
 * @brief Print the given 3D vector to the console.
 *
 * @param vec: the vector
 * @param prefix: text prefix added before the vector
 */
void printVector3d(const Eigen::Vector3d &vec, const std::string &prefix) {

  if (!prefix.empty()) {
    std::cout << prefix << " ";
  }

  std::cout << vec.x() << ", " << vec.y() << ", " << vec.z() << std::endl;
}

//}

/* computeStatistics() //{ */

/**
 * @brief Compute mean and standard deviation of the given vector of doubles.
 *
 * @param vector of double values
 *
 * @example
 *   std::vector<double> vec = {1.0, 2.0, 3.0};
 *   const auto &[mean, stdev] = computeStatistics(vec);
 *
 * @return tuple<double, double> mean and standard deviation of the double values
 */
std::tuple<double, double> computeStatistics(const std::vector<double> &vec) {

  assert(!vec.empty());

  if (vec.size() == 1) {
    return std::make_tuple(vec.at(0), 0.0);
  }

  const double sum  = std::accumulate(vec.begin(), vec.end(), 0.0);
  const double mean = sum / vec.size();

  double accum = 0.0;
  std::for_each(std::begin(vec), std::end(vec), [&](const double d) { accum += (d - mean) * (d - mean); });
  const double stdev = std::sqrt(accum / double((vec.size() - 1)));

  return std::make_tuple(mean, stdev);
}

//}

/* saturateVector() //{ */

/**
 * @brief Saturate the given vector to max length.
 *
 * @param vec to be saturated, if needed
 * @param maximal length to which the vector should be saturated to
 *
 * @return the vector with maximal magnitude of max_len (direction unchanged)
 */
Eigen::Vector3d saturateVector(const Eigen::Vector3d &vec, const double max_len) {

  const double norm = vec.norm();

  if (norm > 0.0 && norm > max_len) {
    return max_len * vec / norm;
  }

  return vec;
}

//}

/* clampVector() //{ */

/**
 * @brief Clamp magnitude of the given vector to the given interval.
 *
 * @return true if the magnitude of the given vector is zero
 */
bool clampVector(Eigen::Vector3d &vec, const double min_len, const double max_len) {

  const double norm = vec.norm();

  if (norm < 1e-7) {
    return true;
  }

  if (norm > max_len) {
    vec = max_len * vec / norm;
  } else if (norm < min_len) {
    vec = min_len * vec / norm;
  }

  return false;
}

//}

/* vectorIsFinite() //{ */

/**
 *
 * @return true if all elements of the vector are valid
 */
bool vectorIsFinite(const Eigen::Vector3d &vec) {

  return std::isfinite(vec.x()) && std::isfinite(vec.y()) && std::isfinite(vec.z());
}

//}

/* vectorToStr() //{ */

std::string vectorToStr(const Eigen::Vector3d &vec) {

  std::ostringstream oss;

  oss << "(";
  oss << std::fixed << std::setprecision(2) << vec.x() << ", ";
  oss << std::fixed << std::setprecision(2) << vec.y() << ", ";
  oss << std::fixed << std::setprecision(2) << vec.z();
  oss << ")";

  return oss.str();
}

//}

/* vectorToStr() //{ */

/**
 * @brief Convert a given vector to polar coordinates.
 *
 * @param vec to be converted
 *
 * @return radius, angle
 */
std::tuple<double, double> vectorToPolarCoordinates(const Eigen::Vector3d &vec) {

  return {vec.norm(), std::atan2(vec.y(), vec.x())};
}

//}

/* vectorFromPolarCoordinates() //{ */

/**
 * @brief Convert polar coordinates to a vector.
 */
Eigen::Vector3d vectorFromPolarCoordinates(const double r, const double az) {

  return r * Eigen::Vector3d(std::cos(az), std::sin(az), 0.0);
}

//}

/* signedAngleBetweenTwoVectors() //{ */

/**
 * @brief Returns the shortest angle (signed) between two vectors.
 */
double signedAngleBetweenTwoVectors(const Eigen::Vector3d &v_from, const Eigen::Vector3d &v_to) {

  const auto &[r_from, th_from] = vectorToPolarCoordinates(v_from);
  const auto &[r_to, th_to]     = vectorToPolarCoordinates(v_to);

  double th = th_to - th_from;

  if (std::fabs(th + 2.0 * M_PI) < std::fabs(th)) {
    th = th + 2.0 * M_PI;
  }

  if (std::fabs(th - 2.0 * M_PI) < std::fabs(th)) {
    th = th - 2.0 * M_PI;
  }

  return th;
}

//}

#endif  // UTILITIES_H
