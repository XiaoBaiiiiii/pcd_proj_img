#ifndef CAMERA_MODEL_H
#define CAMERA_MODEL_H

#include <stdexcept>

namespace common {

struct cameraModel {
int width = 0;
int height = 0;
double fx = 0;
double fy = 0;
double cx = 0;
double cy = 0;

cameraModel& operator=(const cameraModel& other) {
  if (this == &other) {
      return *this;
  }
  
  width = other.width;
  height = other.height;
  fx = other.fx;
  fy = other.fy;
  cx = other.cx;
  cy = other.cy;

  return *this;
}

void checkForEmpty() const {
  if (fx == 0.0 || fy == 0.0 || cx == 0.0 || cy == 0.0) {
      throw std::runtime_error("Camera model contains empty or zero values.");
  }
}


};

// Eigen::Vector2f distortPixel(Eigen::Vector2f& ori_uv){
  
//     // Normalization
//     float x_corrected = (ori_uv(0) - cx_) / fx_;
//     float y_corrected = (ori_uv(1) - cy_) / fy_;

//     float r2 = x_corrected * x_corrected + y_corrected * y_corrected;
//     float deltaRa = 1. + k1_ * r2 + k2_ * r2 * r2;
//     float deltaRb = 1;
//     float deltaTx = 2. * p1_ * x_corrected * y_corrected + p2_ * (r2 + 2. * x_corrected * x_corrected);
//     float deltaTy = p1_ * (r2 + 2. * y_corrected * y_corrected) + 2. * p2_ * x_corrected * y_corrected;

//     float distort_u0 = x_corrected * deltaRa * deltaRb + deltaTx;
//     float distort_v0 = y_corrected * deltaRa * deltaRb + deltaTy;

//     distort_u0 = distort_u0 * fx + cx;
//     distort_v0 = distort_v0 * fy + cy;

//     Eigen::Vector2f dis_uv;
//     dis_uv << distort_u0, distort_v0;

//     return dis_uv;

// }

} // namespace common

#endif