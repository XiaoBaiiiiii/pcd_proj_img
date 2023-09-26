#ifndef PCD_PROJ_IMG_H
#define PCD_PROJ_IMG_H

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <mutex>
#include <memory>
#include "omp.h"
#include "common_lib.h"
#include "camera_model.h"

namespace pcd_proj_img {

class PcdProjImg {
public:
PcdProjImg(std::string& pcd_file, std::string& img_file, common::cameraModel& camera_model):
  pcd_file_(pcd_file),img_file_(img_file){
    camera_model_ = camera_model;
    try {
      camera_model.checkForEmpty();
    } catch (const std::runtime_error& e) {
      std::cerr << "Error: " << e.what() << std::endl;
    }

}

~PcdProjImg(){}

// set filter size for downsample
void setFilterSize(double& filter_size){
  if_downsample_ = true;
  filter_size_ = filter_size;
}

void setNotDownsample(){if_downsample_ = false;}

template <typename T>
void setTransMatrix(std::vector<T> r, std::vector<T> t){
  t_cl_ = common::VecFromArray<float>(t);
  rot_cl_ = common::MatFromArray<float>(r);
}

void setProjScale(int& max_proj_scale_x,
              int& max_proj_scale_y,
              int& min_proj_scale_x,
              int& min_proj_scale_y) {
  max_proj_scale_x_ = max_proj_scale_x;
  max_proj_scale_y_ = max_proj_scale_y;
  min_proj_scale_x_ = min_proj_scale_x;
  min_proj_scale_y_ = min_proj_scale_y;

}

template <typename T>
void setMinProjDist(T& min_proj_dist) {

  min_proj_dist_ = static_cast<float>(min_proj_dist);

}

template <typename T>
void setMaxProjDist(T& max_proj_dist) {

  max_proj_dist_ = static_cast<float>(max_proj_dist);

}




// get projected image
cv::Mat projImg();
// get depth image
cv::Mat depthImg();
// start to process
bool start();
// check the validility for parameters
void check();

private:
  struct fea_compare {
    bool operator()(Eigen::Matrix<int,2,1> const& a, Eigen::Matrix<int,2,1> const& b) const {
      if (a(0)!=b(0)){
        return a(0) < b(0);
      } else {
        return a(1) < b(1);
      }
    }
  };

  // read pcd file
  const bool readPCD();
  // read img file
  const bool readImg();
  // Point cloud down sample
  void downsamplePCD();

  void projPCD(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
  cv::Mat getProjImg(cv::Mat& ori_img, 
                      std::map<Eigen::Matrix<int,2,1>,float,fea_compare>& dist_map, 
                      const float& max_proj_dist);

  std::string pcd_file_;
  std::string img_file_;
  common::cameraModel camera_model_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_ptr_;
  cv::Mat ori_img_;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_pcd_ptr_;
  bool if_downsample_ = true;
  double filter_size_ = 0.2;
  // P_c = rot_cl_ * P_L +  t_cl_;
  Eigen::Matrix3f rot_cl_;
  Eigen::Vector3f t_cl_;
  int img_width_;
  int img_height_;

  int max_proj_scale_x_ = 5;
  int max_proj_scale_y_ = 5;
  int min_proj_scale_x_ = 1;
  int min_proj_scale_y_ = 1;
  float min_proj_dist_ = 0.2;
  float max_proj_dist_ = 40;
  std::map<Eigen::Matrix<int,2,1>,float,fea_compare> dist_map_;
  std::mutex proj_mutex_;

  cv::Mat depth_img_;
  cv::Mat proj_img_;
};

} // namespace pcd_proj_img

#endif
