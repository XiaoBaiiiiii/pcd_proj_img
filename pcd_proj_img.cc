#include <iostream>
#include <yaml-cpp/yaml.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "common_lib.h"

// P_c = rot_cl_ * P_L +  t_cl_;
Eigen::Matrix3f rot_cl_;
Eigen::Vector3f t_cl_; 
pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_ptr_;
pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_pcd_ptr_;
double filter_size_;
int img_width_;
int img_height_;
float fx_;
float fy_;
float cx_;
float cy_;
float k1_;
float k2_;
float p1_;
float p2_;
int max_proj_scale_x_ = 5;
int max_proj_scale_y_ = 5;
int min_proj_scale_x_ = 1;
int min_proj_scale_y_ = 1;
float min_proj_dist_ = 0.2;
float max_proj_dist_ = 40;
std::map<Eigen::Matrix<int,2,1>,float,fea_compare> dist_map_;
std::mutex proj_mutex_;

struct fea_compare{
  bool operator()(Eigen::Matrix<int,2,1> const& a, Eigen::Matrix<int,2,1> const& b) const {
    if (a(0)!=b(0)){
      return a(0) < b(0);
    } else {
      return a(1) < b(1);
    }
  }
};

const bool readPCD(const std::string& file){
  pcd_ptr_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *pcd_ptr_) == -1) {
    printf("Couldn't read file: %s \n", file );
    return false;
  }

  std::cout << "Read "
		<< pcd_ptr_->width * pcd_ptr_->height
		<< " points from "
    << file << std::endl;

  return true;
}

// Point cloud down sample
void downsamplePCD(){
  pcl::VoxelGrid<pcl::PointXYZ> voxel_scan;
  downsample_pcd_ptr_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  voxel_scan.setLeafSize(filter_size_,filter_size_,filter_size_);
  voxel_scan.setInputCloud(pcd_ptr_);
  voxel_scan.filter(*downsample_pcd_ptr_);
  return;
}

Eigen::Vector2f distortPixel(Eigen::Vector2f& ori_uv){
  
    // Normalization
    float x_corrected = (ori_uv(0) - cx_) / fx_;
    float y_corrected = (ori_uv(1) - cy_) / fy_;

    float r2 = x_corrected * x_corrected + y_corrected * y_corrected;
    float deltaRa = 1. + k1_ * r2 + k2_ * r2 * r2;
    float deltaRb = 1;
    float deltaTx = 2. * p1_ * x_corrected * y_corrected + p2_ * (r2 + 2. * x_corrected * x_corrected);
    float deltaTy = p1_ * (r2 + 2. * y_corrected * y_corrected) + 2. * p2_ * x_corrected * y_corrected;

    float distort_u0 = x_corrected * deltaRa * deltaRb + deltaTx;
    float distort_v0 = y_corrected * deltaRa * deltaRb + deltaTy;

    distort_u0 = distort_u0 * fx + cx;
    distort_v0 = distort_v0 * fy + cy;

    Eigen::Vector2f dis_uv;
    dis_uv << distort_u0, distort_v0;

    return dis_uv;

}

cv::Mat projPCD(auto* cloud_ptr){
  Eigen::Matrix3f rot_cw = extrin_R_;
  Eigen::Vector3f t_cw = extrin_t_;
  # pragma omp parallel for
  for (pcl::PointXYZ& pt: cloud_ptr->points) {
    Eigen::Vector3f pt_w = pt.getVector3fMap();
    Eigen::Vector3f pt_c = rot_cw * pt_w + t_cw;
    if (pt_c(2) < 0) continue;
    float u_ori = fx * (pt_c(0) / pt_c(2)) + cx;
    float v_ori = fy * (pt_c(1) / pt_c(2)) + cy;
    Vector2f uv_ori;
    uv_ori << u_ori, v_ori;
    Vector2f uv_dis = uv_ori;
    int u0 = int(round(uv_dis(0)));
    int v0 = int(round(uv_dis(1)));
    float dist = pt_c(2);
    int scale_x;
    int scale_y;

    static float a_x = (max_proj_scale_x_ - min_proj_scale_x_)/
                        (min_proj_dist_ - max_proj_dist_);
    static float b_x = max_proj_scale_x_ - a_x * min_proj_dist_;

    static float a_y = (max_proj_scale_y_ - min_proj_scale_y_)/
                        (min_proj_dist_ - max_proj_dist_);
    static float b_y = min_proj_scale_y_ - a_y * max_proj_dist_;

    if (dist < min_proj_dist_) {
      continue;
    } else if (min_proj_dist_ <= dist && dist<= max_proj_dist_) {
      scale_x = static_cast<int>(round(a_x * dist + b_x));
      scale_y = static_cast<int>(round(a_y * dist + b_y));
    } else {
      scale_x = 1; 
      scale_y = 1;
    } 

    for (int u = u0 - scale_x; u <= u0 + scale_x; u++){
      for (int v = v0 - scale_y; v <= v0 + scale_y; v++){
        if(u < 0 || u >= img_width_ || v < 0 || v >= img_height_){
          continue;
        }
        Eigen::Matrix<int,2,1> uv;
        uv << u, v;
        float dist = pt_c.norm();

        auto iter = dist_map_.find(uv);
        if (iter != dist_map_.end()){
          iter->second = std::min(iter->second,dist);
        } else {
          proj_mutex_.lock();
          dist_map_.insert({uv,dist});
          proj_mutex_.unlock();
        }

      }
    }

  }
}

void saveProjImg(cv::Mat& ori_img, 
                    std::map<Eigen::Matrix<int,2,1>,float,fea_compare>& dist_map, 
                    std::string& file_path) {

  cv::Mat depth_image(img_height_,img_width_,CV_8UC3,cv::Scalar(255,255,255));
  float color_scale = 255/max_proj_dist_;

  for (auto iter : dist_map){
    int u = iter.first(0);
    int v = iter.first(1);
    int pixel_color = static_cast<int>(iter.second * color_scale);
    if (pixel_color > 255) {
      pixel_color = 255;
    }

    depth_image.at<cv::Vec3b>(v,u)[0] = depth_image.at<cv::Vec3b>(v,u)[1] 
                                      = depth_image.at<cv::Vec3b>(v,u)[2] 
                                      = cv::saturate_cast<uint8_t>(pixel_color);

  }

  cv::Mat result_image(img_height_,img_width_,CV_8UC3);
  cv::addWeighted(depth_image,0.8,ori_img,0.2,0,result_image);
  cv::imwrite(file_path,result_image);

}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./pcd_proj_img <yaml_file_path>" << std::endl;
    return 1;
  }

  // 读取YAML配置文件
  YAML::Node config = YAML::LoadFile(argv[1]);
  std::string pcd_file = config["pcd_file"].as<std::string>();
  std::string img_file = config["img_file"].as<std::string>();
  std::vector<double> extrin_t{3, 0.0};  // lidar-imu translation
  std::vector<double> extrin_R{9, 0.0};  // lidar-imu rotation
  extrin_t = config["t_cl"].as<std::vector<double>>();
  extrin_R = config["rot_cl"].as<std::vector<double>>();
  t_cl_ = common::VecFromArray<float>(extrinT_);
  rot_cl_ = common::MatFromArray<float>(extrinR_);

  fx_ = config["camera"]["fx"].as<float>();
  fy_ = config["camera"]["fy"].as<float>();
  cx_ = config["camera"]["cx"].as<float>();
  cy_ = config["camera"]["cy"].as<float>();

  filter_size_ = config["filter_size"].as<double>();

  max_proj_scale_x_ = config["max_proj_scale_x"].as<int>();
  max_proj_scale_y_ = config["max_proj_scale_y"].as<int>();
  min_proj_scale_x_ = config["min_proj_scale_x"].as<int>();
  min_proj_scale_y_ = config["min_proj_scale_y"].as<int>();
  min_proj_dist_ = config["min_proj_dist"].as<float>();
  max_proj_dist_ = config["min_proj_dist"].as<float>();

  std::string folder_path = pcd_file.substr(0, pcd_file.find_last_of('/'));
  std::string proj_img_path = folder_path + "/proj.png";

  cv::Mat original_image = cv::imread(img_file);

  // 读取点云




}