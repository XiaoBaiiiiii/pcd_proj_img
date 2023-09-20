#include <iostream>
#include <yaml-cpp/yaml.h>

#include "common_lib.h"

// P_i = extrin_R_ * P_L +  extrin_t_;
Eigen::Matrix3d extrin_R_;
Eigen::Vector3d extrin_t_; 

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
  extrin_t = config["extrinsic_t"].as<std::vector<double>>();
  extrin_R = config["extrinsic_R"].as<std::vector<double>>();

  extrin_t_ = common::VecFromArray<double>(extrinT_);
  extrin_R_ = common::MatFromArray<double>(extrinR_);

  


}