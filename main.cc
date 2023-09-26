#include <iostream>
#include <yaml-cpp/yaml.h>
#include "pcd_proj_img.h"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./pcd_proj_img <yaml_file_path>" << std::endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(argv[1]);
  std::string pcd_file = config["pcd_file"].as<std::string>();
  std::string img_file = config["img_file"].as<std::string>();

  common::cameraModel cam_model;
  
  cam_model.fx = config["camera"]["fx"].as<float>();
  cam_model.fy = config["camera"]["fy"].as<float>();
  cam_model.cx = config["camera"]["cx"].as<float>();
  cam_model.cy = config["camera"]["cy"].as<float>();

  std::vector<double> extrin_t{3, 0.0};
  std::vector<double> extrin_R{9, 0.0};
  extrin_t = config["t_cl"].as<std::vector<double>>();
  extrin_R = config["rot_cl"].as<std::vector<double>>();

  
  double filter_size_ = config["filter_size"].as<double>();

  int max_proj_scale_x = config["max_proj_scale_x"].as<int>();
  int max_proj_scale_y = config["max_proj_scale_y"].as<int>();
  int min_proj_scale_x = config["min_proj_scale_x"].as<int>();
  int min_proj_scale_y = config["min_proj_scale_y"].as<int>();
  float min_proj_dist = config["min_proj_dist"].as<float>();
  float max_proj_dist = config["min_proj_dist"].as<float>();

  std::string folder_path = pcd_file.substr(0, pcd_file.find_last_of('/'));
  std::string proj_img_path = folder_path + "/proj.png";

  std::shared_ptr<pcd_proj_img::PcdProjImg> pcd_proj_img_processing = std::make_shared<pcd_proj_img::PcdProjImg>(pcd_file,img_file,cam_model);
  pcd_proj_img_processing->setTransMatrix(extrin_R, extrin_t);
  pcd_proj_img_processing->setFilterSize(filter_size_);
  pcd_proj_img_processing->setProjScale(max_proj_scale_x, max_proj_scale_y, min_proj_scale_x, min_proj_scale_y);
  pcd_proj_img_processing->setMinProjDist(min_proj_dist);
  pcd_proj_img_processing->setMaxProjDist(max_proj_dist);

  if (!pcd_proj_img_processing->start()) {
    std::cerr << "Project point cloud to image fail!" << std::endl;
  }

  cv::Mat proj_img = pcd_proj_img_processing->projImg();

  cv::imwrite(proj_img_path, proj_img);
  cv::imshow("Projected Image", proj_img);
  cv::waitKey(0);

  return 0;


}