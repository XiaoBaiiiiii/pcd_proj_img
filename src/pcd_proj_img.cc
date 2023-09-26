#include "pcd_proj_img.h"

namespace pcd_proj_img {

const bool PcdProjImg::readPCD(){
  pcl::PointCloud<pcl::PointXYZ>::Ptr new_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *new_ptr) == -1) {
    printf("Couldn't read pcd file \n");
    return false;
  }

  // std::cout << "Read "
	// 	<< new_ptr->width * new_ptr->height
	// 	<< " points from "
  //   << pcd_file_ << std::endl;

  pcd_ptr_ = new_ptr;
  return true;

}

const bool PcdProjImg::readImg(){
  ori_img_ = cv::imread(img_file_);
  if (ori_img_.empty()) {
    std::cerr << "Error: Failed to read the image!" << std::endl;
    return false;
  }
  
  if (camera_model_.width == 0 || camera_model_.height == 0) {
    img_width_ = camera_model_.width = ori_img_.cols;
    img_height_ = camera_model_.height = ori_img_.rows;
  } else {
    img_width_ = ori_img_.cols;
    img_height_ = ori_img_.rows;
    if (img_width_ != camera_model_.width || img_height_!= camera_model_.height) {
      std::cerr << "Error: Camera model parameters wrong!" << std::endl;
      return false;
    }
  }

  return true;
}

void PcdProjImg::check(){
  if (pcd_file_.empty() || img_file_.empty()) {
    throw std::runtime_error("Invalid class state: pcd_file_ and img_file_ must not be empty.");
  }

  if (!pcd_ptr_) {
    throw std::runtime_error("Invalid class state: pcd_ptr_ is nullptr.");
  }

  if (ori_img_.empty()) {
    throw std::runtime_error("Invalid class state: ori_img_ is empty.");
  }

  if (!(rot_cl_.transpose() * rot_cl_).isApprox(Eigen::Matrix3f::Identity(),1e-6)) {
    throw std::runtime_error("Invalid class state: rot_cl_ is not orthogonal matrix.");
  }

  if (rot_cl_.array().isNaN().any() || t_cl_.array().isNaN().any()) {
     throw std::runtime_error("Invalid class state: rot_cl_ or  t_cl_ has nan.");
  }

  if (img_width_ <= 0 || img_height_ <= 0) {
    throw std::runtime_error("Invalid class state: img_width_ and img_height_ must be positive.");
  }

  if (max_proj_scale_x_ <= 0 || max_proj_scale_y_ <= 0 || min_proj_scale_x_ <= 0 || min_proj_scale_y_ <= 0) {
    throw std::runtime_error("Invalid class state: Projection scale values must be positive.");
  }

  if (min_proj_dist_ <= 0.0 || max_proj_dist_ <= 0.0 || min_proj_dist_ > max_proj_dist_) {
    throw std::runtime_error("Invalid class state: Projection distance values are invalid.");
  }

}

void PcdProjImg::downsamplePCD(){
  pcl::PointCloud<pcl::PointXYZ>::Ptr new_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxel_scan;
  voxel_scan.setLeafSize(filter_size_,filter_size_,filter_size_);
  voxel_scan.setInputCloud(pcd_ptr_);
  voxel_scan.filter(*new_ptr);
  downsample_pcd_ptr_ = new_ptr;
  return;
}

void PcdProjImg::projPCD(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr){
  Eigen::Matrix3f rot_cl = rot_cl_;
  Eigen::Vector3f t_cl = t_cl_;
  # pragma omp parallel for
  for (pcl::PointXYZ& pt: cloud_ptr->points) {
    Eigen::Vector3f pt_l = pt.getVector3fMap();
    Eigen::Vector3f pt_c = rot_cl * pt_l + t_cl;
    if (pt_c(2) < 0) continue;
    float u_ori = camera_model_.fx * (pt_c(0) / pt_c(2)) + camera_model_.cx;
    float v_ori = camera_model_.fy * (pt_c(1) / pt_c(2)) + camera_model_.cy;
    Eigen::Vector2f uv_ori;
    uv_ori << u_ori, v_ori;
    Eigen::Vector2f uv_dis = uv_ori;
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
          proj_mutex_.lock();
          iter->second = std::min(iter->second,dist);
          proj_mutex_.unlock();
        } else {
          proj_mutex_.lock();
          dist_map_.insert({uv,dist});
          proj_mutex_.unlock();
        }

      }
    }

  }

  return;
}

cv::Mat PcdProjImg::getProjImg(cv::Mat& ori_img, 
                    std::map<Eigen::Matrix<int,2,1>,float,fea_compare>& dist_map, 
                    const float& max_proj_dist) {

  int img_width = ori_img.cols;
  int img_height = ori_img.rows;
  cv::Mat depth_image(img_height,img_width,CV_8UC3,cv::Scalar(255,255,255));
  float color_scale = 255/max_proj_dist;

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
  
  depth_img_ = depth_image;
  

  return result_image;

}

cv::Mat PcdProjImg::projImg(){
  return proj_img_;
}

cv::Mat PcdProjImg::depthImg(){
  return depth_img_;
}

bool PcdProjImg::start(){
  if (!readPCD()){return false;}
  if (!readImg()){return false;}
  try {check();} catch (const std::runtime_error& e) {

    std::cerr << "Error: " << e.what() << std::endl;
    return false;
  }

  if (if_downsample_){
    downsamplePCD();
    projPCD(downsample_pcd_ptr_);
  } else {
    projPCD(pcd_ptr_);
  }

  proj_img_ = getProjImg(ori_img_,dist_map_,max_proj_dist_);

  return true;
}


} //namespace pcd_proj_img