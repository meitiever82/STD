#include "include/STDescROS.h"

void read_parameters(std::shared_ptr<rclcpp::Node> node, ConfigSetting &config_setting) {

  // Declare parameters with default values
  node->declare_parameter("ds_size", 0.5);
  node->declare_parameter("maximum_corner_num", 100);
  node->declare_parameter("plane_merge_normal_thre", 0.1);
  node->declare_parameter("plane_detection_thre", 0.01);
  node->declare_parameter("voxel_size", 2.0);
  node->declare_parameter("voxel_init_num", 10);
  node->declare_parameter("proj_image_resolution", 0.5);
  node->declare_parameter("proj_dis_min", 0.0);
  node->declare_parameter("proj_dis_max", 2.0);
  node->declare_parameter("corner_thre", 10.0);
  node->declare_parameter("descriptor_near_num", 10);
  node->declare_parameter("descriptor_min_len", 2.0);
  node->declare_parameter("descriptor_max_len", 50.0);
  node->declare_parameter("non_max_suppression_radius", 2.0);
  node->declare_parameter("std_side_resolution", 0.2);
  node->declare_parameter("skip_near_num", 50);
  node->declare_parameter("candidate_num", 50);
  node->declare_parameter("sub_frame_num", 10);
  node->declare_parameter("rough_dis_threshold", 0.01);
  node->declare_parameter("vertex_diff_threshold", 0.5);
  node->declare_parameter("icp_threshold", 0.5);
  node->declare_parameter("normal_threshold", 0.2);
  node->declare_parameter("dis_threshold", 0.5);

  // Get parameter values
  config_setting.ds_size_ = node->get_parameter("ds_size").as_double();
  config_setting.maximum_corner_num_ = node->get_parameter("maximum_corner_num").as_int();
  config_setting.plane_merge_normal_thre_ = node->get_parameter("plane_merge_normal_thre").as_double();
  config_setting.plane_detection_thre_ = node->get_parameter("plane_detection_thre").as_double();
  config_setting.voxel_size_ = node->get_parameter("voxel_size").as_double();
  config_setting.voxel_init_num_ = node->get_parameter("voxel_init_num").as_int();
  config_setting.proj_image_resolution_ = node->get_parameter("proj_image_resolution").as_double();
  config_setting.proj_dis_min_ = node->get_parameter("proj_dis_min").as_double();
  config_setting.proj_dis_max_ = node->get_parameter("proj_dis_max").as_double();
  config_setting.corner_thre_ = node->get_parameter("corner_thre").as_double();
  config_setting.descriptor_near_num_ = node->get_parameter("descriptor_near_num").as_int();
  config_setting.descriptor_min_len_ = node->get_parameter("descriptor_min_len").as_double();
  config_setting.descriptor_max_len_ = node->get_parameter("descriptor_max_len").as_double();
  config_setting.non_max_suppression_radius_ = node->get_parameter("non_max_suppression_radius").as_double();
  config_setting.std_side_resolution_ = node->get_parameter("std_side_resolution").as_double();
  config_setting.skip_near_num_ = node->get_parameter("skip_near_num").as_int();
  config_setting.candidate_num_ = node->get_parameter("candidate_num").as_int();
  config_setting.sub_frame_num_ = node->get_parameter("sub_frame_num").as_int();
  config_setting.rough_dis_threshold_ = node->get_parameter("rough_dis_threshold").as_double();
  config_setting.vertex_diff_threshold_ = node->get_parameter("vertex_diff_threshold").as_double();
  config_setting.icp_threshold_ = node->get_parameter("icp_threshold").as_double();
  config_setting.normal_threshold_ = node->get_parameter("normal_threshold").as_double();
  config_setting.dis_threshold_ = node->get_parameter("dis_threshold").as_double();

  std::cout << "Sucessfully load parameters:" << std::endl;
  std::cout << "----------------Main Parameters-------------------"
            << std::endl;
  std::cout << "voxel size:" << config_setting.voxel_size_ << std::endl;
  std::cout << "loop detection threshold: " << config_setting.icp_threshold_
            << std::endl;
  std::cout << "sub-frame number: " << config_setting.sub_frame_num_
            << std::endl;
  std::cout << "candidate number: " << config_setting.candidate_num_
            << std::endl;
  std::cout << "maximum corners size: " << config_setting.maximum_corner_num_
            << std::endl;
}

void publish_std_pairs(
    const std::vector<std::pair<STDesc, STDesc>> &match_std_pairs,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr std_publisher) {
  visualization_msgs::msg::MarkerArray ma_line;
  visualization_msgs::msg::Marker m_line;
  m_line.type = visualization_msgs::msg::Marker::LINE_LIST;
  m_line.action = visualization_msgs::msg::Marker::ADD;
  m_line.ns = "lines";
  // Don't forget to set the alpha!
  m_line.scale.x = 0.25;
  m_line.pose.orientation.w = 1.0;
  m_line.header.frame_id = "camera_init";
  m_line.id = 0;
  int max_pub_cnt = 1;
  for (auto var : match_std_pairs) {
    if (max_pub_cnt > 100) {
      break;
    }
    max_pub_cnt++;
    m_line.color.a = 0.8;
    m_line.points.clear();
    m_line.color.r = 138.0 / 255;
    m_line.color.g = 226.0 / 255;
    m_line.color.b = 52.0 / 255;
    geometry_msgs::msg::Point p;
    p.x = var.second.vertex_A_[0];
    p.y = var.second.vertex_A_[1];
    p.z = var.second.vertex_A_[2];
    Eigen::Vector3d t_p;
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    p.x = var.second.vertex_B_[0];
    p.y = var.second.vertex_B_[1];
    p.z = var.second.vertex_B_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.second.vertex_C_[0];
    p.y = var.second.vertex_C_[1];
    p.z = var.second.vertex_C_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    p.x = var.second.vertex_B_[0];
    p.y = var.second.vertex_B_[1];
    p.z = var.second.vertex_B_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.second.vertex_C_[0];
    p.y = var.second.vertex_C_[1];
    p.z = var.second.vertex_C_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    p.x = var.second.vertex_A_[0];
    p.y = var.second.vertex_A_[1];
    p.z = var.second.vertex_A_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    // another
    m_line.points.clear();
    m_line.color.r = 1;
    m_line.color.g = 1;
    m_line.color.b = 1;
    p.x = var.first.vertex_A_[0];
    p.y = var.first.vertex_A_[1];
    p.z = var.first.vertex_A_[2];
    m_line.points.push_back(p);
    p.x = var.first.vertex_B_[0];
    p.y = var.first.vertex_B_[1];
    p.z = var.first.vertex_B_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.first.vertex_C_[0];
    p.y = var.first.vertex_C_[1];
    p.z = var.first.vertex_C_[2];
    m_line.points.push_back(p);
    p.x = var.first.vertex_B_[0];
    p.y = var.first.vertex_B_[1];
    p.z = var.first.vertex_B_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.first.vertex_C_[0];
    p.y = var.first.vertex_C_[1];
    p.z = var.first.vertex_C_[2];
    m_line.points.push_back(p);
    p.x = var.first.vertex_A_[0];
    p.y = var.first.vertex_A_[1];
    p.z = var.first.vertex_A_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
  }
  for (int j = 0; j < 100 * 6; j++) {
    m_line.color.a = 0.00;
    ma_line.markers.push_back(m_line);
    m_line.id++;
  }
  std_publisher->publish(ma_line);
  m_line.id = 0;
  ma_line.markers.clear();
}