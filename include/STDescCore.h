#pragma once

#include "omp.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <fstream>
#include <mutex>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <unordered_map>

#define HASH_P 116101
#define MAX_N 10000000000
#define MAX_FRAME_N 20000

typedef struct ConfigSetting {
  /* for point cloud pre-preocess*/
  int stop_skip_enable_ = 0;
  double ds_size_ = 0.5;
  int maximum_corner_num_ = 30;

  /* for key points*/
  double plane_merge_normal_thre_;
  double plane_merge_dis_thre_;
  double plane_detection_thre_ = 0.01;
  double voxel_size_ = 1.0;
  int voxel_init_num_ = 10;
  double proj_image_resolution_ = 0.5;
  double proj_dis_min_ = 0.2;
  double proj_dis_max_ = 5;
  double corner_thre_ = 10;

  /* for STD */
  int descriptor_near_num_ = 10;
  double descriptor_min_len_ = 1;
  double descriptor_max_len_ = 10;
  double non_max_suppression_radius_ = 3.0;
  double std_side_resolution_ = 0.2;

  /* for place recognition*/
  int skip_near_num_ = 50;
  int candidate_num_ = 50;
  int sub_frame_num_ = 10;
  double rough_dis_threshold_ = 0.03;
  double vertex_diff_threshold_ = 0.7;
  double icp_threshold_ = 0.5;
  double normal_threshold_ = 0.1;
  double dis_threshold_ = 0.3;

} ConfigSetting;

// Structure for Stabel Triangle Descriptor
typedef struct STDesc {
  // the side lengths of STDesc, arranged from short to long
  Eigen::Vector3d side_length_;

  // projection angle between vertices
  Eigen::Vector3d angle_;

  Eigen::Vector3d center_;
  unsigned int frame_id_;

  // three vertexs
  Eigen::Vector3d vertex_A_;
  Eigen::Vector3d vertex_B_;
  Eigen::Vector3d vertex_C_;

  // some other inform attached to each vertex,e.g., intensity
  Eigen::Vector3d vertex_attached_;
} STDesc;

// plane structure for corner point extraction
typedef struct Plane {
  pcl::PointXYZINormal p_center_;
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Matrix3d covariance_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float intercept_ = 0;
  int id_ = 0;
  int sub_plane_num_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
} Plane;

typedef struct STDMatchList {
  std::vector<std::pair<STDesc, STDesc>> match_list_;
  std::pair<int, int> match_id_;
  double mean_dis_;
} STDMatchList;

struct M_POINT {
  float xyz[3];
  float intensity;
  int count = 0;
};

struct VOXEL_LOC {
  int64_t x, y, z;

  VOXEL_LOC() : x(0), y(0), z(0) {}
  VOXEL_LOC(int64_t x_, int64_t y_, int64_t z_) : x(x_), y(y_), z(z_) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

struct VOXEL_LOC_HASHER {
  int64_t operator()(const VOXEL_LOC &voxel_loc) const {
    int64_t kx1 = (int64_t)voxel_loc.x;
    int64_t ky1 = (int64_t)voxel_loc.y;
    int64_t kz1 = (int64_t)voxel_loc.z;
    return (int64_t)((((int64_t)73856093 * kx1) ^ ((int64_t)19349669 * ky1) ^
                      ((int64_t)83492791 * kz1)) %
                     HASH_P);
  }
};

// For STDesc database lookup
typedef VOXEL_LOC STDesc_LOC;

class OctoTree {
public:
  ConfigSetting config_setting_;
  std::vector<Eigen::Vector3d> voxel_points_;
  Plane *plane_ptr_;
  int layer_;
  int octo_state_; // 0 is end of tree, 1 is not
  std::vector<OctoTree *> leaves_;
  bool is_check_connect_[6];
  bool connect_[6];
  OctoTree *connect_tree_[6];
  double voxel_center_[3]; // x, y, z
  std::vector<int> layer_point_size_;
  float quater_length_;
  float planer_threshold_;
  int max_layer_;
  int layer_size_;
  bool is_project_ = false;
  std::vector<Eigen::Vector3d> proj_normal_vec_;

  OctoTree(ConfigSetting &config_setting)
      : config_setting_(config_setting), plane_ptr_(new Plane) {}

  void init_plane();
  void init_octo_tree();
};

// Stable Triangle Descriptor Manager
class STDescManager {
public:
  ConfigSetting config_setting_;
  int current_frame_id_;
  std::unordered_map<VOXEL_LOC, std::vector<STDesc>, VOXEL_LOC_HASHER>
      data_base_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_cloud_vec_;
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> corner_cloud_vec_;
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> plane_cloud_vec_;

  STDescManager(ConfigSetting &config_setting)
      : config_setting_(config_setting) {
    current_frame_id_ = 0;
  };

  void GenerateSTDescs(
      pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
      std::vector<STDesc> &stds_vec);

  void SearchLoop(
      const std::vector<STDesc> &stds_vec,
      std::pair<int, double> &loop_result,
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
      std::vector<std::pair<STDesc, STDesc>> &loop_std_pair);

  void AddSTDescs(const std::vector<STDesc> &stds_vec);

  void init_voxel_map(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
      std::unordered_map<VOXEL_LOC, OctoTree *, VOXEL_LOC_HASHER> &voxel_map);

  void build_connection(std::unordered_map<VOXEL_LOC, OctoTree *,
                                           VOXEL_LOC_HASHER> &feat_map);

  void getPlane(const std::unordered_map<VOXEL_LOC, OctoTree *, VOXEL_LOC_HASHER> &voxel_map,
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud);

  void corner_extractor(
      std::unordered_map<VOXEL_LOC, OctoTree *, VOXEL_LOC_HASHER> &voxel_map,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points);

  void extract_corner(
      const Eigen::Vector3d &proj_center, Eigen::Vector3d proj_normal,
      std::vector<Eigen::Vector3d> proj_points,
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points);

  void build_stdesc(
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points,
      std::vector<STDesc> &stds_vec);

  void non_maxi_suppression(
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points);

  void candidate_selector(const std::vector<STDesc> &stds_vec,
                          std::vector<STDMatchList> &candidate_matcher_vec);

  void candidate_verify(
      const STDMatchList &candidate_matcher, double &verify_score,
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> &relative_pose,
      std::vector<std::pair<STDesc, STDesc>> &sucess_match_vec);

  void triangle_solver(std::vector<std::pair<STDesc, STDesc>> &match_vec,
                       std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform);

  double plane_geometric_verify(
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
      const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform);

  void PlaneGeomrtricIcp(
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform);
};

void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size);

double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
                std::chrono::_V2::system_clock::time_point &t_begin);

void load_pose_with_time(
    const std::string &pose_path,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &poses_vec,
    std::vector<double> &times_vec);

pcl::PointXYZI vec2point(const Eigen::Vector3d &vec);
Eigen::Vector3d point2vec(const pcl::PointXYZI &pi);

bool attach_greater_sort(std::pair<double, int> a, std::pair<double, int> b);

class PlaneSolver {
public:
  PlaneSolver(Eigen::Vector3d curr_point_, Eigen::Vector3d curr_normal_,
              Eigen::Vector3d target_point_, Eigen::Vector3d target_normal_);
  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                     const Eigen::Vector3d curr_normal_,
                                     const Eigen::Vector3d target_point_,
                                     const Eigen::Vector3d target_normal_);

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
    Eigen::Matrix<T, 3, 1> cp{T(curr_point_.x()), T(curr_point_.y()),
                              T(curr_point_.z())};
    Eigen::Matrix<T, 3, 1> cn{T(curr_normal_.x()), T(curr_normal_.y()),
                              T(curr_normal_.z())};
    Eigen::Matrix<T, 3, 1> point_w = q_w_curr * cp + t_w_curr;
    Eigen::Matrix<T, 3, 1> point_target{T(target_point_.x()),
                                        T(target_point_.y()),
                                        T(target_point_.z())};
    Eigen::Matrix<T, 3, 1> norm_target{T(target_normal_.x()),
                                       T(target_normal_.y()),
                                       T(target_normal_.z())};
    residual[0] = norm_target.dot(point_w - point_target);
    return true;
  }

  Eigen::Vector3d curr_point_;
  Eigen::Vector3d curr_normal_;
  Eigen::Vector3d target_point_;
  Eigen::Vector3d target_normal_;
};