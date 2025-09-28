#include "include/STDescCore.h"
#include "include/STDescROS.h"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>
#include <chrono>

int findPoseIndexUsingTime(std::vector<double> &time_list, double &time) {
  double time_inc = 10000000000;
  int min_index = -1;
  for (size_t i = 0; i < time_list.size(); i++) {
    if (fabs(time_list[i] - time) < time_inc) {
      time_inc = fabs(time_list[i] - time);
      min_index = i;
    }
  }
  if (time_inc > 0.5) {
    std::string msg = "The timestamp between poses and point cloud is:" +
                      std::to_string(time_inc) + "s. Please check it!";
    RCLCPP_ERROR(rclcpp::get_logger("livox_demo"), "%s", msg.c_str());
    std::cout << "Timestamp for point cloud:" << time << std::endl;
    std::cout << "Timestamp for pose:" << time_list[min_index] << std::endl;
    exit(-1);
  }
  return min_index;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("demo_livox");

  std::string bag_path = "";
  std::string pose_path = "";
  node->declare_parameter("bag_path", "");
  node->declare_parameter("pose_path", "");
  node->get_parameter("bag_path", bag_path);
  node->get_parameter("pose_path", pose_path);

  ConfigSetting config_setting;
  read_parameters(node, config_setting);

  auto pubOdomAftMapped = node->create_publisher<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init", 10);
  auto pubCureentCloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_current", 100);
  auto pubCurrentCorner = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_key_points", 100);
  auto pubMatchedCloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_matched", 100);
  auto pubMatchedCorner = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_matched_key_points", 100);
  auto pubSTD = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "descriptor_line", 10);

  rclcpp::Rate loop(500);
  rclcpp::Rate slow_loop(10);

  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> poses_vec;
  std::vector<double> times_vec;
  load_pose_with_time(pose_path, poses_vec, times_vec);
  std::cout << "Successfully load pose with number: " << poses_vec.size()
            << std::endl;

  STDescManager *std_manager = new STDescManager(config_setting);

  size_t cloudInd = 0;
  size_t keyCloudInd = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());

  std::vector<double> descriptor_time;
  std::vector<double> querying_time;
  std::vector<double> update_time;
  int triggle_loop_num = 0;

  // Setup rosbag2 reader
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  try {
    reader.open(storage_options, converter_options);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("livox_demo"), "Failed to open bag: %s", e.what());
    return -1;
  }

  // Set topic filter for PointCloud2 messages
  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back("sensor_msgs/msg/PointCloud2");
  reader.set_filter(filter);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;

  while (rclcpp::ok() && reader.has_next()) {
    auto bag_message = reader.read_next();

    if (bag_message->topic_name.find("PointCloud2") != std::string::npos ||
        bag_message->topic_name.find("point") != std::string::npos) {

      auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);

      try {
        serialization.deserialize_message(&serialized_msg, cloud_msg.get());
      } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("livox_demo"), "Failed to deserialize message: %s", e.what());
        continue;
      }

      double laser_time = rclcpp::Time(cloud_msg->header.stamp).seconds();

      pcl::PCLPointCloud2 pcl_pc;
      pcl_conversions::toPCL(*cloud_msg, pcl_pc);
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::fromPCLPointCloud2(pcl_pc, cloud);

      int pose_index = findPoseIndexUsingTime(times_vec, laser_time);
      Eigen::Vector3d translation = poses_vec[pose_index].first;
      Eigen::Matrix3d rotation = poses_vec[pose_index].second;

      for (size_t i = 0; i < cloud.size(); i++) {
        Eigen::Vector3d pv = point2vec(cloud.points[i]);
        pv = rotation * pv + translation;
        cloud.points[i] = vec2point(pv);
      }
      down_sampling_voxel(cloud, config_setting.ds_size_);
      for (auto pv : cloud.points) {
        temp_cloud->points.push_back(pv);
      }

      // check if keyframe
      if (cloudInd % config_setting.sub_frame_num_ == 0 && cloudInd != 0) {
        std::cout << "Key Frame id:" << keyCloudInd
                  << ", cloud size: " << temp_cloud->size() << std::endl;
        // step1. Descriptor Extraction
        auto t_descriptor_begin = std::chrono::high_resolution_clock::now();
        std::vector<STDesc> stds_vec;
        std_manager->GenerateSTDescs(temp_cloud, stds_vec);
        auto t_descriptor_end = std::chrono::high_resolution_clock::now();
        descriptor_time.push_back(
            time_inc(t_descriptor_end, t_descriptor_begin));
        // step2. Searching Loop
        auto t_query_begin = std::chrono::high_resolution_clock::now();
        std::pair<int, double> search_result(-1, 0);
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
        loop_transform.first << 0, 0, 0;
        loop_transform.second = Eigen::Matrix3d::Identity();
        std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
        if (keyCloudInd > config_setting.skip_near_num_) {
          std_manager->SearchLoop(stds_vec, search_result, loop_transform,
                                  loop_std_pair);
        }
        if (search_result.first > 0) {
          std::cout << "[Loop Detection] triggle loop: " << keyCloudInd
                    << "--" << search_result.first
                    << ", score:" << search_result.second << std::endl;
        }
        auto t_query_end = std::chrono::high_resolution_clock::now();
        querying_time.push_back(time_inc(t_query_end, t_query_begin));

        // step3. Add descriptors to the database
        auto t_map_update_begin = std::chrono::high_resolution_clock::now();
        std_manager->AddSTDescs(stds_vec);
        auto t_map_update_end = std::chrono::high_resolution_clock::now();
        update_time.push_back(time_inc(t_map_update_end, t_map_update_begin));
        std::cout << "[Time] descriptor extraction: "
                  << time_inc(t_descriptor_end, t_descriptor_begin) << "ms, "
                  << "query: " << time_inc(t_query_end, t_query_begin)
                  << "ms, "
                  << "update map:"
                  << time_inc(t_map_update_end, t_map_update_begin) << "ms"
                  << std::endl;
        std::cout << std::endl;

        pcl::PointCloud<pcl::PointXYZI> save_key_cloud;
        save_key_cloud = *temp_cloud;

        std_manager->key_cloud_vec_.push_back(save_key_cloud.makeShared());

        // publish
        sensor_msgs::msg::PointCloud2 pub_cloud;
        pcl::toROSMsg(*temp_cloud, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubCureentCloud->publish(pub_cloud);
        pcl::toROSMsg(*std_manager->corner_cloud_vec_.back(), pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubCurrentCorner->publish(pub_cloud);

        if (search_result.first > 0) {
          triggle_loop_num++;
          pcl::toROSMsg(*std_manager->key_cloud_vec_[search_result.first],
                        pub_cloud);
          pub_cloud.header.frame_id = "camera_init";
          pubMatchedCloud->publish(pub_cloud);
          slow_loop.sleep();
          pcl::toROSMsg(*std_manager->corner_cloud_vec_[search_result.first],
                        pub_cloud);
          pub_cloud.header.frame_id = "camera_init";
          pubMatchedCorner->publish(pub_cloud);
          publish_std_pairs(loop_std_pair, pubSTD);
          slow_loop.sleep();
        }
        temp_cloud->clear();
        keyCloudInd++;
        loop.sleep();
      }
      nav_msgs::msg::Odometry odom;
      odom.header.frame_id = "camera_init";
      odom.pose.pose.position.x = translation[0];
      odom.pose.pose.position.y = translation[1];
      odom.pose.pose.position.z = translation[2];
      Eigen::Quaterniond q(rotation);
      odom.pose.pose.orientation.w = q.w();
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      pubOdomAftMapped->publish(odom);
      loop.sleep();
      cloudInd++;
    }
  }

  double mean_descriptor_time =
      std::accumulate(descriptor_time.begin(), descriptor_time.end(), 0) *
      1.0 / descriptor_time.size();
  double mean_query_time =
      std::accumulate(querying_time.begin(), querying_time.end(), 0) * 1.0 /
      querying_time.size();
  double mean_update_time =
      std::accumulate(update_time.begin(), update_time.end(), 0) * 1.0 /
      update_time.size();
  std::cout << "Total key frame number:" << keyCloudInd
            << ", loop number:" << triggle_loop_num << std::endl;
  std::cout << "Mean time for descriptor extraction: " << mean_descriptor_time
            << "ms, query: " << mean_query_time
            << "ms, update: " << mean_update_time << "ms, total: "
            << mean_descriptor_time + mean_query_time + mean_update_time
            << "ms" << std::endl;

  rclcpp::shutdown();
  return 0;
}