#ifndef _PGO_R2H_PGO_HPP_
#define _PGO_R2H_PGO_HPP_


#define _USE_MATH_DEFINES


#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <queue>
#include <deque>
#include <mutex>
#include <memory>
#include <vector>
#include <thread>
#include <chrono>
#include <string>
#include <utility>
#include <iostream>
#include <functional>

#include <Eigen/Eigen>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "pgo_r2h/lib/time.hpp"
#include "pgo_r2h/lib/conversion.hpp"
#include "pgo_r2h/lib/ros2.hpp"


/* --------------------------------------------------------------------------------------------- */


struct PGOParam{

  /* frame id */
  std::string frame_id_slam_frame;


  /* input */
  std::string topic_sub_odom;
  std::string topic_sub_cloud;


  /* keyframe selection */
  double keyframe_gap_meter;
  double keyframe_gap_deg;


  /* voxel grid filter */
  double leaf_size_keyframe;
  double leaf_size_icp;


  /* loop candidate */
  double lc_max_radius_m;
  double lc_min_time_diff_s;


  /* ICP test */
  int    icp_type;
  int    icp_stack_size;
  int    icp_config_max_iter;
  double icp_config_max_cor_dist;
  double icp_test_max_fitness;


  /* visualization */
  bool enable_pub_cloud_keyframe;

  bool enable_pub_icp;

  bool   enable_pub_graph;
  double nodes_scale;
  double nodes_alpha;
  double edges_scale;
  double edges_alpha;
  double loops_pass_scale;
  double loops_pass_alpha;
  double loops_fail_scale;
  double loops_fail_alpha;


  /* ---------- */


  /* derived */
  double keyframe_gap_rad;

};


/* --------------------------------------------------------------------------------------------- */


struct PGOPose{

  /* time [s] */
  double t;

  /* position [m] */
  double px;
  double py;
  double pz;

  /* quaternion */
  double qw;
  double qx;
  double qy;
  double qz;

  /* valid */
  bool valid;

};


struct PGOData{

  /* subscription buffer */
  std::queue<nav_msgs::msg::Odometry::SharedPtr>       buf_odom;  // NOTE: SHARED
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> buf_cloud; // NOTE: SHARED

  /* keyframe selection */
  PGOPose last_kf_pose;

  /* voxel grid filter */
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_kf;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_icp;

  /* keyframe data */
  std::deque<PGOPose>                              kf_poses;     // NOTE: SHARED
  std::deque<PGOPose>                              kf_poses_opt; // NOTE: SHARED
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> kf_clouds;    // NOTE: SHARED
  int64_t                                          kf_size = 0;  // NOTE: SHARED

  /* loop candidate */
  pcl::PointCloud<pcl::PointXYZ>::Ptr kf_positions;
  std::queue<std::pair<int, int>>     buf_loop_candidate; // NOTE: SHARED

  /* pose graph (visualization) */
  visualization_msgs::msg::Marker vis_graph_nodes;      // NOTE: SHARED
  visualization_msgs::msg::Marker vis_graph_edges;      // NOTE: SHARED
  visualization_msgs::msg::Marker vis_graph_loops_fail; // NOTE: SHARED
  visualization_msgs::msg::Marker vis_graph_loops_pass; // NOTE: SHARED

};


/* --------------------------------------------------------------------------------------------- */


class PGO{

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* constructor */
  explicit PGO(const rclcpp::Node::SharedPtr& node);

  /* APIs */
  void init(void);
  void run(void);

private:

  /* node */
  const rclcpp::Node::SharedPtr node_;


  /* parameter */
  PGOParam param_;


  /* data */
  PGOData data_;


  /* mutex */
  std::mutex mtx_sub_;       // subscription
  std::mutex mtx_kf_;        // keyframe
  std::mutex mtx_lc_;        // loop candidate
  std::mutex mtx_vis_graph_; // visualization of the graph


  /* publication */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_cloud_keyframe_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_graph_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_icp_source_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_icp_target_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_icp_aligned_;


  /* subscription */
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>>                                           mf_sub_odom_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>                                     mf_sub_cloud_;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2> MFSyncPolicyOdomCloud;
  typedef std::shared_ptr<message_filters::Synchronizer<MFSyncPolicyOdomCloud>>                                   MFSyncOdomCloud;
  MFSyncOdomCloud                                                                                                 mf_sync_odom_cloud_;
  void callback_mf_sync_odom_cloud(const nav_msgs::msg::Odometry::SharedPtr msg_odom, const sensor_msgs::msg::PointCloud2::SharedPtr msg_cloud);


  /* main thread functions */
  void func_pose_graph(void);   // NOTE: add frame-to-frame factor
  void func_loop_closure(void); // NOTE: add loop factor


  /* ---------- */


  /* APIs: conversion */
  double stamp_to_second(const builtin_interfaces::msg::Time& stamp);
  builtin_interfaces::msg::Time second_to_stamp(const double second);

  PGOPose odom_to_pgo_pose(const nav_msgs::msg::Odometry::SharedPtr& odom);

  geometry_msgs::msg::Point pgo_pose_to_msg_point(const PGOPose& pose);
  pcl::PointXYZ             pgo_pose_to_pcl_point(const PGOPose& pose);
  Eigen::Matrix4d           pgo_pose_to_tf(       const PGOPose& pose);


  /* APIs: keyframe */
  bool is_keyframe(const PGOPose& cur_pose);


  /* APIs: loop closure */
  bool is_loop(const std::pair<int, int>& loop_candidate, Eigen::Matrix4d& icp_tf_source_to_target);


  /* APIs: visualization */
  void init_vis_graph_all(void);

};


/* --------------------------------------------------------------------------------------------- */


#endif
