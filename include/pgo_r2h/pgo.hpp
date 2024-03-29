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
  double leaf_size_out_map;


  /* loop candidate */
  double lc_max_radius_m;
  double lc_min_time_diff_s;


  /* ICP test */
  int    icp_type;
  int    icp_stack_size;
  int    icp_config_max_iter;
  double icp_config_max_cor_dist;
  double icp_test_max_fitness;


  /* GTSAM factor noise variance */
  double var_prior;
  double var_odom;
  double var_loop;


  /* GTSAM pose graph optimization */
  int num_update;


  /* outout */
  bool enable_out_map;


  /* visualization */
  bool enable_pub_keyframe_cloud;

  bool   enable_pub_graph;
  double nodes_scale;
  double nodes_alpha;
  double edges_scale;
  double edges_alpha;
  double loops_pass_scale;
  double loops_pass_alpha;
  double loops_fail_scale;
  double loops_fail_alpha;

  bool enable_pub_icp;


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
  std::queue<nav_msgs::msg::Odometry::SharedPtr>       buf_odom;  // NOTE: SHARED, use "mtx_sub_"
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> buf_cloud; // NOTE: SHARED, use "mtx_sub_"


  /* keyframe selection */
  PGOPose last_kf_pose;


  /* voxel grid filter */
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_kf;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_icp;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_out_map;


  /* keyframe data */
  std::deque<PGOPose>                              kf_poses;     // NOTE: SHARED, use "mtx_kf_"
  std::deque<PGOPose>                              kf_poses_opt; // NOTE: SHARED, use "mtx_kf_"
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> kf_clouds;    // NOTE: SHARED, use "mtx_kf_"
  int64_t                                          kf_size = 0;  // NOTE: SHARED, use "mtx_kf_"

  pcl::PointCloud<pcl::PointXYZ>::Ptr              kf_positions; // NOTE: SHARED, use "mtx_kf_" (for KDTree radius search)


  /* pose graph */
  bool                      has_graph = false;
  gtsam::NonlinearFactorGraph   graph;         // NOTE: SHARED, use "mtx_graph_"
  gtsam::Values                 init_estimate; // NOTE: SHARED, use "mtx_graph_"
  gtsam::Values                 curr_estimate;
  std::shared_ptr<gtsam::ISAM2> isam2;

  gtsam::noiseModel::Diagonal::shared_ptr noise_prior;
  gtsam::noiseModel::Diagonal::shared_ptr noise_odom;
  gtsam::noiseModel::Base::shared_ptr     noise_loop;


  /* loop candidate */
  // NOTE:   std::pair<prv, cur>
  std::queue<std::pair<int, int>> buf_loop_candidate; // NOTE: SHARED, use "mtx_lc_"
  std::deque<int>                 loops_fail;         // NOTE: SHARED, use "mtx_lc_"
  std::deque<int>                 loops_pass;         // NOTE: SHARED, use "mtx_lc_"


  /* visualization */
  bool                        run_visualization = false; // NOTE: SHARED, use "mtx_bools_"
  visualization_msgs::msg::Marker vis_graph_nodes;
  visualization_msgs::msg::Marker vis_graph_edges;
  visualization_msgs::msg::Marker vis_graph_loops_fail;
  visualization_msgs::msg::Marker vis_graph_loops_pass;

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
  std::mutex mtx_sub_;   // subscription
  std::mutex mtx_kf_;    // keyframe
  std::mutex mtx_graph_; // pose graph
  std::mutex mtx_lc_;    // loop candidate
  std::mutex mtx_bools_; // helper mutex for booleans


  /* publication */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_keyframe_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_graph_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_icp_source_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_icp_target_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_icp_aligned_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_out_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_test_a;


  /* subscription */
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>>                                           mf_sub_odom_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>                                     mf_sub_cloud_;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2> MFSyncPolicyOdomCloud;
  typedef std::shared_ptr<message_filters::Synchronizer<MFSyncPolicyOdomCloud>>                                   MFSyncOdomCloud;
  MFSyncOdomCloud                                                                                                 mf_sync_odom_cloud_;
  void callback_mf_sync_odom_cloud(const nav_msgs::msg::Odometry::SharedPtr msg_odom, const sensor_msgs::msg::PointCloud2::SharedPtr msg_cloud);


  /* main thread functions */
  void func_pose_graph(void);   // NOTE: add prior/odometry factor + optimization
  void func_loop_closure(void); // NOTE: add   loop closure factor + optimization
  void func_vis_and_out(void);


  /* ---------- */


  /* APIs: conversion */
  double stamp_to_second(const builtin_interfaces::msg::Time& stamp);
  builtin_interfaces::msg::Time second_to_stamp(const double second);

  PGOPose odom_to_pgo_pose(const nav_msgs::msg::Odometry::SharedPtr& odom);

  geometry_msgs::msg::Point pgo_pose_to_msg_point(  const PGOPose& pose);
  pcl::PointXYZ             pgo_pose_to_pcl_point(  const PGOPose& pose);
  gtsam::Pose3              pgo_pose_to_gtsam_pose3(const PGOPose& pose);
  Eigen::Matrix4d           pgo_pose_to_tf(         const PGOPose& pose);

  gtsam::Pose3 tf_to_gtsam_pose3(const Eigen::Matrix4d& tf);


  /* APIs: keyframe selection */
  bool is_keyframe(const PGOPose& cur_pose);


  /* APIs: loop closure */
  // NOTE:           std::pair<prv, cur>
  bool is_loop(const std::pair<int, int>& loop_candidate, gtsam::Pose3& pose_from_cur_to_prv);


  /* APIs: pose graph optimization */
  void pose_graph_opt(void);


  /* APIs: visualization and output */
  void init_vis_graph_all(void);
  void vis_graph(void);
  void out_map(void);

};


/* --------------------------------------------------------------------------------------------- */


#endif
