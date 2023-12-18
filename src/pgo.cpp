#include "pgo_r2h/pgo.hpp"


PGO::PGO(const rclcpp::Node::SharedPtr& node) : node_(node) {

  /* print */
  std::printf("[INFO] class PGO has been created\n");

}


void PGO::init(void){

  /* parameter: declare and get */
  lib::ros2::declare_get_param_or_exit(node_, "frame_id_slam_frame",       rclcpp::PARAMETER_STRING, param_.frame_id_slam_frame,       true);
  lib::ros2::declare_get_param_or_exit(node_, "topic_sub_odom",            rclcpp::PARAMETER_STRING, param_.topic_sub_odom,            true);
  lib::ros2::declare_get_param_or_exit(node_, "topic_sub_cloud",           rclcpp::PARAMETER_STRING, param_.topic_sub_cloud,           true);
  lib::ros2::declare_get_param_or_exit(node_, "keyframe_gap_meter",        rclcpp::PARAMETER_DOUBLE, param_.keyframe_gap_meter,        true);
  lib::ros2::declare_get_param_or_exit(node_, "keyframe_gap_deg",          rclcpp::PARAMETER_DOUBLE, param_.keyframe_gap_deg,          true);
  lib::ros2::declare_get_param_or_exit(node_, "leaf_size_icp",             rclcpp::PARAMETER_DOUBLE, param_.leaf_size_icp,             true);
  lib::ros2::declare_get_param_or_exit(node_, "enable_pub_cloud_keyframe", rclcpp::PARAMETER_BOOL,   param_.enable_pub_cloud_keyframe, true);
  lib::ros2::declare_get_param_or_exit(node_, "enable_pub_graph",          rclcpp::PARAMETER_BOOL,   param_.enable_pub_graph,          true);
  lib::ros2::declare_get_param_or_exit(node_, "nodes_scale",               rclcpp::PARAMETER_DOUBLE, param_.nodes_scale,               true);
  lib::ros2::declare_get_param_or_exit(node_, "nodes_alpha",               rclcpp::PARAMETER_DOUBLE, param_.nodes_alpha,               true);
  lib::ros2::declare_get_param_or_exit(node_, "edges_scale",               rclcpp::PARAMETER_DOUBLE, param_.edges_scale,               true);
  lib::ros2::declare_get_param_or_exit(node_, "edges_alpha",               rclcpp::PARAMETER_DOUBLE, param_.edges_alpha,               true);
  lib::ros2::declare_get_param_or_exit(node_, "loops_pass_scale",          rclcpp::PARAMETER_DOUBLE, param_.loops_pass_scale,          true);
  lib::ros2::declare_get_param_or_exit(node_, "loops_pass_alpha",          rclcpp::PARAMETER_DOUBLE, param_.loops_pass_alpha,          true);
  lib::ros2::declare_get_param_or_exit(node_, "loops_fail_scale",          rclcpp::PARAMETER_DOUBLE, param_.loops_fail_scale,          true);
  lib::ros2::declare_get_param_or_exit(node_, "loops_fail_alpha",          rclcpp::PARAMETER_DOUBLE, param_.loops_fail_alpha,          true);
  lib::ros2::declare_get_param_or_exit(node_, "print_thread_pose_graph",   rclcpp::PARAMETER_BOOL,   param_.print_thread_pose_graph,   true);


  /* parameter: derived */
  param_.keyframe_gap_rad = param_.keyframe_gap_deg / 180.0 * M_PI;


  /* data */
  // ...
  // ...
  // ...


  /* publication */
  pub_cloud_keyframe_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(       "scpgo/cloud/keyframe", rclcpp::QoS(10));
  pub_graph_          = node_->create_publisher<visualization_msgs::msg::MarkerArray>("scpgo/graph",          rclcpp::QoS(10));
  pub_icp_source_     = node_->create_publisher<sensor_msgs::msg::PointCloud2>(       "scpgo/icp/source",     rclcpp::QoS(10));
  pub_icp_target_     = node_->create_publisher<sensor_msgs::msg::PointCloud2>(       "scpgo/icp/target",     rclcpp::QoS(10));
  pub_icp_align_      = node_->create_publisher<sensor_msgs::msg::PointCloud2>(       "scpgo/icp/align",      rclcpp::QoS(10));


  /* subscription */
  int  mf_qos_profile_depth = 100; // NOTE: Reference: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html#qos-policies
  auto mf_qos_profile = rmw_qos_profile_default;    mf_qos_profile.depth = mf_qos_profile_depth;
  mf_sub_odom_.reset( new message_filters::Subscriber<nav_msgs::msg::Odometry>(      node_, param_.topic_sub_odom,  mf_qos_profile));
  mf_sub_cloud_.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(node_, param_.topic_sub_cloud, mf_qos_profile));
  mf_sync_odom_cloud_.reset(new message_filters::Synchronizer<MFSyncPolicyOdomCloud>(MFSyncPolicyOdomCloud(mf_qos_profile_depth), *mf_sub_odom_, *mf_sub_cloud_));
  mf_sync_odom_cloud_->registerCallback(&PGO::callback_mf_sync_odom_cloud, this);

}


void PGO::run(void){

  /* spin */
  rclcpp::spin(node_);

}


/* --------------------------------------------------------------------------------------------- */


void PGO::callback_mf_sync_odom_cloud(const nav_msgs::msg::Odometry::SharedPtr msg_odom, const sensor_msgs::msg::PointCloud2::SharedPtr msg_cloud){
  double t_odom_s  = double(msg_odom->header.stamp.sec)  + double(msg_odom->header.stamp.nanosec)  / 1e9;
  double t_cloud_s = double(msg_cloud->header.stamp.sec) + double(msg_cloud->header.stamp.nanosec) / 1e9;
  double t_diff_ms = (t_odom_s - t_cloud_s) * 1e3;
  printf("time difference in [ms]: %10.4f\n", t_diff_ms);
}
