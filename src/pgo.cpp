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
  lib::ros2::declare_get_param_or_exit(node_, "leaf_size_keyframe",        rclcpp::PARAMETER_DOUBLE, param_.leaf_size_keyframe,        true);
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
  data_.voxel_grid_kf.setLeafSize(param_.leaf_size_keyframe, param_.leaf_size_keyframe, param_.leaf_size_keyframe);
  data_.voxel_grid_icp.setLeafSize(param_.leaf_size_icp, param_.leaf_size_icp, param_.leaf_size_icp);

  data_.last_kf_pose.valid = false;

  init_vis_graph_all();


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

  /* threads */
  std::thread thread_pose_graph(std::bind(&PGO::func_pose_graph, this));

  /* spin */
  rclcpp::spin(node_);

}


/* --------------------------------------------------------------------------------------------- */


void PGO::callback_mf_sync_odom_cloud(const nav_msgs::msg::Odometry::SharedPtr msg_odom, const sensor_msgs::msg::PointCloud2::SharedPtr msg_cloud){

  /* update: buffer */
  mtx_buf_.lock();
  data_.buf_odom.push(msg_odom);
  data_.buf_cloud.push(msg_cloud);
  mtx_buf_.unlock();

}


/* --------------------------------------------------------------------------------------------- */


void PGO::func_pose_graph(void){

  /* infinite loop */
  while (true){

    /* check: buffer */
    while ((!data_.buf_odom.empty()) && (!data_.buf_cloud.empty())){

      /* sample */
      mtx_buf_.lock();
      PGOPose cur_pose = odom_to_pgo_pose(data_.buf_odom.front());
      pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*data_.buf_cloud.front(), *cur_cloud);
      data_.buf_odom.pop();
      data_.buf_cloud.pop();
      mtx_buf_.unlock();


      /* is it keyframe ? */
      if (is_keyframe(cur_pose)) { data_.last_kf_pose = cur_pose; } // NOTE: update last keyframe pose
      else                       { continue;                      } // NOTE: continue


      /* cloud down sampling */
      pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud_ds(new pcl::PointCloud<pcl::PointXYZI>());
      data_.voxel_grid_kf.setInputCloud(cur_cloud);
      data_.voxel_grid_kf.filter(*cur_cloud_ds);


      /* publish: keyframe cloud */
      if (param_.enable_pub_cloud_keyframe){

        /* declaration */
        pcl::PointCloud<pcl::PointXYZI>::Ptr msg_pcl(new pcl::PointCloud<pcl::PointXYZI>());
        sensor_msgs::msg::PointCloud2        msg_ros;

        /* transform */
        Eigen::Matrix4d tf = lib::conversion::get_transformation_matrix(cur_pose.px, cur_pose.py, cur_pose.pz,
                                                                        cur_pose.qw, cur_pose.qx, cur_pose.qy, cur_pose.qz);
        pcl::transformPointCloud(*cur_cloud_ds, *msg_pcl, tf);

        /* publish */
        pcl::toROSMsg(*msg_pcl, msg_ros);
        msg_ros.header.frame_id = param_.frame_id_slam_frame;
        msg_ros.header.stamp    = second_to_stamp(cur_pose.t);
        pub_cloud_keyframe_->publish(msg_ros);

      }


      /* update: keyframe data */
      mtx_kf_.lock();
      // HERE
      mtx_kf_.unlock();


      // HERE: temporal vis code
      {
        auto point = pgo_pose_to_msg_point(cur_pose);
        auto stamp = lib::ros2::get_stamp();

        data_.vis_graph_nodes.points.push_back(point);
        data_.vis_graph_nodes.header.stamp = stamp;

        data_.vis_graph_edges.points.push_back(point);
        data_.vis_graph_edges.header.stamp = stamp;

        visualization_msgs::msg::MarkerArray ma;
        ma.markers.push_back(data_.vis_graph_nodes);
        ma.markers.push_back(data_.vis_graph_edges);
        ma.markers.push_back(data_.vis_graph_loops_fail);
        ma.markers.push_back(data_.vis_graph_loops_pass);
        pub_graph_->publish(ma);
      }

    }

    /* sleep */
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

  }

}


/* --------------------------------------------------------------------------------------------- */


double PGO::stamp_to_second(const builtin_interfaces::msg::Time& stamp){
  double x = double(stamp.sec) + double(stamp.nanosec) / 1e9;
  return x;
}


builtin_interfaces::msg::Time PGO::second_to_stamp(const double second){
  builtin_interfaces::msg::Time x;
  x.sec     =  int32_t(std::floor(second));
  x.nanosec = uint32_t(second - std::floor(second) * 1e9);
  return x;
}


PGOPose PGO::odom_to_pgo_pose(const nav_msgs::msg::Odometry::SharedPtr& odom){
  PGOPose pose;
  pose.t     = stamp_to_second(odom->header.stamp);
  pose.px    = odom->pose.pose.position.x;
  pose.py    = odom->pose.pose.position.y;
  pose.pz    = odom->pose.pose.position.z;
  pose.qw    = odom->pose.pose.orientation.w;
  pose.qx    = odom->pose.pose.orientation.x;
  pose.qy    = odom->pose.pose.orientation.y;
  pose.qz    = odom->pose.pose.orientation.z;
  pose.valid = true;
  return pose;
}


geometry_msgs::msg::Point PGO::pgo_pose_to_msg_point(const PGOPose& pose){
  geometry_msgs::msg::Point point;
  point.x = pose.px;
  point.y = pose.py;
  point.z = pose.pz;
  return point;
}


bool PGO::is_keyframe(const PGOPose& cur_pose){

  /* check: valid */
  if (!data_.last_kf_pose.valid) { return true; }

  /* delta translation */
  Eigen::Vector3d last_pos(data_.last_kf_pose.px, data_.last_kf_pose.py, data_.last_kf_pose.pz);
  Eigen::Vector3d  cur_pos(cur_pose.px, cur_pose.py, cur_pose.pz);
  double del_trs = (cur_pos - last_pos).norm();
  if (del_trs > param_.keyframe_gap_meter) { return true; }

  /* delta rotation */
  Eigen::Matrix3d last_rot;
  Eigen::Matrix3d  cur_rot;
  lib::conversion::quaternion_to_rotation_matrix(data_.last_kf_pose.qw, data_.last_kf_pose.qx, data_.last_kf_pose.qy, data_.last_kf_pose.qz, last_rot);
  lib::conversion::quaternion_to_rotation_matrix(cur_pose.qw, cur_pose.qx, cur_pose.qy, cur_pose.qz, cur_rot);
  Eigen::AngleAxisd del_angle_axis(last_rot.transpose() * cur_rot);
  if (del_angle_axis.angle() > param_.keyframe_gap_rad) { return true; }

  /* return */
  return false;

}


void PGO::init_vis_graph_all(void){

  /* nodes */
  visualization_msgs::msg::Marker& nodes = data_.vis_graph_nodes;
  nodes.header.frame_id    = param_.frame_id_slam_frame;
  nodes.ns                 = "nodes";
  nodes.id                 = 0;
  nodes.type               = visualization_msgs::msg::Marker::SPHERE_LIST;
  nodes.action             = visualization_msgs::msg::Marker::ADD;
  nodes.pose.orientation.w = 1;
  nodes.scale.x            = param_.nodes_scale;
  nodes.scale.y            = param_.nodes_scale;
  nodes.scale.z            = param_.nodes_scale;
  nodes.color.r            = 0;
  nodes.color.g            = 1;
  nodes.color.b            = 1;
  nodes.color.a            = param_.nodes_alpha;
  nodes.points.clear();

  /* edges */
  visualization_msgs::msg::Marker& edges = data_.vis_graph_edges;
  edges.header.frame_id    = param_.frame_id_slam_frame;
  edges.ns                 = "edges";
  edges.id                 = 0;
  edges.type               = visualization_msgs::msg::Marker::LINE_STRIP;
  edges.action             = visualization_msgs::msg::Marker::ADD;
  edges.pose.orientation.w = 1;
  edges.scale.x            = param_.edges_scale;
  edges.color.r            = 0;
  edges.color.g            = 1;
  edges.color.b            = 1;
  edges.color.a            = param_.edges_alpha;
  edges.points.clear();

  /* loops fail */
  visualization_msgs::msg::Marker& lf = data_.vis_graph_loops_fail;
  lf.header.frame_id    = param_.frame_id_slam_frame;
  lf.ns                 = "loops_fail";
  lf.id                 = 0;
  lf.type               = visualization_msgs::msg::Marker::LINE_LIST;
  lf.action             = visualization_msgs::msg::Marker::ADD;
  lf.pose.orientation.w = 1;
  lf.scale.x            = param_.loops_fail_scale;
  lf.color.r            = 1;
  lf.color.g            = 0;
  lf.color.b            = 1;
  lf.color.a            = param_.loops_fail_alpha;
  lf.points.clear();

  /* loops pass */
  visualization_msgs::msg::Marker& lp = data_.vis_graph_loops_pass;
  lp.header.frame_id    = param_.frame_id_slam_frame;
  lp.ns                 = "loops_pass";
  lp.id                 = 0;
  lp.type               = visualization_msgs::msg::Marker::LINE_LIST;
  lp.action             = visualization_msgs::msg::Marker::ADD;
  lp.pose.orientation.w = 1;
  lp.scale.x            = param_.loops_pass_scale;
  lp.color.r            = 1;
  lp.color.g            = 1;
  lp.color.b            = 0;
  lp.color.a            = param_.loops_pass_alpha;
  lp.points.clear();

}
