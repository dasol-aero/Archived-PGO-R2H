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
  lib::ros2::declare_get_param_or_exit(node_, "lc_max_radius_m",           rclcpp::PARAMETER_DOUBLE, param_.lc_max_radius_m,           true);
  lib::ros2::declare_get_param_or_exit(node_, "lc_min_time_diff_s",        rclcpp::PARAMETER_DOUBLE, param_.lc_min_time_diff_s,        true);
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
  data_.last_kf_pose.valid = false;

  data_.voxel_grid_kf.setLeafSize(param_.leaf_size_keyframe, param_.leaf_size_keyframe, param_.leaf_size_keyframe);
  data_.voxel_grid_icp.setLeafSize(param_.leaf_size_icp, param_.leaf_size_icp, param_.leaf_size_icp);

  data_.kf_positions.reset(new pcl::PointCloud<pcl::PointXYZ>());

  init_vis_graph_all();


  /* publication */
  pub_cloud_keyframe_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(       "pgo/cloud/keyframe", rclcpp::QoS(10));
  pub_graph_          = node_->create_publisher<visualization_msgs::msg::MarkerArray>("pgo/graph",          rclcpp::QoS(10));
  pub_icp_source_     = node_->create_publisher<sensor_msgs::msg::PointCloud2>(       "pgo/icp/source",     rclcpp::QoS(10));
  pub_icp_target_     = node_->create_publisher<sensor_msgs::msg::PointCloud2>(       "pgo/icp/target",     rclcpp::QoS(10));
  pub_icp_align_      = node_->create_publisher<sensor_msgs::msg::PointCloud2>(       "pgo/icp/align",      rclcpp::QoS(10));


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
  std::thread thread_pose_graph(  std::bind(&PGO::func_pose_graph,   this));
  std::thread thread_loop_closure(std::bind(&PGO::func_loop_closure, this));

  /* spin */
  rclcpp::spin(node_);

}


/* --------------------------------------------------------------------------------------------- */


void PGO::callback_mf_sync_odom_cloud(const nav_msgs::msg::Odometry::SharedPtr msg_odom, const sensor_msgs::msg::PointCloud2::SharedPtr msg_cloud){

  /* update: subscription buffer */
  mtx_sub_.lock();
  data_.buf_odom.push(msg_odom);
  data_.buf_cloud.push(msg_cloud);
  mtx_sub_.unlock();

}


/* --------------------------------------------------------------------------------------------- */


void PGO::func_pose_graph(void){

  /* infinite loop */
  while (true){

    /* check: subscription buffer */
    while ((!data_.buf_odom.empty()) && (!data_.buf_cloud.empty())){

      /* sample */
      mtx_sub_.lock();
      PGOPose cur_pose = odom_to_pgo_pose(data_.buf_odom.front());
      pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*data_.buf_cloud.front(), *cur_cloud);
      data_.buf_odom.pop();
      data_.buf_cloud.pop();
      mtx_sub_.unlock();


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
      data_.kf_poses.push_back(cur_pose);
      data_.kf_poses_opt.push_back(cur_pose);
      data_.kf_clouds.push_back(cur_cloud_ds);
      data_.kf_size = data_.kf_clouds.size();
      mtx_kf_.unlock();


      /* update: pose graph */
      // HERE
      // ...
      // ...


      /* update: keyframe positions (no mutex needed) */
      data_.kf_positions->push_back(pgo_pose_to_pcl_point(cur_pose));


      /* find loop candidate */
      bool   lc_found       = false;
      int    lc_prv_kf_ind  = -1;
      int    lc_cur_kf_ind  = data_.kf_positions->size() - 1;
      double lc_time_diff_s = -1;
      {

        /* KDTree radius search */
        std::vector<int>   lc_inds;
        std::vector<float> lc_sq_dists;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(data_.kf_positions);
        kdtree.radiusSearch(data_.kf_positions->back(), param_.lc_max_radius_m, lc_inds, lc_sq_dists);

        /* check: candidates */
        mtx_kf_.lock();
        for (int i : lc_inds){

          /* time difference */
          lc_time_diff_s = std::abs(data_.kf_poses_opt[lc_cur_kf_ind].t - data_.kf_poses_opt[i].t);

          /* check: criteria */
          if ((i != lc_cur_kf_ind) && (lc_time_diff_s > param_.lc_min_time_diff_s)){
            lc_found      = true;
            lc_prv_kf_ind = i;
            std::printf("[INFO] loop candidate found, %4d - %4d, time diff: %7.2f [s]\n", lc_prv_kf_ind, lc_cur_kf_ind, lc_time_diff_s);
            break;
          }

        }
        mtx_kf_.unlock();

      }


      /* update: loop candidate buffer */
      if (lc_found){

        /* push */
        mtx_lc_.lock();
        data_.buf_loop_candidate.push(std::pair<int, int>(lc_prv_kf_ind, lc_cur_kf_ind));
        mtx_lc_.unlock();

      }


      // HERE: temporal vis code
      {
        auto point = pgo_pose_to_msg_point(cur_pose);
        auto stamp = lib::ros2::get_stamp();

        data_.vis_graph_nodes.points.push_back(point);
        data_.vis_graph_nodes.header.stamp = stamp;

        data_.vis_graph_edges.points.push_back(point);
        data_.vis_graph_edges.header.stamp = stamp;

        // HERE: TEMPORAL VIS CODE
        if (lc_found){
          data_.vis_graph_loops_pass.points.push_back(pgo_pose_to_msg_point(data_.kf_poses_opt[lc_prv_kf_ind]));
          data_.vis_graph_loops_pass.points.push_back(point);
          data_.vis_graph_loops_pass.header.stamp = stamp;
        }

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


void PGO::func_loop_closure(void){

  /* infinite loop */
  while (true){

    /* check: loop candidate buffer */
    while (!data_.buf_loop_candidate.empty()){

      /* sample */
      mtx_lc_.lock();
      int lc_prv_kf_ind = data_.buf_loop_candidate.front().first;
      int lc_cur_kf_ind = data_.buf_loop_candidate.front().second;
      data_.buf_loop_candidate.pop();
      mtx_lc_.unlock();





      // HERE: ICP TEMP CODE (NEED TO CHECK AGAIN!)
      pcl::PointCloud<pcl::PointXYZI>::Ptr icp_source(new pcl::PointCloud<pcl::PointXYZI>()); // current
      pcl::PointCloud<pcl::PointXYZI>::Ptr icp_target(new pcl::PointCloud<pcl::PointXYZI>()); // previous (down-sampled)

      mtx_kf_.lock();
      pcl::transformPointCloud(*data_.kf_clouds[lc_cur_kf_ind], *icp_source, pgo_pose_to_tf(data_.kf_poses_opt[lc_cur_kf_ind]));
      mtx_kf_.unlock();

      mtx_kf_.lock();
      int near_size = 20;
      pcl::PointCloud<pcl::PointXYZI>::Ptr prv_kf_clouds(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr prv_kf_cloud( new pcl::PointCloud<pcl::PointXYZI>());
      for (int i = lc_prv_kf_ind - near_size; i < (lc_prv_kf_ind + near_size + 1); i++){
        if ((i < 0) || (i >= data_.kf_size)) { continue; }
        pcl::transformPointCloud(*data_.kf_clouds[i], *prv_kf_cloud, pgo_pose_to_tf(data_.kf_poses_opt[i]));
        *prv_kf_clouds += *prv_kf_cloud;
      }
      mtx_kf_.unlock();

      if (prv_kf_clouds->empty()) { break; }
      data_.voxel_grid_icp.setInputCloud(prv_kf_clouds);
      data_.voxel_grid_icp.filter(*icp_target);

      sensor_msgs::msg::PointCloud2 msg_icp_source;
      sensor_msgs::msg::PointCloud2 msg_icp_target;
      pcl::toROSMsg(*icp_source, msg_icp_source);
      pcl::toROSMsg(*icp_target, msg_icp_target);
      msg_icp_source.header.frame_id = param_.frame_id_slam_frame;
      msg_icp_target.header.frame_id = param_.frame_id_slam_frame;
      msg_icp_source.header.stamp    = lib::ros2::get_stamp();
      msg_icp_target.header.stamp    = lib::ros2::get_stamp();
      pub_icp_source_->publish(msg_icp_source);
      pub_icp_target_->publish(msg_icp_target);





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


pcl::PointXYZ PGO::pgo_pose_to_pcl_point(const PGOPose& pose){
  pcl::PointXYZ point(pose.px, pose.py, pose.pz);
  return point;
}


Eigen::Matrix4d PGO::pgo_pose_to_tf(const PGOPose& pose){
  return lib::conversion::get_transformation_matrix(pose.px, pose.py, pose.pz, pose.qw, pose.qx, pose.qy, pose.qz);
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
