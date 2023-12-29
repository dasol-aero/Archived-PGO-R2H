#include "pgo_r2h/pgo.hpp"


PGO::PGO(const rclcpp::Node::SharedPtr& node) : node_(node) {

  /* print */
  std::printf("[INFO] class PGO has been created\n");

}


void PGO::init(void){

  /* parameter: declare and get */
  lib::ros2::declare_get_param_or_exit(node_, "frame_id_slam_frame",       rclcpp::PARAMETER_STRING,  param_.frame_id_slam_frame,       true);
  lib::ros2::declare_get_param_or_exit(node_, "topic_sub_odom",            rclcpp::PARAMETER_STRING,  param_.topic_sub_odom,            true);
  lib::ros2::declare_get_param_or_exit(node_, "topic_sub_cloud",           rclcpp::PARAMETER_STRING,  param_.topic_sub_cloud,           true);
  lib::ros2::declare_get_param_or_exit(node_, "keyframe_gap_meter",        rclcpp::PARAMETER_DOUBLE,  param_.keyframe_gap_meter,        true);
  lib::ros2::declare_get_param_or_exit(node_, "keyframe_gap_deg",          rclcpp::PARAMETER_DOUBLE,  param_.keyframe_gap_deg,          true);
  lib::ros2::declare_get_param_or_exit(node_, "leaf_size_keyframe",        rclcpp::PARAMETER_DOUBLE,  param_.leaf_size_keyframe,        true);
  lib::ros2::declare_get_param_or_exit(node_, "leaf_size_icp",             rclcpp::PARAMETER_DOUBLE,  param_.leaf_size_icp,             true);
  lib::ros2::declare_get_param_or_exit(node_, "leaf_size_out_map",         rclcpp::PARAMETER_DOUBLE,  param_.leaf_size_out_map,         true);
  lib::ros2::declare_get_param_or_exit(node_, "lc_max_radius_m",           rclcpp::PARAMETER_DOUBLE,  param_.lc_max_radius_m,           true);
  lib::ros2::declare_get_param_or_exit(node_, "lc_min_time_diff_s",        rclcpp::PARAMETER_DOUBLE,  param_.lc_min_time_diff_s,        true);
  lib::ros2::declare_get_param_or_exit(node_, "icp_type",                  rclcpp::PARAMETER_INTEGER, param_.icp_type,                  true);
  lib::ros2::declare_get_param_or_exit(node_, "icp_stack_size",            rclcpp::PARAMETER_INTEGER, param_.icp_stack_size,            true);
  lib::ros2::declare_get_param_or_exit(node_, "icp_config_max_iter",       rclcpp::PARAMETER_INTEGER, param_.icp_config_max_iter,       true);
  lib::ros2::declare_get_param_or_exit(node_, "icp_config_max_cor_dist",   rclcpp::PARAMETER_DOUBLE,  param_.icp_config_max_cor_dist,   true);
  lib::ros2::declare_get_param_or_exit(node_, "icp_test_max_fitness",      rclcpp::PARAMETER_DOUBLE,  param_.icp_test_max_fitness,      true);
  lib::ros2::declare_get_param_or_exit(node_, "var_prior",                 rclcpp::PARAMETER_DOUBLE,  param_.var_prior,                 true);
  lib::ros2::declare_get_param_or_exit(node_, "var_odom",                  rclcpp::PARAMETER_DOUBLE,  param_.var_odom,                  true);
  lib::ros2::declare_get_param_or_exit(node_, "var_loop",                  rclcpp::PARAMETER_DOUBLE,  param_.var_loop,                  true);
  lib::ros2::declare_get_param_or_exit(node_, "num_update",                rclcpp::PARAMETER_INTEGER, param_.num_update,                true);
  lib::ros2::declare_get_param_or_exit(node_, "enable_out_map",            rclcpp::PARAMETER_BOOL,    param_.enable_out_map,            true);
  lib::ros2::declare_get_param_or_exit(node_, "enable_pub_keyframe_cloud", rclcpp::PARAMETER_BOOL,    param_.enable_pub_keyframe_cloud, true);
  lib::ros2::declare_get_param_or_exit(node_, "enable_pub_graph",          rclcpp::PARAMETER_BOOL,    param_.enable_pub_graph,          true);
  lib::ros2::declare_get_param_or_exit(node_, "nodes_scale",               rclcpp::PARAMETER_DOUBLE,  param_.nodes_scale,               true);
  lib::ros2::declare_get_param_or_exit(node_, "nodes_alpha",               rclcpp::PARAMETER_DOUBLE,  param_.nodes_alpha,               true);
  lib::ros2::declare_get_param_or_exit(node_, "edges_scale",               rclcpp::PARAMETER_DOUBLE,  param_.edges_scale,               true);
  lib::ros2::declare_get_param_or_exit(node_, "edges_alpha",               rclcpp::PARAMETER_DOUBLE,  param_.edges_alpha,               true);
  lib::ros2::declare_get_param_or_exit(node_, "loops_pass_scale",          rclcpp::PARAMETER_DOUBLE,  param_.loops_pass_scale,          true);
  lib::ros2::declare_get_param_or_exit(node_, "loops_pass_alpha",          rclcpp::PARAMETER_DOUBLE,  param_.loops_pass_alpha,          true);
  lib::ros2::declare_get_param_or_exit(node_, "loops_fail_scale",          rclcpp::PARAMETER_DOUBLE,  param_.loops_fail_scale,          true);
  lib::ros2::declare_get_param_or_exit(node_, "loops_fail_alpha",          rclcpp::PARAMETER_DOUBLE,  param_.loops_fail_alpha,          true);
  lib::ros2::declare_get_param_or_exit(node_, "enable_pub_icp",            rclcpp::PARAMETER_BOOL,    param_.enable_pub_icp,            true);


  /* parameter: derived */
  param_.keyframe_gap_rad = param_.keyframe_gap_deg / 180.0 * M_PI;


  /* data */
  data_.last_kf_pose.valid = false;

  data_.voxel_grid_kf.setLeafSize(     param_.leaf_size_keyframe, param_.leaf_size_keyframe, param_.leaf_size_keyframe);
  data_.voxel_grid_icp.setLeafSize(    param_.leaf_size_icp,      param_.leaf_size_icp,      param_.leaf_size_icp);
  data_.voxel_grid_out_map.setLeafSize(param_.leaf_size_out_map,  param_.leaf_size_out_map,  param_.leaf_size_out_map);

  data_.kf_positions.reset(new pcl::PointCloud<pcl::PointXYZ>());

  {

    /* ISAM2 */
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip      = 1;
    data_.isam2.reset(new gtsam::ISAM2(isam2_params));

    /* GTSAM factor noise model */
    gtsam::Vector6 var_prior(param_.var_prior, param_.var_prior, param_.var_prior, param_.var_prior, param_.var_prior, param_.var_prior);
    gtsam::Vector6 var_odom( param_.var_odom,  param_.var_odom,  param_.var_odom,  param_.var_odom,  param_.var_odom,  param_.var_odom);
    gtsam::Vector6 var_loop( param_.var_loop,  param_.var_loop,  param_.var_loop,  param_.var_loop,  param_.var_loop,  param_.var_loop);
    data_.noise_prior = gtsam::noiseModel::Diagonal::Variances(var_prior);
    data_.noise_odom  = gtsam::noiseModel::Diagonal::Variances(var_odom);
    data_.noise_loop  = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(1), gtsam::noiseModel::Diagonal::Variances(var_loop));

  }

  init_vis_graph_all();


  /* publication */
  pub_keyframe_cloud_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(       "pgo/keyframe/cloud", rclcpp::QoS(10));
  pub_graph_          = node_->create_publisher<visualization_msgs::msg::MarkerArray>("pgo/graph",          rclcpp::QoS(10));
  pub_icp_source_     = node_->create_publisher<sensor_msgs::msg::PointCloud2>(       "pgo/icp/source",     rclcpp::QoS(10));
  pub_icp_target_     = node_->create_publisher<sensor_msgs::msg::PointCloud2>(       "pgo/icp/target",     rclcpp::QoS(10));
  pub_icp_aligned_    = node_->create_publisher<sensor_msgs::msg::PointCloud2>(       "pgo/icp/aligned",    rclcpp::QoS(10));
  pub_out_map_        = node_->create_publisher<sensor_msgs::msg::PointCloud2>(       "pgo/out/map",        rclcpp::QoS(10));
  pub_test_a          = node_->create_publisher<sensor_msgs::msg::PointCloud2>(       "pgo/test/a",         rclcpp::QoS(10));


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
  std::thread thread_pose_graph(    std::bind(&PGO::func_pose_graph,     this));
  std::thread thread_loop_closure(  std::bind(&PGO::func_loop_closure,   this));
  std::thread thread_pose_graph_opt(std::bind(&PGO::func_pose_graph_opt, this));

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


      /* update: keyframe data */
      mtx_kf_.lock();
      data_.kf_poses.push_back(cur_pose);
      data_.kf_poses_opt.push_back(cur_pose);
      data_.kf_clouds.push_back(cur_cloud_ds);
      data_.kf_size = data_.kf_clouds.size();
      data_.kf_positions->push_back(pgo_pose_to_pcl_point(cur_pose));
      mtx_kf_.unlock();


      /* publish: keyframe cloud */
      if (param_.enable_pub_keyframe_cloud){

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
        pub_keyframe_cloud_->publish(msg_ros);

      }


      /* update: pose graph */
      if (data_.has_graph){

        /* pose from and to */
        mtx_kf_.lock();
        int64_t       ind_from = data_.kf_size - 2;
        int64_t       ind_to   = data_.kf_size - 1;
        gtsam::Pose3 pose_from = pgo_pose_to_gtsam_pose3(data_.kf_poses[ind_from]);
        gtsam::Pose3 pose_to   = pgo_pose_to_gtsam_pose3(data_.kf_poses[ind_to]);
        mtx_kf_.unlock();

        /* odometry factor */
        mtx_graph_.lock();
        data_.graph.add(gtsam::BetweenFactor<gtsam::Pose3>(ind_from, ind_to, pose_from.between(pose_to), data_.noise_odom));
        data_.init_estimate.insert(ind_to, pose_to);
        mtx_graph_.unlock();

        /* update: flag */
        mtx_bools_.lock();
        data_.run_pose_graph_opt = true;
        mtx_bools_.unlock();

      } else {

        /* prior pose */
        gtsam::Pose3 prior_pose = pgo_pose_to_gtsam_pose3(cur_pose);

        /* prior factor */
        mtx_graph_.lock();
        data_.graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, prior_pose, data_.noise_prior));
        data_.init_estimate.insert(0, prior_pose);
        mtx_graph_.unlock();

        /* update flag */
        data_.has_graph = true;

      }


      /* find loop candidate */
      mtx_kf_.lock();
      bool   lc_found      = false;
      int    lc_prv_kf_ind;
      int    lc_cur_kf_ind = data_.kf_size - 1;
      double lc_time_diff_s;
      {

        /* KDTree radius search */
        std::vector<int>   lc_inds;
        std::vector<float> lc_sq_dists;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(data_.kf_positions);
        kdtree.radiusSearch(data_.kf_positions->back(), param_.lc_max_radius_m, lc_inds, lc_sq_dists);

        /* check: candidates */
        double lc_time_diff_max_s = -1e9;
        for (int i : lc_inds){

          /* time difference */
          lc_time_diff_s = data_.kf_poses_opt[lc_cur_kf_ind].t - data_.kf_poses_opt[i].t;

          /* check: criteria */
          if ((i != lc_cur_kf_ind) &&
              (lc_time_diff_s > param_.lc_min_time_diff_s) &&
              (lc_time_diff_s > lc_time_diff_max_s)){
            lc_found           = true;
            lc_prv_kf_ind      = i;
            lc_time_diff_max_s = lc_time_diff_s;
          }

        }

        /* print (if found) */
        if (lc_found) { std::printf("\n[INFO-LC] loop candidate found (%d-%d)    time diff [s]: %.2f\n", lc_prv_kf_ind, lc_cur_kf_ind, lc_time_diff_s); }

      }
      mtx_kf_.unlock();


      /* update: loop candidate buffer */
      if (lc_found){

        /* push */
        mtx_lc_.lock();
        data_.buf_loop_candidate.push(std::pair<int, int>(lc_prv_kf_ind, lc_cur_kf_ind));
        mtx_lc_.unlock();

      }

    }


    /* sleep */
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

  }

} // func_pose_graph


/* --------------------------------------------------------------------------------------------- */


void PGO::func_loop_closure(void){

  /* infinite loop */
  while (true){

    /* check: loop candidate buffer */
    while (!data_.buf_loop_candidate.empty()){

      /* get loop candidate */
      mtx_lc_.lock();
      std::pair<int, int> loop_candidate = data_.buf_loop_candidate.front();
      data_.buf_loop_candidate.pop();
      mtx_lc_.unlock();

      /* is it loop ? */
      gtsam::Pose3 pose_from_cur_to_prv;
      if (is_loop(loop_candidate, pose_from_cur_to_prv)){

        /* update: loops pass */
        mtx_lc_.lock();
        data_.loops_pass.push_back(loop_candidate.first);
        data_.loops_pass.push_back(loop_candidate.second);
        mtx_lc_.unlock();

        /* loop factor */
        int loop_prv_ind = loop_candidate.first;
        int loop_cur_ind = loop_candidate.second;
        mtx_graph_.lock();
        data_.graph.add(gtsam::BetweenFactor<gtsam::Pose3>(loop_cur_ind, loop_prv_ind, pose_from_cur_to_prv, data_.noise_loop));
        mtx_graph_.unlock();

        /* update: flag */
        mtx_bools_.lock();
        data_.run_pose_graph_opt = true;
        mtx_bools_.unlock();

      } else {

        /* update: loops fail */
        mtx_lc_.lock();
        data_.loops_fail.push_back(loop_candidate.first);
        data_.loops_fail.push_back(loop_candidate.second);
        mtx_lc_.unlock();

      }

    }

    /* sleep */
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

  }

} // func_loop_closure


/* --------------------------------------------------------------------------------------------- */


void PGO::func_pose_graph_opt(void){

  /* infinite loop */
  while (true){

    /* check: flag */
    if (data_.run_pose_graph_opt){

      /* pose graph optimization */
      pose_graph_opt();

      /* clear: flag */
      mtx_bools_.lock();
      data_.run_pose_graph_opt = false;
      mtx_bools_.unlock();

      foo(); // FIX: FOO

    }

    /* sleep */
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

  }

} // func_pose_graph_opt


/* --------------------------------------------------------------------------------------------- */


double PGO::stamp_to_second(const builtin_interfaces::msg::Time& stamp){
  double x = double(stamp.sec) + double(stamp.nanosec) / 1e9;
  return x;
}


builtin_interfaces::msg::Time PGO::second_to_stamp(const double second){
  builtin_interfaces::msg::Time x;
  x.sec     =  int32_t(std::floor(second));
  x.nanosec = uint32_t((second - std::floor(second)) * 1e9);
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


gtsam::Pose3 PGO::pgo_pose_to_gtsam_pose3(const PGOPose& pose){
  return gtsam::Pose3(gtsam::Rot3::Quaternion(pose.qw, pose.qx, pose.qy, pose.qz), gtsam::Point3(pose.px, pose.py, pose.pz));
}


Eigen::Matrix4d PGO::pgo_pose_to_tf(const PGOPose& pose){
  return lib::conversion::get_transformation_matrix(pose.px, pose.py, pose.pz, pose.qw, pose.qx, pose.qy, pose.qz);
}


gtsam::Pose3 PGO::tf_to_gtsam_pose3(const Eigen::Matrix4d& tf){
  return gtsam::Pose3(gtsam::Rot3(tf.block(0, 0, 3, 3)), gtsam::Point3(tf.block(0, 3, 3, 1)));
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


bool PGO::is_loop(const std::pair<int, int>& loop_candidate, gtsam::Pose3& pose_from_cur_to_prv){

  /* declaration */
  int  lc_prv_kf_ind = loop_candidate.first;  // ICP target
  int  lc_cur_kf_ind = loop_candidate.second; // ICP source
  pcl::PointCloud<pcl::PointXYZI>::Ptr icp_source( new pcl::PointCloud<pcl::PointXYZI>()); //  current keyframe
  pcl::PointCloud<pcl::PointXYZI>::Ptr icp_target( new pcl::PointCloud<pcl::PointXYZI>()); // previous keyframes (stacked and down-sampled)
  pcl::PointCloud<pcl::PointXYZI>::Ptr icp_aligned(new pcl::PointCloud<pcl::PointXYZI>());


  /* ICP source */
  mtx_kf_.lock(); // NOTE: mutex keyframe LOCK
  PGOPose prv_pgo_pose = data_.kf_poses_opt[lc_prv_kf_ind];
  PGOPose cur_pgo_pose = data_.kf_poses_opt[lc_cur_kf_ind];
  pcl::transformPointCloud(*data_.kf_clouds[lc_cur_kf_ind], *icp_source, pgo_pose_to_tf(cur_pgo_pose));


  /* ICP target*/
  pcl::PointCloud<pcl::PointXYZI>::Ptr stacked_clouds(new pcl::PointCloud<pcl::PointXYZI>()); // ICP target (stacked and not down-sampled)
  int is = std::max(lc_prv_kf_ind - param_.icp_stack_size,     0);                  // index start (inclusive)
  int ie = std::min(lc_prv_kf_ind + param_.icp_stack_size + 1, int(data_.kf_size)); // index   end (exclusive)
  for (int i = is; i < ie; i++){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_to_stack(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*data_.kf_clouds[i], *cloud_to_stack, pgo_pose_to_tf(data_.kf_poses_opt[i]));
    *stacked_clouds += *cloud_to_stack;
  }
  mtx_kf_.unlock(); // NOTE: mutex keyframe UNLOCK
  data_.voxel_grid_icp.setInputCloud(stacked_clouds);
  data_.voxel_grid_icp.filter(*icp_target);


  /* ICP test */
  int64_t         icp_ts = lib::time::get_time_since_epoch_ns_int64();
  std::string     icp_type;
  bool            icp_converged;
  double          icp_fitness;
  bool            icp_test_pass;
  Eigen::Matrix4d icp_tf_src2tg;
  double          icp_elapsed_ms;
  if (param_.icp_type == 0){

    /* use ICP */
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setMaximumIterations(param_.icp_config_max_iter);
    icp.setMaxCorrespondenceDistance(param_.icp_config_max_cor_dist);
    icp.setTransformationEpsilon(1e-6); // 1 mm * 1 mm
    icp.setInputSource(icp_source);
    icp.setInputTarget(icp_target);
    icp.align(*icp_aligned);

    /* results */
    icp_type       = "ICP";
    icp_converged  = icp.hasConverged();
    icp_fitness    = icp.getFitnessScore();
    icp_test_pass  = (icp_fitness < param_.icp_test_max_fitness);
    icp_tf_src2tg  = icp.getFinalTransformation().cast<double>();
    icp_elapsed_ms = double(lib::time::get_time_since_epoch_ns_int64() - icp_ts) / 1e6;

  } else if (param_.icp_type == 1){

    /* use G-ICP */
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    gicp.setMaximumIterations(param_.icp_config_max_iter);
    gicp.setMaxCorrespondenceDistance(param_.icp_config_max_cor_dist);
    gicp.setTransformationEpsilon(1e-6); // 1 mm * 1 mm
    gicp.setInputSource(icp_source);
    gicp.setInputTarget(icp_target);
    gicp.align(*icp_aligned);

    /* results */
    icp_type       = "G-ICP";
    icp_converged  = gicp.hasConverged();
    icp_fitness    = gicp.getFitnessScore();
    icp_test_pass  = (icp_fitness < param_.icp_test_max_fitness);
    icp_tf_src2tg  = gicp.getFinalTransformation().cast<double>();
    icp_elapsed_ms = double(lib::time::get_time_since_epoch_ns_int64() - icp_ts) / 1e6;

  } else {

    /* error */
    std::printf("[ERROR] PGO::is_loop(), undefined icp_type(%d) is given, exit\n", param_.icp_type);
    std::exit(EXIT_FAILURE);

  }


  /* relative pose from CURRENT (ICP SOURCE) to PREVIOUS (ICP TARGET) */
  // NOTE: icp_tf_src2tg means that  "active" transformation of source cloud to target cloud
  // NOTE: icp_tf_src2tg means that "passive" transformation of target frame to source frame ---> interpreted as this way
  // NOTE: icp_tf_src2tg means that correction for current pose
  gtsam::Pose3 cur_gtsam_pose = tf_to_gtsam_pose3(icp_tf_src2tg * pgo_pose_to_tf(cur_pgo_pose));
  gtsam::Pose3 prv_gtsam_pose = pgo_pose_to_gtsam_pose3(prv_pgo_pose);


  /* print ICP test results */
  {

    /* declaration */
    double tx, ty, tz;             //  translation (each axis)
    double er_rad, ep_rad, ey_rad; // euler angles (each axis) [rad]
    double er_deg, ep_deg, ey_deg; // euler angles (each axis) [deg]
    double trs;                    //  translation (magnitude)
    double rot_deg;                //     rotation (magnitude)

    /* tf decomposition (each axis) */
    lib::conversion::transformation_matrix_to_pose(icp_tf_src2tg, tx, ty, tz, er_rad, ep_rad, ey_rad);
    er_deg = er_rad / M_PI * 180.0;
    ep_deg = ep_rad / M_PI * 180.0;
    ey_deg = ey_rad / M_PI * 180.0;

    /* tf decomposition (magnitude) */
    Eigen::AngleAxisd rot_angle_axis(lib::conversion::get_rot_mat_from_transformation_matrix(icp_tf_src2tg));
    trs     = std::sqrt(tx * tx + ty * ty + tz * tz);
    rot_deg = rot_angle_axis.angle() / M_PI * 180.0;

    /* print */
    std::printf("\n");
    std::printf("[INFO-ICP] %s converged: %s    fitness: %.4f    test pass: %s    elapsed [ms]: %.2f\n",
      icp_type.c_str(), icp_converged ? "T" : "F", icp_fitness, icp_test_pass ? "T" : "F", icp_elapsed_ms
    );
    std::printf("[INFO-ICP] TF source to target:  %.4f  %.4f  %.4f [m]    %.2f  %.2f  %.2f [deg]\n",
      tx, ty, tz, er_deg, ep_deg, ey_deg
    );
    std::printf("[INFO-ICP] TF source to target:  %.4f [m]    %.2f [deg]\n", trs, rot_deg);

  }


  /* visualization */
  if (param_.enable_pub_icp){

    /* stamp */
    builtin_interfaces::msg::Time msg_stamp = lib::ros2::get_stamp();

    /* declaration */
    sensor_msgs::msg::PointCloud2 msg_icp_source;
    sensor_msgs::msg::PointCloud2 msg_icp_target;
    sensor_msgs::msg::PointCloud2 msg_icp_aligned;

    /* to ROS msg */
    pcl::toROSMsg(*icp_source,  msg_icp_source);
    pcl::toROSMsg(*icp_target,  msg_icp_target);
    pcl::toROSMsg(*icp_aligned, msg_icp_aligned);

    /* header */
    msg_icp_source.header.frame_id  = param_.frame_id_slam_frame;
    msg_icp_target.header.frame_id  = param_.frame_id_slam_frame;
    msg_icp_aligned.header.frame_id = param_.frame_id_slam_frame;
    msg_icp_source.header.stamp     = msg_stamp;
    msg_icp_target.header.stamp     = msg_stamp;
    msg_icp_aligned.header.stamp    = msg_stamp;

    /* publish */
    pub_icp_source_->publish(msg_icp_source);
    pub_icp_target_->publish(msg_icp_target);
    pub_icp_aligned_->publish(msg_icp_aligned);

  }


  /* update: output */
  pose_from_cur_to_prv = cur_gtsam_pose.between(prv_gtsam_pose);


  /* return */
  return icp_test_pass;

}


void PGO::pose_graph_opt(void){

  /* start time */
  int64_t ts = lib::time::get_time_since_epoch_ns_int64();

  /* update: ISAM2 */
  mtx_graph_.lock();
  data_.isam2->update(data_.graph, data_.init_estimate);
  for (int i = 0; i < param_.num_update; i++) { data_.isam2->update(); }
  data_.graph.resize(0);
  data_.init_estimate.clear();
  mtx_graph_.unlock();

  /* update: current estimate */
  data_.curr_estimate = data_.isam2->calculateEstimate();

  /* update: optimized pose */
  mtx_kf_.lock();
  for (int i = 0; i < int(data_.curr_estimate.size()); i++){

    auto trs = data_.curr_estimate.at<gtsam::Pose3>(i).translation();
    auto qua = data_.curr_estimate.at<gtsam::Pose3>(i).rotation().toQuaternion();

    data_.kf_poses_opt[i].px = trs.x();
    data_.kf_poses_opt[i].py = trs.y();
    data_.kf_poses_opt[i].pz = trs.z();

    data_.kf_poses_opt[i].qw = qua.w();
    data_.kf_poses_opt[i].qx = qua.x();
    data_.kf_poses_opt[i].qy = qua.y();
    data_.kf_poses_opt[i].qz = qua.z();

    data_.kf_positions->points[i].x = trs.x();
    data_.kf_positions->points[i].y = trs.y();
    data_.kf_positions->points[i].z = trs.z();

  }
  mtx_kf_.unlock();

  /* end time and print */
  double elapsed_ms = double(lib::time::get_time_since_epoch_ns_int64() - ts) / 1e6;
  std::printf("\n[INFO-OPT] elapsed time [ms]: %.2f\n", elapsed_ms);

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


void PGO::foo(void){ // FIX: FOO

  /* graph nodes and edges */
  mtx_kf_.lock();
  for (int i = 0; i < data_.kf_size; i++){
    geometry_msgs::msg::Point msg_point = pgo_pose_to_msg_point(data_.kf_poses_opt[i]);
    if (i < int(data_.vis_graph_nodes.points.size())){
      data_.vis_graph_nodes.points[i] = msg_point;
      data_.vis_graph_edges.points[i] = msg_point;
    } else {
      data_.vis_graph_nodes.points.push_back(msg_point);
      data_.vis_graph_edges.points.push_back(msg_point);
    }
  }
  mtx_kf_.unlock();

  /* graph loops fail and pass */
  mtx_lc_.lock();
  for (int i = 0; i < int(data_.loops_fail.size()); i++){
    if (i < int(data_.vis_graph_loops_fail.points.size())){
      data_.vis_graph_loops_fail.points[i] = data_.vis_graph_nodes.points[data_.loops_fail[i]];
    } else {
      data_.vis_graph_loops_fail.points.push_back(data_.vis_graph_nodes.points[data_.loops_fail[i]]);
    }
  }
  for (int i = 0; i < int(data_.loops_pass.size()); i++){
    if (i < int(data_.vis_graph_loops_pass.points.size())){
      data_.vis_graph_loops_pass.points[i] = data_.vis_graph_nodes.points[data_.loops_pass[i]];
    } else {
      data_.vis_graph_loops_pass.points.push_back(data_.vis_graph_nodes.points[data_.loops_pass[i]]);
    }
  }
  mtx_lc_.unlock();

  /* publish graph */
  visualization_msgs::msg::MarkerArray ma;
  builtin_interfaces::msg::Time msg_stamp = lib::ros2::get_stamp();
  data_.vis_graph_nodes.header.stamp      = msg_stamp;
  data_.vis_graph_edges.header.stamp      = msg_stamp;
  data_.vis_graph_loops_fail.header.stamp = msg_stamp;
  data_.vis_graph_loops_pass.header.stamp = msg_stamp;
  ma.markers.push_back(data_.vis_graph_nodes);
  ma.markers.push_back(data_.vis_graph_edges);
  ma.markers.push_back(data_.vis_graph_loops_fail);
  ma.markers.push_back(data_.vis_graph_loops_pass);
  pub_graph_->publish(ma);

  { // FIX: FOO (CLEAN UP)
    pcl::PointCloud<pcl::PointXYZI>::Ptr pgo_map(new pcl::PointCloud<pcl::PointXYZI>());
    PGOPose pose;
    Eigen::Matrix4d tf;

    mtx_kf_.lock();
    for (int i = 0; i < data_.kf_size; i++){
      pose = data_.kf_poses_opt[i];
      tf = pgo_pose_to_tf(pose);
      pcl::PointCloud<pcl::PointXYZI>::Ptr pgo_map_part(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::transformPointCloud(*data_.kf_clouds[i], *pgo_map_part, tf);
      *pgo_map += *pgo_map_part;
    }
    mtx_kf_.unlock();

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*pgo_map, msg);
    msg.header.frame_id = param_.frame_id_slam_frame;
    msg.header.stamp    = lib::ros2::get_stamp();
    pub_out_map_->publish(msg);
  }

}
