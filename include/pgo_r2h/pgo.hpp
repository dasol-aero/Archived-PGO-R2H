#ifndef _PGO_R2H_PGO_HPP_
#define _PGO_R2H_PGO_HPP_


#define _USE_MATH_DEFINES


#include <cmath>
#include <cstdint>
#include <queue>
#include <deque>
#include <mutex>
#include <thread>
#include <chrono>
#include <string>
#include <iostream>

#include <Eigen/Eigen>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "rclcpp/rclcpp.hpp"

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


struct ParamPGO{

};


/* --------------------------------------------------------------------------------------------- */


struct DataPGO{

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
  ParamPGO param_;


  /* data */
  DataPGO data_;


  /* mutex */
  // ...


  /* publication */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_cloud_keyframe_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_graph_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_icp_source_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_icp_target_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_icp_align_;


  /* subscription */

};


/* --------------------------------------------------------------------------------------------- */


#endif
