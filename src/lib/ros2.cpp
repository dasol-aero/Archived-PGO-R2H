#include "pgo_r2h/lib/ros2.hpp"


namespace lib{
namespace ros2{


builtin_interfaces::msg::Time get_stamp(void){

  /* declaration */
  builtin_interfaces::msg::Time stamp;

  /* time since epoch in [ns] */
  int64_t t = time::get_time_since_epoch_ns_int64();

  /* update */
  int64_t sec     = t / 1000000000;
  int64_t nanosec = t - sec * 1000000000;
  stamp.sec       = sec;
  stamp.nanosec   = nanosec;

  /* return */
  return stamp;

}


geometry_msgs::msg::PoseStamped get_msg_pose_stamped_from_tf(const Eigen::Matrix4d& transformation_matrix, const std::string& frame_id){

  /* declaration */
  double px, py, pz;
  double qw, qx, qy, qz;
  geometry_msgs::msg::PoseStamped msg;

  /* tf to pose */
  conversion::transformation_matrix_to_pose(transformation_matrix, px, py, pz, qw, qx, qy, qz);

  /* update */
  msg.header.frame_id    = frame_id;
  msg.header.stamp       = get_stamp();
  msg.pose.position.x    = px;
  msg.pose.position.y    = py;
  msg.pose.position.z    = pz;
  msg.pose.orientation.w = qw;
  msg.pose.orientation.x = qx;
  msg.pose.orientation.y = qy;
  msg.pose.orientation.z = qz;

  /* return */
  return msg;

}


geometry_msgs::msg::TransformStamped get_msg_transform_stamped_from_tf(const Eigen::Matrix4d&   transformation_matrix,
                                                                       const std::string&       frame_id,
                                                                       const std::string& child_frame_id){

  /* declaration */
  double tx, ty, tz;
  double qw, qx, qy, qz;
  geometry_msgs::msg::TransformStamped msg;

  /* translation and rotation */
  conversion::transformation_matrix_to_pose(transformation_matrix, tx, ty, tz, qw, qx, qy, qz);

  /* update */
  msg.header.stamp            = get_stamp();
  msg.header.frame_id         = frame_id;
  msg.child_frame_id          = child_frame_id;
  msg.transform.translation.x = tx;
  msg.transform.translation.y = ty;
  msg.transform.translation.z = tz;
  msg.transform.rotation.w    = qw;
  msg.transform.rotation.x    = qx;
  msg.transform.rotation.y    = qy;
  msg.transform.rotation.z    = qz;

  /* return */
  return msg;

}


}
}
