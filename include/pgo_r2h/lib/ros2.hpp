#ifndef _PGO_R2H_LIB_ROS2_HPP_
#define _PGO_R2H_LIB_ROS2_HPP_


#include <cstdlib>
#include <string>
#include <iostream>

#include <Eigen/Eigen>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "pgo_r2h/lib/time.hpp"
#include "pgo_r2h/lib/conversion.hpp"


namespace lib{
namespace ros2{


/* --------------------------------------------------------------------------------------------- */


template<typename ParameterT> // NOTE: func(in, in, out, in)
void get_parameter_or_exit(const rclcpp::Node::SharedPtr& node, const std::string& name, ParameterT& parameter, const bool& print){
  if (!node->get_parameter(name, parameter)){
    std::printf("[ERROR] get_parameter_or_exit(), cannot get %s, exit!\n", name.c_str());
    std::exit(EXIT_FAILURE);
  }
  if (print) { std::cout << "[INFO] get_parameter_or_exit(), " << name << ": " << parameter << std::endl; }
}


template<typename ParameterT> // NOTE: func(in, in, in, out, in)
void declare_get_param_or_exit(const rclcpp::Node::SharedPtr& node,
                               const std::string&             name,
                               const rclcpp::ParameterType&   type,
                                     ParameterT&              parameter,
                               const bool&                    print){
  node->declare_parameter(name, type);
  get_parameter_or_exit(node, name, parameter, print);
}


/* --------------------------------------------------------------------------------------------- */


builtin_interfaces::msg::Time get_stamp(void);


/* --------------------------------------------------------------------------------------------- */


geometry_msgs::msg::PoseStamped get_msg_pose_stamped_from_tf(const Eigen::Matrix4d& transformation_matrix, const std::string& frame_id);


/* --------------------------------------------------------------------------------------------- */


geometry_msgs::msg::TransformStamped get_msg_transform_stamped_from_tf(const Eigen::Matrix4d&   transformation_matrix,
                                                                       const std::string&       frame_id,
                                                                       const std::string& child_frame_id);


/* --------------------------------------------------------------------------------------------- */


}
}


#endif
