#ifndef _PGO_R2H_LIB_CONVERSION_HPP_
#define _PGO_R2H_LIB_CONVERSION_HPP_


#include <iostream>

#include <Eigen/Eigen>


namespace lib{
namespace conversion{


/* --------------------------------------------------------------------------------------------- */


// NOTE: Tait-Bryan Angles, Z(heading)-Y(pitch)-X(bank) Rotation
// NOTE: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_(in_3-2-1_sequence)_to_quaternion_conversion
void euler_angle_to_quaternion(
  const double& er_rad,
  const double& ep_rad,
  const double& ey_rad,
  double& qw,
  double& qx,
  double& qy,
  double& qz
);


// NOTE: Tait-Bryan Angles, Z(heading)-Y(pitch)-X(bank) Rotation
// NOTE: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_(in_3-2-1_sequence)_conversion
void quaternion_to_euler_angle(
  const double& qw,
  const double& qx,
  const double& qy,
  const double& qz,
  double& er_rad,
  double& ep_rad,
  double& ey_rad
);


void quaternion_to_euler_angle(const Eigen::Vector4d& quaternion_wxyz, Eigen::Vector3d& euler_angle_rpy_rad);


/* --------------------------------------------------------------------------------------------- */


void quaternion_to_rotation_matrix(
  const double& qw,
  const double& qx,
  const double& qy,
  const double& qz,
  Eigen::Matrix3d& rot_mat
);


void rotation_matrix_to_quaternion(
  const Eigen::Matrix3d& rot_mat,
  double& qw,
  double& qx,
  double& qy,
  double& qz
);


/* --------------------------------------------------------------------------------------------- */


void rotation_matrix_to_euler_angle(
  const Eigen::Matrix3d& rot_mat,
  double& er_rad,
  double& ep_rad,
  double& ey_rad
);


void euler_angle_to_rotation_matrix(
  const double& er_rad,
  const double& ep_rad,
  const double& ey_rad,
  Eigen::Matrix3d& rot_mat
);


/* --------------------------------------------------------------------------------------------- */


Eigen::Matrix4d get_transformation_matrix(
  const double& tx, const double& ty, const double& tz,                  // NOTE: translation x, y, z
  const double& qw, const double& qx, const double& qy, const double& qz // NOTE:  quaternion w, x, y, z
);


Eigen::Matrix4d get_transformation_matrix(
  const double& tx,     const double& ty,     const double& tz,    // NOTE:  translation x, y, z
  const double& er_rad, const double& ep_rad, const double& ey_rad // NOTE: euler angles roll, pitch, yaw in [rad] (use ZYX rotation)
);


Eigen::Matrix4d get_inverse_transformation_matrix(const Eigen::Matrix4d& transformation_matrix);


/* --------------------------------------------------------------------------------------------- */


void transformation_matrix_to_pose(
  const Eigen::Matrix4d& transformation_matrix,
  double& px, double& py, double& pz,
  double& qw, double& qx, double& qy, double& qz
);


void transformation_matrix_to_pose(
  const Eigen::Matrix4d& transformation_matrix,
  double& px,     double& py,     double& pz,
  double& er_rad, double& ep_rad, double& ey_rad
);


/* --------------------------------------------------------------------------------------------- */


Eigen::Vector3d get_pos_from_transformation_matrix(const Eigen::Matrix4d& transformation_matrix);


Eigen::Matrix3d get_rot_mat_from_transformation_matrix(const Eigen::Matrix4d& transformation_matrix);


// NOTE: it returns quaternion: (qw, qx, qy, qz)
Eigen::Vector4d get_quaternion_from_transformation_matrix(const Eigen::Matrix4d& transformation_matrix);


// NOTE: it returns euler angle: (roll_rad, pitch_rad, yaw_rad)
Eigen::Vector3d get_euler_angle_from_transformation_matrix(const Eigen::Matrix4d& transformation_matrix);


/* --------------------------------------------------------------------------------------------- */


// NOTE: it returns yaw angle in [rad] (use ZYX rotation)
double get_yaw_angle_from_transformation_matrix(const Eigen::Matrix4d& transformation_matrix);


/* --------------------------------------------------------------------------------------------- */


// TODO: [AL2-LATER] integrity check!
void transform_twist(
  const Eigen::Matrix4d& transformation_matrix, // NOTE: transformation matrix (from ➤ to)
  const Eigen::Vector3d& from_linear,           // NOTE:  linear velocity in [m/s]
  const Eigen::Vector3d& from_angular,          // NOTE: angular velocity in [rad/s]
        Eigen::Vector3d&   to_linear,           // NOTE:  linear velocity in [m/s]
        Eigen::Vector3d&   to_angular           // NOTE: angular velocity in [rad/s]
);


/* --------------------------------------------------------------------------------------------- */


}
}


#endif
