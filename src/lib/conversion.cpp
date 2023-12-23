#include "pgo_r2h/lib/conversion.hpp"


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
){

  /* helper */
  double sr = std::sin(0.5 * er_rad);
  double cr = std::cos(0.5 * er_rad);
  double sp = std::sin(0.5 * ep_rad);
  double cp = std::cos(0.5 * ep_rad);
  double sy = std::sin(0.5 * ey_rad);
  double cy = std::cos(0.5 * ey_rad);

  /* quaternion */
  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;

}


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
){

  /* roll angle [rad] */
  er_rad = std::atan2(
        2 * (qw * qx + qy * qz),
    1 - 2 * (qx * qx + qy * qy)
  );

  /* pitch angle [rad] */
  ep_rad = -0.5 * M_PI + 2 * std::atan2(
    std::sqrt(1 + 2 * (qw * qy - qx * qz)),
    std::sqrt(1 - 2 * (qw * qy - qx * qz))
  );

  /* yaw angle [rad] */
  ey_rad = std::atan2(
        2 * (qw * qz + qx * qy),
    1 - 2 * (qy * qy + qz * qz)
  );

}


void quaternion_to_euler_angle(const Eigen::Vector4d& quaternion_wxyz, Eigen::Vector3d& euler_angle_rpy_rad){

  quaternion_to_euler_angle(
    quaternion_wxyz(0), quaternion_wxyz(1), quaternion_wxyz(2), quaternion_wxyz(3),
    euler_angle_rpy_rad(0), euler_angle_rpy_rad(1), euler_angle_rpy_rad(2)
  );

}


/* --------------------------------------------------------------------------------------------- */


void quaternion_to_rotation_matrix(
  const double& qw,
  const double& qx,
  const double& qy,
  const double& qz,
  Eigen::Matrix3d& rot_mat
){

  Eigen::Quaterniond quaternion(qw, qx, qy, qz);
  rot_mat = quaternion.normalized().toRotationMatrix();

}


void rotation_matrix_to_quaternion(
  const Eigen::Matrix3d& rot_mat,
  double& qw,
  double& qx,
  double& qy,
  double& qz
){

  Eigen::Quaterniond quaternion(rot_mat);
  quaternion.normalize();

  qw = quaternion.w();
  qx = quaternion.x();
  qy = quaternion.y();
  qz = quaternion.z();

}


/* --------------------------------------------------------------------------------------------- */


void rotation_matrix_to_euler_angle(
  const Eigen::Matrix3d& rot_mat,
  double& er_rad,
  double& ep_rad,
  double& ey_rad
){

  double qw;
  double qx;
  double qy;
  double qz;
  rotation_matrix_to_quaternion(rot_mat, qw, qx, qy, qz);
  quaternion_to_euler_angle(qw, qx, qy, qz, er_rad, ep_rad, ey_rad);

}


void euler_angle_to_rotation_matrix(
  const double& er_rad,
  const double& ep_rad,
  const double& ey_rad,
  Eigen::Matrix3d& rot_mat
){

  /*
  // code before Nov 19, 2023
  rot_mat = Eigen::AngleAxisd(ey_rad, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(ep_rad, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(er_rad, Eigen::Vector3d::UnitX());
  */

  double qw;
  double qx;
  double qy;
  double qz;
  euler_angle_to_quaternion(er_rad, ep_rad, ey_rad, qw, qx, qy, qz);
  quaternion_to_rotation_matrix(qw, qx, qy, qz, rot_mat);

}


/* --------------------------------------------------------------------------------------------- */


Eigen::Matrix4d get_transformation_matrix(
  const double& tx, const double& ty, const double& tz,                  // NOTE: translation x, y, z
  const double& qw, const double& qx, const double& qy, const double& qz // NOTE:  quaternion w, x, y, z
){

  /* declaration */
  Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();

  /* translation and rotation */
  Eigen::Vector3d T(tx, ty, tz);
  Eigen::Matrix3d R;
  quaternion_to_rotation_matrix(qw, qx, qy, qz, R);

  /* update */
  tf.block(0, 3, 3, 1) = T;
  tf.block(0, 0, 3, 3) = R;

  /* return */
  return tf;

}


Eigen::Matrix4d get_transformation_matrix(
  const double& tx,     const double& ty,     const double& tz,    // NOTE:  translation x, y, z
  const double& er_rad, const double& ep_rad, const double& ey_rad // NOTE: euler angles roll, pitch, yaw in [rad] (use ZYX rotation)
){

  /* declaration */
  Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();

  /* translation and rotation */
  Eigen::Vector3d T(tx, ty, tz);
  Eigen::Matrix3d R;
  euler_angle_to_rotation_matrix(er_rad, ep_rad, ey_rad, R);

  /* update */
  tf.block(0, 3, 3, 1) = T;
  tf.block(0, 0, 3, 3) = R;

  /* return */
  return tf;

}


Eigen::Matrix4d get_inverse_transformation_matrix(const Eigen::Matrix4d& transformation_matrix){

  /* declaration */
  Eigen::Matrix4d itf = Eigen::Matrix4d::Identity(); // inverse tf

  /* inverse */
  Eigen::Vector3d T = transformation_matrix.block(0, 3, 3, 1);
  Eigen::Matrix3d R = transformation_matrix.block(0, 0, 3, 3);
  itf.block(0, 3, 3, 1) = -R.transpose() * T;
  itf.block(0, 0, 3, 3) =  R.transpose();

  /* return */
  return itf;

}


/* --------------------------------------------------------------------------------------------- */


void transformation_matrix_to_pose(
  const Eigen::Matrix4d& transformation_matrix,
  double& px, double& py, double& pz,
  double& qw, double& qx, double& qy, double& qz
){

  /* parsing */
  Eigen::Vector3d T = transformation_matrix.block(0, 3, 3, 1);
  Eigen::Matrix3d R = transformation_matrix.block(0, 0, 3, 3);

  /* position */
  px = T(0);
  py = T(1);
  pz = T(2);

  /* quaternion */
  rotation_matrix_to_quaternion(R, qw, qx, qy, qz);

}


void transformation_matrix_to_pose(
  const Eigen::Matrix4d& transformation_matrix,
  double& px,     double& py,     double& pz,
  double& er_rad, double& ep_rad, double& ey_rad
){

  /* parsing */
  Eigen::Vector3d T = transformation_matrix.block(0, 3, 3, 1);
  Eigen::Matrix3d R = transformation_matrix.block(0, 0, 3, 3);

  /* position */
  px = T(0);
  py = T(1);
  pz = T(2);

  /* euler angle */
  rotation_matrix_to_euler_angle(R, er_rad, ep_rad, ey_rad);

}


/* --------------------------------------------------------------------------------------------- */


Eigen::Vector3d get_pos_from_transformation_matrix(const Eigen::Matrix4d& transformation_matrix){

  return transformation_matrix.block(0, 3, 3, 1);

}


Eigen::Matrix3d get_rot_mat_from_transformation_matrix(const Eigen::Matrix4d& transformation_matrix){

  return transformation_matrix.block(0, 0, 3, 3);

}


// NOTE: it returns quaternion: (qw, qx, qy, qz)
Eigen::Vector4d get_quaternion_from_transformation_matrix(const Eigen::Matrix4d& transformation_matrix){

  /* declaration */
  Eigen::Vector4d qwxyz;

  /* quaternion */
  rotation_matrix_to_quaternion(transformation_matrix.block(0, 0, 3, 3), qwxyz(0), qwxyz(1), qwxyz(2), qwxyz(3));

  /* return */
  return qwxyz;

}


// NOTE: it returns euler angle: (roll_rad, pitch_rad, yaw_rad)
Eigen::Vector3d get_euler_angle_from_transformation_matrix(const Eigen::Matrix4d& transformation_matrix){

  /* declaration */
  Eigen::Vector3d erpy_rad;

  /* euler angle */
  rotation_matrix_to_euler_angle(transformation_matrix.block(0, 0, 3, 3), erpy_rad(0), erpy_rad(1), erpy_rad(2));

  /* return */
  return erpy_rad;

}


/* --------------------------------------------------------------------------------------------- */


// NOTE: it returns yaw angle in [rad] (use ZYX rotation)
double get_yaw_angle_from_transformation_matrix(const Eigen::Matrix4d& transformation_matrix){

  /* euler angle */
  Eigen::Vector3d erpy_rad = get_euler_angle_from_transformation_matrix(transformation_matrix);

  /* return */
  return erpy_rad(2); // NOTE: yaw angle in [rad]

}


/* --------------------------------------------------------------------------------------------- */


}
}
