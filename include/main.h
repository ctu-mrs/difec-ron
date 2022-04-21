#ifndef MAIN_H
#define MAIN_H

#include <ros/package.h>
#include <ros/ros.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/dynamic_reconfigure_mgr.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/geometry/conversions.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/vector_converter.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/math.h>

namespace difec_ron
{
  using vec2_t = Eigen::Vector2d;
  using vec3_t = Eigen::Vector3d;
  using vec3i_t = Eigen::Vector3i;
  using vec4_t = Eigen::Vector4d;
  using quat_t = Eigen::Quaterniond;
  using anax_t = Eigen::AngleAxisd;
  using mat3_t = Eigen::Matrix3d;
  using mat6_t = Eigen::Matrix<double, 6, 6>;
  using rads_t = mrs_lib::geometry::sradians;
}

#endif //  MAIN_H
