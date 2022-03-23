#ifndef MAIN_H
#define MAIN_H

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Range.h>
#include <dynamic_reconfigure/server.h>

#include <std_srvs/Trigger.h>
#include <std_msgs/Float64.h>
#include <mrs_msgs/Sphere.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include <list>

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>

#include <uav_detect/Detection.h>
#include <uav_detect/Detections.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/dynamic_reconfigure_mgr.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/vector_converter.h>
#include <mrs_lib/utils.h>

#include "uav_detect/point_types.h"

namespace uav_detect
{
  using pt_t = ouster_ros::Point;
  using pc_t = pcl::PointCloud<pt_t>;
  
  using pt_XYZ_t = pcl::PointXYZ;
  using pc_XYZ_t = pcl::PointCloud<pt_XYZ_t>;
  using pt_XYZR_t = uav_detect::PointXYZR;
  using pc_XYZR_t = pcl::PointCloud<pt_XYZR_t>;
  using pt_XYZRI_t = uav_detect::PointXYZRI;
  using pc_XYZRI_t = pcl::PointCloud<pt_XYZRI_t>;
  using pt_XYZt_t = pcl::PointXYZI;
  using pc_XYZt_t = pcl::PointCloud<pt_XYZt_t>;
  using octree = pcl::octree::OctreePointCloudSearch<pt_XYZ_t>;
  
  using vec2_t = Eigen::Vector2d;
  using vec3_t = Eigen::Vector3f;
  using vec3i_t = Eigen::Vector3i;
  using vec4_t = Eigen::Vector4f;
  using quat_t = Eigen::Quaternionf;
  using anax_t = Eigen::AngleAxisf;
  using mat3_t = Eigen::Matrix3f;
  using vec3s_t = Eigen::Matrix<float, 3, -1>;
}

#endif //  MAIN_H
