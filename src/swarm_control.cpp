/* includes etc. //{ */

#include "main.h"

#include <cmath>
#include <thread>
#include <algorithm>

#include <nodelet/nodelet.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <mrs_lib/scope_timer.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>

//}

namespace difec_ron
{
  class SwarmControl : public nodelet::Nodelet
  {

    struct det_t
    {
      vec3_t p; // relative position measurement
      mat3_t C; // position uncertainty covariance matrix
      double psi; // relative heading measurement
      double sig; // heading uncertainty standard deviation
    };

  public:
    /* onInit() method //{ */
    void onInit()
    {
      ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
      ROS_INFO("[SwarmControl]: Waiting for valid time...");
      ros::Time::waitForValid();

      m_node_name = "SwarmControl";

      /* Load parameters from ROS //{*/
      mrs_lib::ParamLoader pl(nh, m_node_name);
      // LOAD STATIC PARAMETERS
      NODELET_INFO("Loading static parameters:");
      const auto uav_name = pl.loadParam2<std::string>("uav_name");
      pl.loadParam("world_frame_id", m_world_frame_id);
      pl.loadParam("transform_lookup_timeout", m_transform_lookup_timeout);
      pl.loadParam("throttle_period", m_throttle_period);

      // CHECK LOADING STATUS
      if (!pl.loadedSuccessfully())
      {
        NODELET_ERROR("Some compulsory parameters were not loaded successfully, ending the node");
        ros::shutdown();
        return;
      }
      //}

      /* Create publishers and subscribers //{ */
      // Initialize transformer
      m_transformer = mrs_lib::Transformer(nh, m_node_name);

      mrs_lib::SubscribeHandlerOptions shopts(nh);
      shopts.no_message_timeout = ros::Duration(5.0);
      // Initialize subscribers
      mrs_lib::construct_object(m_sh_detections, shopts, "detections_in");
      //}

      m_main_thread = std::thread(&SwarmControl::main_loop, this);
      m_main_thread.detach();

      std::cout << "----------------------------------------------------------" << std::endl;
    }
    //}

    void main_loop()
    {
      const ros::WallDuration timeout(1.0/10.0);
      while (ros::ok())
      {
        const auto msg_ptr = m_sh_detections.waitForNew(timeout);
        if (msg_ptr)
          process_msg(msg_ptr);
      }
    }

    void process_msg(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr msg)
    {
      ROS_INFO_STREAM_THROTTLE(m_throttle_period, "[SwarmControl]: Receiving UVDAR detections (see " << msg->poses.size() << " neighbors)");

      const ros::Time now = ros::Time::now();
      const int n_m = msg->poses.size();

      const auto tf_opt = m_transformer.getTransform(msg->header.frame_id, msg->header.stamp, m_uav_frame_id, now, m_world_frame_id);
      if (!tf_opt.has_value())
      {
        ROS_ERROR_STREAM_THROTTLE(m_throttle_period, "[SwarmControl]: Could not lookup transformation from \"" << msg->header.frame_id << "\" to \"" << m_uav_frame_id << "\" through inertial frame \"" << m_world_frame_id << "\", ignoring detection");
        return;
      }
      const auto tf_ros = tf_opt.value();
      const Eigen::Isometry3d tf = tf2::transformToEigen(tf_ros);
      const mat3_t tf_rot = tf.rotation();
      const double tf_hdg = mrs_lib::geometry::headingFromRot(tf_rot);

      // convert the message to measurements in a usable format
      std::vector<det_t> measurements;
      measurements.reserve(n_m);
      for (const auto& pose : msg->poses)
      {
        // position in the original frame of the message
        const vec3_t pos_orig = mrs_lib::geometry::toEigen(pose.pose.position);
        // orientation in the original frame of the message
        const quat_t ori_orig = mrs_lib::geometry::toEigen(pose.pose.orientation);
        // heading in the original frame of the message
        const double hdg_orig = mrs_lib::geometry::headingFromRot(ori_orig);
        // covariance matrix of the pose in the original frame of the message
        const mat6_t cov_orig = mrs_lib::geometry::toEigenMatrix(pose.covariance);

        // position measurement
        const vec3_t p = tf*pos_orig;
        // heading measurement
        // TODO: make sure this makes sense
        const double psi = hdg_orig + tf_hdg;

        // covariance matrix of the position
        const mat3_t C = mrs_lib::geometry::rotateCovariance(cov_orig.block<3, 3>(0, 0), tf_rot);;
        // standard deviation of the heading (hopefully...)
        // TODO: check that yaw really corresponds to heading here and that it doesn't have to be transformed
        const double sig = cov_orig(6, 6);

        // add the detection to the list
        const det_t det{std::move(p), std::move(C), psi, sig};
        measurements.push_back(std::move(det));
      }

/*       vec3_t u_a = vec3_t::Zero(); */
/*       for (int it = 0; it < n_m; it++) */
/*       { */
/*         const vec3_t& p_c1 = dets_c1.at(it); */
/*         const vec3_t& p_d = dets_d.at(it); */
/*         const vec3_t& p_m = dets_m.at(it); */
/*         u_a += clamp(p_c1 - p_d, p_d, p_m); */
/*       } */

/*       vec3_t u_b = vec3_t::Zero(); */
/*       for (int it = 0; it < n_m; it++) */
/*       { */
/*         const vec3_t& p_c2 = dets_c2.at(it); */
/*         const vec3_t& p_dR = dets_dR.at(it); */
/*         const vec3_t& p_m = dets_m.at(it); */
/*         u_b += clamp(p_c2 - p_dR, p_dR, p_m); */
/*       } */

/*       const vec3_t u = k_e*(u_a + u_b); */
    }

  private:

    std::thread m_main_thread;

  private:
    // --------------------------------------------------------------
    // |                ROS-related member variables                |
    // --------------------------------------------------------------

    /* ROS related variables (subscribers, timers etc.) //{ */
    mrs_lib::Transformer m_transformer;
    mrs_lib::SubscribeHandler<mrs_msgs::PoseWithCovarianceArrayStamped> m_sh_detections;
    std::string m_node_name;
    //}

  private:
    // --------------------------------------------------------------
    // |                 Parameters, loaded from ROS                |
    // --------------------------------------------------------------

    /* Parameters, loaded from ROS //{ */

    std::string m_world_frame_id;
    std::string m_uav_frame_id;
    ros::Duration m_transform_lookup_timeout;
    double m_throttle_period;

    //}

  };  // class SwarmControl
};    // namespace difec_ron

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(difec_ron::SwarmControl, nodelet::Nodelet)
