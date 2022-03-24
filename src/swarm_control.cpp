// clang: MatousFormat

// change the default Eigen formatting for prettier printing (has to be done before including Eigen)
#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "[", "]")

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

    /* some common type definitions //{ */
    
    struct det_t
    {
      vec3_t p; // relative position measurement
      mat3_t C; // position uncertainty covariance matrix
      double psi; // relative heading measurement
      double sig; // heading uncertainty standard deviation
    };
    
    struct pose_t
    {
      vec3_t p; // position
      double psi; // heading
    };
    
    struct agent_t
    {
      uint64_t id; // ID of this agent
      std::string uav_name;
      pose_t desired_relative_pose; // the desired pose in the formation relative to the current agent
    };
    
    //}

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
      pl.loadParam("world_frame_id", m_world_frame_id);
      pl.loadParam("transform_lookup_timeout", m_transform_lookup_timeout);
      pl.loadParam("throttle_period", m_throttle_period);

      // load the formation
      const auto formation_opt = load_formation(pl);

      // CHECK LOADING STATUS
      if (!pl.loadedSuccessfully() || !formation_opt.has_value())
      {
        NODELET_ERROR("Some compulsory parameters were not loaded successfully, ending the node");
        ros::shutdown();
        return;
      }

      m_formation = formation_opt.value();
      NODELET_INFO_STREAM("[SwarmControl]: Loaded formation relative to UAV " << m_uav_name << ": ");
      print_formation(m_formation);
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
      ROS_INFO_STREAM_THROTTLE(m_throttle_period, "[SwarmControl]: Receiving detections of " << msg->poses.size() << " neighbors out of " << m_formation.size()-1 << " total neighbors.");

      const ros::Time now = ros::Time::now();

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

      // first, find out which neighbors we see and save them sorted
      std::vector<std::optional<mrs_msgs::PoseWithCovarianceIdentified>> neighbor_dets(m_formation.size(), std::nullopt);
      for (const auto& pose : msg->poses)
      {
        const int id = pose.id;
        if ((size_t)id >= m_formation.size())
        {
          ROS_WARN_STREAM_THROTTLE(m_throttle_period, "[SwarmControl]: Received detection of unknown neighbor with ID" << id << ". Max. expected ID is " << m_formation.size()-1);
          continue;
        }
        neighbor_dets.at(id) = pose;
      }

      // convert the message to measurements in a usable format
      std::vector<det_t> measurements;
      std::vector<pose_t> desired_poses;
      for (const auto& pose_opt : neighbor_dets)
      {
        if (!pose_opt.has_value())
          continue;
        const auto& pose = pose_opt.value();

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
        measurements.emplace_back(std::move(det));

        // add the corresponding desired pose to the list
        for (const auto& agent : m_formation)
          if (agent.id == pose.id)
            desired_poses.push_back(agent.desired_relative_pose);
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

    std::string to_string(XmlRpc::XmlRpcValue::Type type)
    {
      switch (type)
      {
        case XmlRpc::XmlRpcValue::Type::TypeInvalid: return "TypeInvalid";
        case XmlRpc::XmlRpcValue::Type::TypeBoolean: return "TypeBoolean";
        case XmlRpc::XmlRpcValue::Type::TypeInt: return "TypeInt";
        case XmlRpc::XmlRpcValue::Type::TypeDouble: return "TypeDouble";
        case XmlRpc::XmlRpcValue::Type::TypeString: return "TypeString";
        case XmlRpc::XmlRpcValue::Type::TypeDateTime: return "TypeDateTime";
        case XmlRpc::XmlRpcValue::Type::TypeBase64: return "TypeBase64";
        case XmlRpc::XmlRpcValue::Type::TypeArray: return "TypeArray";
        case XmlRpc::XmlRpcValue::Type::TypeStruct: return "TypeStruct";
      }
      return "TypeUnknown";
    }

    /* load_formation() method //{ */
    std::optional<std::vector<agent_t>> load_formation(mrs_lib::ParamLoader& pl)
    {
      const auto this_uav_name = pl.loadParam2<std::string>("uav_name");
      const auto xml_formation = pl.loadParam2<XmlRpc::XmlRpcValue>("formation");
    
      bool all_ok = true;
      std::vector<agent_t> formation;
      for (const auto& uav : xml_formation)
      {
        const std::string uav_name = uav.first;
        const auto uav_data = uav.second;
        ROS_INFO_STREAM("[SwarmControl]: Loading formation data for UAV " << uav_name);
    
        std::optional<uint64_t> id = std::nullopt;
        std::optional<std::array<double, 3>> coords = std::nullopt;
        std::optional<double> heading = std::nullopt;
    
        for (const auto& el : uav_data)
        {
          if (el.first == "uvdar_id" && el.second.getType() == XmlRpc::XmlRpcValue::TypeInt)
            id = int(el.second);
          else if (el.first == "heading" && el.second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
            heading = double(el.second);
          else if (el.first == "position" && el.second.getType() == XmlRpc::XmlRpcValue::TypeArray && el.second.size() == 3)
          {
            coords = {0,0,0};
            for (int it = 0; it < el.second.size(); it++)
              coords.value()[it] = el.second[it];
          }
          else
          {
            const auto type = to_string(el.second.getType());
            ROS_WARN_STREAM("[SwarmControl]: Unexpected or invalid XML member encountered: " << el.first << " (" << type << "), ignoring.");
            all_ok = false;
          }
        }
    
        if (!id.has_value() || !coords.has_value() || !heading.has_value())
        {
          ROS_WARN_STREAM("[SwarmControl]: Couldn't load all necessary formation information about UAV " << uav_name << ", ignoring. Expected fields:\n\tuvdar_id (int)\n\theading (double)\n\tposition (double[3])");
          all_ok = false;
          continue;
        }
        const vec3_t position(coords.value()[0], coords.value()[1], coords.value()[2]);
        const pose_t agent_pose = {position, heading.value()};
        const agent_t agent = {id.value(), uav_name, agent_pose};
        formation.emplace_back(agent);
      }

      // find this UAV in the formation specification
      const auto this_agent_it = std::find_if(std::begin(formation), std::end(formation), [&this_uav_name](const auto& agent){return agent.uav_name == this_uav_name;});
      if (this_agent_it == std::end(formation))
      {
        ROS_ERROR_STREAM("[SwarmControl]: The current UAV " << this_uav_name << " not found in the specified formation!");
        return std::nullopt;
      }
      const agent_t& this_agent = *this_agent_it;
      m_uav_name = this_uav_name;
      m_uvdar_id = this_agent.id;
      const pose_t this_pose = this_agent.desired_relative_pose;

      // now recalculate the agent poses to be relative to the current UAV
      const mat3_t rot( anax_t(-this_pose.psi, vec3_t::UnitZ()) );
      for (auto& agent : formation)
      {
        agent.desired_relative_pose.p = rot*(agent.desired_relative_pose.p - this_pose.p);
        agent.desired_relative_pose.psi = agent.desired_relative_pose.psi - this_pose.psi;
      }
    
      if (all_ok)
        return formation;
      else
        return std::nullopt;
    }
    //}

    /* print_formation() method //{ */
    void print_formation(const std::vector<agent_t>& m_formation)
    {
      for (const auto& agent : m_formation)
      {
        std::cout << "\t" << agent.uav_name << " (ID" << agent.id << ")\n";
        std::cout << "\t\tposition: " << agent.desired_relative_pose.p.transpose() << ", heading: " << agent.desired_relative_pose.psi << "\n";
      }
    }
    //}

  private:

    std::thread m_main_thread;
    std::string m_uav_name;
    uint64_t m_uvdar_id;

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

    std::vector<agent_t> m_formation;

    //}

  };  // class SwarmControl
};    // namespace difec_ron

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(difec_ron::SwarmControl, nodelet::Nodelet)
