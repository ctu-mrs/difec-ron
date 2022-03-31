// clang: MatousFormat

// change the default Eigen formatting for prettier printing (has to be done before including Eigen)
#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "[", "]")

/* includes etc. //{ */

#include "main.h"

#include <cmath>
#include <thread>
#include <algorithm>

#include <boost/circular_buffer.hpp>

#include <nodelet/nodelet.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mrs_msgs/VelocityReferenceStamped.h>

#include <mrs_lib/scope_timer.h>
#include <difec_ron/FormationControlParamsConfig.h>

//}

namespace difec_ron
{
  class SwarmControl : public nodelet::Nodelet
  {

    /* some common type definitions //{ */

    // shortcut type to the dynamic reconfigure manager template instance
    using drmgr_t = mrs_lib::DynamicReconfigureMgr<difec_ron::FormationControlParamsConfig>;

    struct det_t
    {
      uint64_t id;
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

    struct agent_meas_t
    {
      agent_t formation;
      det_t detected;
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
      NODELET_INFO("Loading default dynamic parameters:");
      m_drmgr_ptr = std::make_unique<drmgr_t>(nh, m_node_name);

      // CHECK LOADING STATUS
      if (!m_drmgr_ptr->loaded_successfully())
      {
        NODELET_ERROR("Some compulsory parameters were not loaded successfully, ending the node");
        ros::shutdown();
        return;
      }

      mrs_lib::ParamLoader pl(nh, m_node_name);
      // LOAD STATIC PARAMETERS
      NODELET_INFO("Loading static parameters:");
      pl.loadParam("world_frame_id", m_world_frame_id);
      pl.loadParam("uav_frame_id", m_uav_frame_id);
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
      m_transformer.retryLookupNewest(true);

      mrs_lib::SubscribeHandlerOptions shopts(nh);
      shopts.no_message_timeout = ros::Duration(5.0);
      // Initialize subscribers
      mrs_lib::construct_object(m_sh_detections, shopts, "detections_in");

      m_period_buffer.set_capacity(50);

      //Initialize publishers
      m_pub_vis_formation = nh.advertise<visualization_msgs::MarkerArray>("visualization_formation", 10, true);
      m_pub_vis_p_cs = nh.advertise<visualization_msgs::MarkerArray>("visualization_p_c12", 10, true);
      m_pub_vis_u = nh.advertise<visualization_msgs::Marker>("visualization_u", 10);
      m_pub_vis_omega = nh.advertise<visualization_msgs::Marker>("visualization_omega", 10);
      m_pub_vel_ref = nh.advertise<mrs_msgs::VelocityReferenceStamped>("velocity_out", 10);
      //}

      m_pub_vis_formation.publish(formation_vis(m_formation, ros::Time::now()));

      m_vis_u_color.a = 1.0;
      m_vis_u_color.r = 1.0;
      m_vis_u_color.b = 1.0;

      m_vis_p1_color.a = 1.0;
      m_vis_p1_color.r = 0.5;
      m_vis_p1_color.g = 0.5;
      m_vis_p1_color.b = 1.0;

      m_vis_pmd_color.a = 0.5;
      m_vis_pmd_color.r = 0.5;
      m_vis_pmd_color.g = 0.5;
      m_vis_pmd_color.b = 1.0;

      m_vis_p2_color.a = 1.0;
      m_vis_p2_color.r = 0.5;
      m_vis_p2_color.g = 1.0;
      m_vis_p2_color.b = 0.5;

      m_vis_pmdR_color.a = 0.5;
      m_vis_pmdR_color.r = 0.5;
      m_vis_pmdR_color.g = 1.0;
      m_vis_pmdR_color.b = 0.5;

      m_main_thread = std::thread(&SwarmControl::main_loop, this);
      m_main_thread.detach();

      std::cout << "----------------------------------------------------------" << std::endl;
    }
    //}

    // --------------------------------------------------------------
    // |                    Main implementation                     |
    // --------------------------------------------------------------

    /* main_loop() method //{ */
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
    //}

    /* process_msg() method //{ */
    void process_msg(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr msg)
    {
      if (msg->poses.empty())
      {
        ROS_WARN_STREAM_THROTTLE(m_throttle_period, "[SwarmControl]: No neighbors detected out of " << m_formation.size()-1 << " total neighbors. Doing nothing.");
        return;
      }
      ROS_INFO_STREAM_THROTTLE(m_throttle_period, "[SwarmControl]: Receiving detections of " << msg->poses.size() << " neighbors out of " << m_formation.size()-1 << " total neighbors.");

      const ros::Time now = ros::Time::now();
      static ros::Time prev_action_time = now;
      const double period = (now - prev_action_time).toSec();
      m_period_buffer.push_back(period);
      const double fil_freq = 1.0/median(m_period_buffer);

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

      // fill the vector of detected agents and their corresponding desired poses in the formation
      std::vector<agent_meas_t> agent_meass;
      for (const auto& pose : msg->poses)
      {
        // first, find the corresponding agent in the formation
        std::optional<agent_t> formation_agent_opt;
        for (const auto& agent : m_formation)
          if (agent.id == pose.id)
            formation_agent_opt = agent;

        // ignore unexpected IDs
        if (!formation_agent_opt.has_value())
        {
          ROS_WARN_STREAM_THROTTLE(m_throttle_period, "[SwarmControl]: Received detection of neighbor with unexpected ID" << pose.id << ". Ignoring.");
          continue;
        }

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

        // add the measurement to the list
        const det_t det{pose.id, std::move(p), std::move(C), psi, sig};
        agent_meass.push_back({formation_agent_opt.value(), std::move(det)});
      }

      // some debugging shit
      visualization_msgs::MarkerArray p_cs;
      const bool visualize = m_pub_vis_p_cs.getNumSubscribers() > 0;

      // prepare the necessary constants
      const double l = m_drmgr_ptr->config.control__admissible_overshoot_probability;
      const double k_e = m_drmgr_ptr->config.control__proportional_constant/fil_freq;
      const double eps = 1e-9;

      vec3_t u_accum_1 = vec3_t::Zero();
      vec3_t u_accum_2 = vec3_t::Zero();
      double omega_accum_1 = 0.0;
      double omega_accum_2 = 0.0;
      for (const auto& meas : agent_meass)
      {
        // shorthand for the measured relative pose
        const auto& m = meas.detected;
        // shorthand for the desired relative pose
        const auto& d = meas.formation.desired_relative_pose;
        // heading difference between measured and desired position
        const double psi_md = m.psi - d.psi;
        // rotation matrix corresponding to the heading difference
        const mat3_t R_dpsi( anax_t(psi_md, vec3_t::UnitZ()) );
        // position difference between measured and desired position
        const vec3_t p_md = m.p - d.p;

        // | --------------------- calculate p_c1 --------------------- |
        // TODO: proper inverse calculation and checking
        const double sig_p = p_md.norm()/sqrt(p_md.transpose() * m.C.inverse() * p_md);
        const vec3_t p_c1 = sig_p*p_md.normalized()*mrs_lib::probit(l) + m.p;
        const double psi_c = m.sig * mrs_lib::signum(psi_md) * mrs_lib::probit(l) + m.psi;

        // | --------------------- calculate p_c2 --------------------- |
        const vec3_t p_dR = R_dpsi.transpose()*d.p;
        // construct the matrices V, L and then C_t
        const vec3_t v_d1 = vec3_t(-p_dR.y(), p_dR.x(), 0).normalized();
        const vec3_t v_d2 = vec3_t(p_dR.x(), p_dR.x(), 0).normalized();
        const vec3_t v_d3(0, 0, 1);
        mat3_t V;
        V.col(0) = v_d1;
        V.col(1) = v_d2;
        V.col(2) = v_d3;
        const mat3_t L = vec3_t(m.sig*m.sig, eps, eps).asDiagonal();
        const mat3_t C_t = V*L*V.transpose();
        const mat3_t C_c = m.C + C_t;
        const vec3_t p_mdR = m.p - p_dR;
        // TODO: proper inverse calculation and checking
        const vec3_t p_c2 = p_mdR/sqrt(p_mdR.transpose() * C_c.inverse() * p_mdR) * mrs_lib::probit(l) + m.p;

        // | --------------------- calculate p_c3 --------------------- |
        const double beta = -std::atan2(m.p.y(), m.p.x());
        const mat3_t R_beta( anax_t(beta, vec3_t::UnitZ()) );
        const mat3_t C_r = R_beta*m.C*R_beta.transpose();
        const double sig_gamma = std::sqrt(C_r(1, 1))/m.p.norm();
        const double tot_angle = sig_gamma * mrs_lib::signum(d.psi - m.psi) * mrs_lib::probit(l);
        const mat3_t R_tot( anax_t(tot_angle, vec3_t::UnitZ()) );
        const vec3_t p_c3 = R_tot*m.p;

        const mat3_t S = skew_symmetric(vec3_t::UnitZ());
        u_accum_1 += clamp(p_c1 - d.p, vec3_t::Zero(), p_md);
        u_accum_2 += clamp(p_c2 - p_dR, vec3_t::Zero(), p_mdR);
        const double tmp1 = d.p.transpose()*S.transpose()*p_c3;
        const double tmp2 = d.p.transpose()*S.transpose()*m.p;
        omega_accum_1 += clamp(tmp1, 0.0, tmp2);
        omega_accum_2 += clamp(psi_c - d.psi, 0.0, m.psi - d.psi);

        if (visualize)
        {
          visualization_msgs::Marker p_c1d_vis = vector_vis(p_c1 - d.p, now, m_vis_p1_color);
          p_c1d_vis.id = 4*meas.detected.id;
          p_c1d_vis.ns = "p^c1 - p^d";

          visualization_msgs::Marker p_md_vis = vector_vis(p_md, now, m_vis_pmd_color);
          p_md_vis.id = 4*meas.detected.id + 1;
          p_md_vis.ns = "p^m - p^d";

          visualization_msgs::Marker p_c2dR_vis = vector_vis(p_c2 - p_dR, now, m_vis_p2_color);
          p_c2dR_vis.id = 4*meas.detected.id + 2;
          p_c2dR_vis.ns = "p^c2 - p^dR";

          visualization_msgs::Marker p_mdR_vis = vector_vis(p_mdR, now, m_vis_pmdR_color);
          p_mdR_vis.id = 4*meas.detected.id + 3;
          p_mdR_vis.ns = "p^m - p^dR";

          p_cs.markers.push_back(p_c1d_vis);
          p_cs.markers.push_back(p_md_vis);
          p_cs.markers.push_back(p_c2dR_vis);
          p_cs.markers.push_back(p_mdR_vis);
        }
        /* ROS_INFO_STREAM_THROTTLE(m_throttle_period, "[SwarmControl]: p_c1: " << p_c1.transpose()); */
        /* ROS_INFO_STREAM_THROTTLE(m_throttle_period, "[SwarmControl]: p_c2: " << p_c2.transpose()); */
        /* ROS_INFO_STREAM_THROTTLE(m_throttle_period, "[SwarmControl]: p_c3: " << p_c3.transpose()); */
        /* ROS_INFO_STREAM_THROTTLE(m_throttle_period, "[SwarmControl]: psi_c: " << psi_c); */
      }

      const vec3_t u = k_e*(u_accum_1 + u_accum_2);
      const double omega = k_e*(omega_accum_1 + 2*omega_accum_2);

      ROS_INFO_STREAM_THROTTLE(m_throttle_period, "[SwarmControl]: Calculated action is u: " << u.transpose() << "^T, omega: " << omega << ". Average input frequency: " << fil_freq << "Hz");
      ROS_INFO_STREAM_THROTTLE(m_throttle_period, "[SwarmControl]: Rotation action components: heading: " << omega_accum_2 << ", bearing: " << omega_accum_1);

      m_pub_vis_u.publish(vector_vis(u, now, m_vis_u_color));
      m_pub_vis_omega.publish(heading_vis(omega, now));
      if (visualize)
        m_pub_vis_p_cs.publish(p_cs);

      if (m_drmgr_ptr->config.control__enabled)
      {
        mrs_msgs::VelocityReferenceStamped vel_out;
        vel_out.header.frame_id = m_uav_frame_id;
        vel_out.header.stamp = now;
        vel_out.reference.velocity.x = u.x();
        vel_out.reference.velocity.y = u.y();
        vel_out.reference.velocity.z = u.z();
        vel_out.reference.heading_rate = omega;
        vel_out.reference.use_heading_rate = true;
        m_pub_vel_ref.publish(vel_out);
      }
      else
        ROS_WARN_STREAM_THROTTLE(m_throttle_period, "Control is disabled, not publishing to MRS UAV pipeline!");
      prev_action_time = now;
    }
    //}

    // --------------------------------------------------------------
    // |                  Mathematical functions                    |
    // --------------------------------------------------------------

    /* median() method //{ */
    double median(const boost::circular_buffer<double>& buffer) const
    {
      std::vector<double> sorted;
      sorted.insert(std::begin(sorted), std::begin(buffer), std::end(buffer));
      const size_t median_pos = std::round(buffer.size()/2);
      std::nth_element(std::begin(sorted), std::begin(sorted) + median_pos, std::end(sorted));
      return sorted.at(median_pos);
    }
    //}

    /* skew_symmetric() method //{ */
    mat3_t skew_symmetric(const vec3_t& from) const
    {
      mat3_t ss;
      ss << 0, -from.z(), from.y(),
            from.z(), 0, -from.x(),
            -from.y(), from.x(), 0;
      return ss;
    }
    //}

    /* clamp() method //{ */
    double clamp(const double what, const double a, const double b) const
    {
      double min = a;
      double max = b;
      if (min > max)
        std::swap(min, max);
      if (what < min || what > max)
        return 0.0;
      else
        return what;
    }

    vec3_t clamp(const vec3_t& what, const vec3_t& a, const vec3_t& b) const
    {
      vec3_t min = a;
      vec3_t max = b;
      double nmin = min.norm();
      double nmax = max.norm();
      if (nmin > nmax)
      {
        std::swap(min, max);
        std::swap(nmin, nmax);
      }
      const vec3_t dir = max - min;
      const double proj = what.dot(dir)/dir.norm();
      if (proj < nmin || proj > nmax)
        return vec3_t::Zero();
      else
        return what;
    }
    //}

    // --------------------------------------------------------------
    // |                  Visualization functions                   |
    // --------------------------------------------------------------

    /* vector_vis() method //{ */
    visualization_msgs::Marker vector_vis(const vec3_t& vec, const ros::Time& time, const std_msgs::ColorRGBA& color) const
    {
      visualization_msgs::Marker mkr;
      mkr.header.frame_id = m_uav_frame_id;
      mkr.header.stamp = time;
      mkr.type = visualization_msgs::Marker::ARROW;
      mkr.pose.orientation.w = 1.0;
      mkr.scale.x = 0.05; // shaft diameter
      mkr.scale.y = 0.15; // head diameter
      mkr.color = color;
      geometry_msgs::Point pnt;
      mkr.points.push_back(pnt);
      pnt.x = vec.x();
      pnt.y = vec.y();
      pnt.z = vec.z();
      mkr.points.push_back(pnt);
      return mkr;
    }
    //}

    /* heading_vis() method //{ */
    visualization_msgs::Marker heading_vis(const double hdg, const ros::Time& time) const
    {
      visualization_msgs::Marker mkr;
      mkr.header.frame_id = m_uav_frame_id;
      mkr.header.stamp = time;
      mkr.type = visualization_msgs::Marker::ARROW;
      mkr.pose.orientation.w = 1.0;
      mkr.scale.x = 0.05; // shaft diameter
      mkr.scale.y = 0.15; // head diameter
      mkr.color.g = 1.0;
      mkr.color.b = 1.0;
      mkr.color.a = 1.0;
      geometry_msgs::Point pnt;
      mkr.points.push_back(pnt);
      pnt.z = hdg;
      mkr.points.push_back(pnt);
      return mkr;
    }
    //}

    /* formation_vis() method //{ */
    visualization_msgs::MarkerArray formation_vis(const std::vector<agent_t>& formation, const ros::Time& time) const
    {
      visualization_msgs::MarkerArray ret;
      for (const auto& agent : formation)
      {
        visualization_msgs::Marker mkr;

        mkr.id = agent.id;
        mkr.ns = agent.uav_name;
        mkr.header.frame_id = m_uav_frame_id;
        mkr.header.stamp = time;
        mkr.frame_locked = true;
        mkr.type = visualization_msgs::Marker::ARROW;
        mkr.pose.orientation.w = 1.0;
        mkr.scale.x = 0.05; // shaft diameter
        mkr.scale.y = 0.15; // head diameter
        mkr.color.r = 1.0;
        mkr.color.g = 1.0;
        mkr.color.a = 1.0;

        geometry_msgs::Point pnt;

        const vec3_t& des_pos = agent.desired_relative_pose.p;
        pnt.x = des_pos.x();
        pnt.y = des_pos.y();
        pnt.z = des_pos.z();
        mkr.points.push_back(pnt);

        const vec3_t des_pos_ori = des_pos + anax_t(agent.desired_relative_pose.psi, vec3_t::UnitZ())*vec3_t::UnitX();
        pnt.x = des_pos_ori.x();
        pnt.y = des_pos_ori.y();
        pnt.z = des_pos_ori.z();
        mkr.points.push_back(pnt);

        ret.markers.push_back(mkr);
      }
      return ret;
    }
    //}

    // --------------------------------------------------------------
    // |                  Miscellaneous functions                   |
    // --------------------------------------------------------------

    /* to_string() method //{ */
    std::string to_string(XmlRpc::XmlRpcValue::Type type) const
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
    //}

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
    void print_formation(const std::vector<agent_t>& m_formation) const
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
    boost::circular_buffer<double> m_period_buffer;
    std_msgs::ColorRGBA m_vis_u_color;
    std_msgs::ColorRGBA m_vis_p1_color;
    std_msgs::ColorRGBA m_vis_pmd_color;
    std_msgs::ColorRGBA m_vis_p2_color;
    std_msgs::ColorRGBA m_vis_pmdR_color;

  private:
    // --------------------------------------------------------------
    // |                ROS-related member variables                |
    // --------------------------------------------------------------

    /* ROS related variables (subscribers, timers etc.) //{ */
    std::string m_node_name;

    mrs_lib::Transformer m_transformer;
    std::unique_ptr<drmgr_t> m_drmgr_ptr;
    mrs_lib::SubscribeHandler<mrs_msgs::PoseWithCovarianceArrayStamped> m_sh_detections;

    ros::Publisher m_pub_vis_formation;
    ros::Publisher m_pub_vis_p_cs;
    ros::Publisher m_pub_vis_u;
    ros::Publisher m_pub_vis_omega;
    ros::Publisher m_pub_vel_ref;

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
