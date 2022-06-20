// clang: MatousFormat

// change the default Eigen formatting for prettier printing (has to be done before including Eigen)
#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "[", "]")

/* includes etc. //{ */

#include "main.h"

#include <cmath>
#include <thread>
#include <algorithm>

#include <boost/circular_buffer.hpp>

#include <yaml-cpp/yaml.h>

#include <nodelet/nodelet.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mrs_msgs/VelocityReferenceStamped.h>

#include <mrs_lib/scope_timer.h>
#include <difec_ron/FormationControlParamsConfig.h>

//}

namespace difec_ron
{
  class FormationControl : public nodelet::Nodelet
  {

    /* some common type definitions //{ */

    // shortcut type to the dynamic reconfigure manager template instance
    using drmgr_t = mrs_lib::DynamicReconfigureMgr<difec_ron::FormationControlParamsConfig>;

    struct det_t
    {
      uint64_t id;
      vec3_t p; // relative position measurement
      mat3_t C; // position uncertainty covariance matrix
      rads_t psi; // relative heading measurement
      double sig; // heading uncertainty standard deviation
    };
    
    struct pose_t
    {
      vec3_t p; // position
      rads_t psi; // heading
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

    struct visualization_t
    {
      visualization_t(const bool visualize) : visualize(visualize) {};
      const bool visualize;

      visualization_msgs::MarkerArray p_cs;
      visualization_msgs::MarkerArray psi_cs;
      visualization_msgs::MarkerArray p_c2s;
    };
    
    //}

  public:
    /* onInit() method //{ */
    void onInit()
    {
      ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
      ROS_INFO("[FormationControl]: Waiting for valid time...");
      ros::Time::waitForValid();

      m_node_name = "FormationControl";

      /* Load parameters from ROS //{*/
      NODELET_INFO("Loading default dynamic parameters:");
      m_drmgr_ptr = std::make_unique<drmgr_t>(nh, true, m_node_name, boost::bind(&FormationControl::dynparam_callback, this, _1, _2));

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
      pl.loadParam("uav_name", m_uav_name);
      pl.loadParam("world_frame_id", m_world_frame_id);
      pl.loadParam("uav_frame_id", m_uav_frame_id);
      pl.loadParam("transform_lookup_timeout", m_transform_lookup_timeout);
      pl.loadParam("throttle_period", m_throttle_period);
      const double repeat_rate = pl.loadParam2<double>("control/repeat_rate");

      // load the formation
      const auto formation_opt = load_formation(m_uav_name, m_drmgr_ptr->config.formation__filename);

      // CHECK LOADING STATUS
      if (!pl.loadedSuccessfully() || !formation_opt.has_value())
      {
        NODELET_ERROR("Some compulsory parameters were not loaded successfully, ending the node");
        ros::shutdown();
        return;
      }

      m_formation = formation_opt.value();
      NODELET_INFO_STREAM("[FormationControl]: Loaded formation relative to UAV " << m_uav_name << ": ");
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
      m_pub_vis_psi_cs = nh.advertise<visualization_msgs::MarkerArray>("visualization_psi_comp", 10, true);
      m_pub_vis_p_c1s = nh.advertise<visualization_msgs::MarkerArray>("visualization_p_c1", 10, true);
      m_pub_vis_u = nh.advertise<visualization_msgs::Marker>("visualization_u", 10);
      m_pub_vis_omega = nh.advertise<visualization_msgs::Marker>("visualization_omega", 10);
      m_pub_vel_ref = nh.advertise<mrs_msgs::VelocityReferenceStamped>("velocity_out", 10);
      //}

      m_tim_vel_cmd = nh.createTimer(ros::Duration(1.0/repeat_rate), &FormationControl::velocity_command_loop, this);
      m_tim_vel_cmd = nh.createTimer(ros::Duration(1.0), &FormationControl::dynrec_publish_loop, this);

      m_pub_vis_formation.publish(formation_vis(m_formation, ros::Time::now()));

      /* visualization colors //{ */
      
      // linear velocity colors
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
      
      
      // rotation speed colors
      m_vis_omega_color.a = 1.0;
      m_vis_omega_color.g = 1.0;
      m_vis_omega_color.b = 1.0;
      
      m_vis_psidc_color.a = 1.0;
      m_vis_psidc_color.r = 1.0;
      m_vis_psidc_color.g = 0.2;
      m_vis_psidc_color.b = 0.2;
      
      m_vis_psidm_color.a = 0.5;
      m_vis_psidm_color.r = 1.0;
      m_vis_psidm_color.g = 0.2;
      m_vis_psidm_color.b = 0.2;
      
      m_vis_psi_bearing_r_color.a = 1.0;
      m_vis_psi_bearing_r_color.r = 0.5;
      m_vis_psi_bearing_r_color.g = 1.0;
      m_vis_psi_bearing_r_color.b = 0.2;
      
      m_vis_psi_bearing_o_color.a = 0.5;
      m_vis_psi_bearing_o_color.r = 0.2;
      m_vis_psi_bearing_o_color.g = 1.0;
      m_vis_psi_bearing_o_color.b = 0.2;
      
      //}

      m_main_thread = std::thread(&FormationControl::main_loop, this);
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
      std::scoped_lock lck(m_mutex);

      if (msg->poses.empty())
      {
        ROS_WARN_STREAM_THROTTLE(m_throttle_period, "[FormationControl]: No neighbors detected out of " << m_formation.size()-1 << " total neighbors. Doing nothing.");
        return;
      }
      ROS_INFO_STREAM_THROTTLE(m_throttle_period, "[FormationControl]: Receiving detections of " << msg->poses.size() << " neighbors out of " << m_formation.size()-1 << " total neighbors.");

      const ros::Time now = ros::Time::now();
      static ros::Time prev_action_time = now;
      const double period = (now - prev_action_time).toSec();
      m_period_buffer.push_back(period);
      const double fil_freq = 1.0/median(m_period_buffer);

      const auto tf_opt = m_transformer.getTransform(msg->header.frame_id, msg->header.stamp, m_uav_frame_id, now, m_world_frame_id);
      if (!tf_opt.has_value())
      {
        ROS_ERROR_STREAM_THROTTLE(m_throttle_period, "[FormationControl]: Could not lookup transformation from \"" << msg->header.frame_id << "\" to \"" << m_uav_frame_id << "\" through inertial frame \"" << m_world_frame_id << "\", ignoring detection");
        return;
      }
      const auto tf_ros = tf_opt.value();
      const Eigen::Isometry3d tf = tf2::transformToEigen(tf_ros);
      const mat3_t tf_rot = tf.rotation();
      const rads_t tf_hdg = mrs_lib::geometry::headingFromRot(tf_rot);

      // | ----------------- Prepare data for DIFEC ----------------- |
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
          ROS_WARN_STREAM_THROTTLE(m_throttle_period, "[FormationControl]: Received detection of neighbor with unexpected ID" << pose.id << ". Ignoring.");
          continue;
        }

        // position in the original frame of the message
        const vec3_t pos_orig = mrs_lib::geometry::toEigen(pose.pose.position);
        // orientation in the original frame of the message
        const quat_t ori_orig = mrs_lib::geometry::toEigen(pose.pose.orientation);
        // heading in the original frame of the message
        const rads_t hdg_orig = mrs_lib::geometry::headingFromRot(ori_orig);
        // covariance matrix of the pose in the original frame of the message
        const mat6_t cov_orig = mrs_lib::geometry::toEigenMatrix(pose.covariance);

        // position measurement
        const vec3_t p = tf*pos_orig;
        // heading measurement
        // TODO: make sure this makes sense
        const rads_t psi = hdg_orig + tf_hdg;

        // covariance matrix of the position
        const mat3_t C = mrs_lib::geometry::rotateCovariance(cov_orig.block<3, 3>(0, 0), tf_rot);;
        // standard deviation of the heading (hopefully...)
        // TODO: check that yaw really corresponds to heading here and that it doesn't have to be transformed
        // TODO: It should be transformed, but the difference for sigma will typically not be that bad
        const double sig = sqrt(cov_orig(5, 5)); //square root, since covariance has squared elements

        ROS_INFO_STREAM("[FormationControl]: Target ID: " << pose.id << " has C: " << std::endl << cov_orig);
        ROS_INFO_STREAM("[FormationControl]: Target ID: " << pose.id << " has heading sigma: " << sig << " rad");

        // add the measurement to the list
        const det_t det{pose.id, std::move(p), std::move(C), psi, sig};
        agent_meass.push_back({formation_agent_opt.value(), std::move(det)});
      }

      // some debugging shit
      visualization_t vis(m_pub_vis_p_cs.getNumSubscribers() > 0 || m_pub_vis_psi_cs.getNumSubscribers() > 0 || m_pub_vis_p_c1s.getNumSubscribers() > 0);

      // | ----------------------- APPLY DIFEC ---------------------- |
      vec3_t u;
      double omega;
      if (m_drmgr_ptr->config.control__use_noise)
      {
        const auto action = difec_ron(agent_meass, now, fil_freq, vis);
        u = action.first;
        omega = action.second;
      }
      else
      {
        const auto action = difec(agent_meass, now, fil_freq, vis);
        u = action.first;
        omega = action.second;
      }

      ROS_INFO_STREAM_THROTTLE(m_throttle_period, "[FormationControl]: Calculated action is u: " << u.transpose() << "^T, omega: " << omega << ". Average input frequency: " << fil_freq << "Hz");

      m_pub_vis_u.publish(vector_vis(u, now, m_vis_u_color));
      m_pub_vis_omega.publish(heading_vis(omega, now, m_vis_omega_color));
      if (vis.visualize)
      {
        m_pub_vis_p_cs.publish(vis.p_cs);
        m_pub_vis_psi_cs.publish(vis.psi_cs);
        m_pub_vis_p_c1s.publish(vis.p_c2s);
      }

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
        std::scoped_lock lck(m_last_vel_out_mtx);
        m_last_vel_out = vel_out;
        m_last_vel_out_set = true;
        m_pub_vel_ref.publish(vel_out);
      }
      else
        ROS_WARN_STREAM_THROTTLE(m_throttle_period, "Control is disabled, not publishing to MRS UAV pipeline!");
      prev_action_time = now;
    }
    //}
    
    /* difec_ron() method //{ */
    std::pair<vec3_t, double> difec_ron(const std::vector<agent_meas_t>& agent_meass, const ros::Time& now, const double fil_freq, visualization_t& vis)
    {
      // prepare the necessary constants
      const double l = m_drmgr_ptr->config.control__admissible_overshoot_probability;
      const double k_e = m_drmgr_ptr->config.control__proportional_constant/fil_freq;
    
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
        const rads_t psi_md = m.psi - d.psi;
        // rotation matrix corresponding to the heading difference
        const mat3_t R_dpsi( anax_t(psi_md.value(), vec3_t::UnitZ()) );
        // position difference between measured and desired position
        const vec3_t p_md = m.p - d.p;

        ROS_INFO_STREAM("[FormationControl]: Target ID: " << m.id << " has heading sigma: " << m.sig << " rad");
    
        // | --------------------- calculate p_c1 --------------------- |
        // TODO: proper inverse calculation and checking
        const double sig_p = p_md.norm()/sqrt(p_md.transpose() * m.C.inverse() * p_md);
        const vec3_t p_c1 = sig_p*p_md.normalized()*mrs_lib::probit(l) + m.p;
        const rads_t psi_c = m.sig * mrs_lib::signum(psi_md) * mrs_lib::probit(l) + m.psi;
    
        // | --------------------- calculate p_c2 --------------------- |
        const vec3_t p_dR = R_dpsi*d.p;
        // convert envelope the circular gaussian distribution with a 3D gaussian
        const auto [p_dRp, C_t] = envelope_circular_gaussian(p_dR, m.sig);
        const vec3_t p_mdRp = m.p - p_dRp;
        const vec3_t dir = p_mdRp.normalized();
        // the merged gaussian distrubution around the measurement
        const mat3_t C_c = m.C + C_t;
        // project the merged and offset distribution to a line between p_m and p_dR
        const auto [mu, sig] = project_gaussian_to_line(m.p, C_c, p_dRp, dir);
        // calculate the resulting vector that intersects the desired overshoot probability ellipsoid
        const double len = mu + sig*mrs_lib::probit(l);
        const vec3_t p_c2 = p_dRp + dir.normalized() * len;
    
        // | --------------------- calculate p_c3 --------------------- |
        const rads_t beta = -std::atan2(m.p.y(), m.p.x());
        const mat3_t R_beta( anax_t(beta.value(), vec3_t::UnitZ()) );
        const mat3_t C_r = R_beta*m.C*R_beta.transpose();
        const double sig_gamma = std::sqrt(C_r(1, 1))/m.p.norm();
        const rads_t tot_angle = sig_gamma * mrs_lib::signum(d.psi - m.psi) * mrs_lib::probit(l);
        const mat3_t R_tot( anax_t(tot_angle.value(), vec3_t::UnitZ()) );
        const vec3_t p_c3 = R_tot*m.p;
    
        // | ------------- accumulate the calculated stuff ------------ |
        const mat3_t S = skew_symmetric(vec3_t::UnitZ());
        u_accum_1 += clamp(p_c1 - d.p, vec3_t::Zero(), p_md);
        u_accum_2 += clamp(p_c2 - p_dRp, vec3_t::Zero(), p_mdRp);
        const double tmp1 = d.p.transpose()*S.transpose()*p_c3;
        const double tmp2 = d.p.transpose()*S.transpose()*m.p;
        omega_accum_1 += clamp(tmp1, 0.0, tmp2);
        omega_accum_2 += clamp(psi_c - d.psi, 0.0, m.psi - d.psi);
    
        /* visualization stuff //{ */
    
        if (vis.visualize)
        {
          // Positional components
          {
            visualization_msgs::Marker p_c1d_vis = vector_vis(p_c1 - d.p, now, m_vis_p1_color);
            p_c1d_vis.id = 4*meas.detected.id;
            p_c1d_vis.ns = "p^c1 - p^d";
            
            visualization_msgs::Marker p_md_vis = vector_vis(p_md, now, m_vis_pmd_color);
            p_md_vis.id = 4*meas.detected.id + 1;
            p_md_vis.ns = "p^m - p^d";
            
            visualization_msgs::Marker p_c2dR_vis = vector_vis(p_c2 - p_dRp, now, m_vis_p2_color);
            p_c2dR_vis.id = 4*meas.detected.id + 2;
            p_c2dR_vis.ns = "p^c2 - p^dR'";
            
            visualization_msgs::Marker p_mdR_vis = vector_vis(p_mdRp, now, m_vis_pmdR_color);
            p_mdR_vis.id = 4*meas.detected.id + 3;
            p_mdR_vis.ns = "p^m - p^dR'";
            
            vis.p_cs.markers.push_back(p_c1d_vis);
            vis.p_cs.markers.push_back(p_md_vis);
            vis.p_cs.markers.push_back(p_c2dR_vis);
            vis.p_cs.markers.push_back(p_mdR_vis);
          }
    
          // Rotational components
          {
            visualization_msgs::Marker psi_cd_vis = heading_vis(2*(psi_c - d.psi), now, m_vis_psidc_color);
            psi_cd_vis.id = 4*meas.detected.id + 1;
            psi_cd_vis.ns = "2*(psi^c - psi^d)";
            
            visualization_msgs::Marker psi_cm_vis = heading_vis(2*(m.psi - d.psi), now, m_vis_psidm_color);
            psi_cm_vis.id = 4*meas.detected.id + 2;
            psi_cm_vis.ns = "2*(psi^m - psi^d)";
            
            visualization_msgs::Marker psi_bearing_restrained_vis = heading_vis(tmp1, now, m_vis_psi_bearing_r_color);
            psi_bearing_restrained_vis.id = 4*meas.detected.id + 3;
            psi_bearing_restrained_vis.ns = "p^d'*S'*p^c3";
            
            visualization_msgs::Marker psi_bearing_orig_vis = heading_vis(tmp2, now, m_vis_psi_bearing_o_color);
            psi_bearing_orig_vis.id = 4*meas.detected.id + 4;
            psi_bearing_orig_vis.ns = "p^d'*S'*p^m";
            
            vis.psi_cs.markers.push_back(psi_cd_vis);
            vis.psi_cs.markers.push_back(psi_cm_vis);
            vis.psi_cs.markers.push_back(psi_bearing_restrained_vis);
            vis.psi_cs.markers.push_back(psi_bearing_orig_vis);
          }

          // construction of p_c2
          {
            visualization_msgs::Marker p_dR_vis = vector_vis(p_dR, now, m_vis_p1_color);
            p_dR_vis.id = 8*meas.detected.id;
            p_dR_vis.ns = "p^dR";
            
            visualization_msgs::Marker p_dRp_vis = vector_vis(p_dRp, now, m_vis_p1_color);
            p_dRp_vis.id = 8*meas.detected.id + 1;
            p_dRp_vis.ns = "p^dR'";
            
            visualization_msgs::Marker p_m_vis = vector_vis(m.p, now, m_vis_p1_color);
            p_m_vis.id = 8*meas.detected.id + 2;
            p_m_vis.ns = "p^m";
            
            visualization_msgs::Marker p_sig_vis = heading_uncertainty_vis(p_dR, m.sig, now, m_vis_pmdR_color);
            p_sig_vis.id = 8*meas.detected.id + 3;
            p_sig_vis.ns = "sig_m";
            
            visualization_msgs::Marker p_Ct_vis = covariance_vis(p_dRp, C_t, now, m_vis_psidc_color);
            p_Ct_vis.id = 8*meas.detected.id + 4;
            p_Ct_vis.ns = "C_t";
            
            visualization_msgs::Marker p_Cm_vis = covariance_vis(m.p, m.C, now, m_vis_psidc_color);
            p_Cm_vis.id = 8*meas.detected.id + 5;
            p_Cm_vis.ns = "C_m";
            
            visualization_msgs::Marker p_Cc_vis = covariance_vis(m.p, C_c, now, m_vis_psidm_color);
            p_Cc_vis.id = 8*meas.detected.id + 6;
            p_Cc_vis.ns = "C_c";
            
            visualization_msgs::Marker p_c2_vis = vector_vis(p_c2, now, m_vis_p2_color);
            p_c2_vis.id = 8*meas.detected.id + 7;
            p_c2_vis.ns = "p^c2";
            
            vis.p_c2s.markers.push_back(p_dR_vis);
            vis.p_c2s.markers.push_back(p_dRp_vis);
            vis.p_c2s.markers.push_back(p_m_vis);
            vis.p_c2s.markers.push_back(p_sig_vis);
            vis.p_c2s.markers.push_back(p_Ct_vis);
            vis.p_c2s.markers.push_back(p_Cm_vis);
            vis.p_c2s.markers.push_back(p_Cc_vis);
            vis.p_c2s.markers.push_back(p_c2_vis);
          }
        }
    
        //}
      }
    
      const vec3_t u = k_e*(u_accum_1 + u_accum_2);
      const double omega = k_e*(omega_accum_1 + 2*omega_accum_2);
    
      ROS_INFO_STREAM_THROTTLE(m_throttle_period, "[FormationControl]: Rotation action components: heading: " << omega_accum_2 << ", bearing: " << omega_accum_1);
    
      return {u, omega};
    }
    //}

    /* difec() method //{ */
    std::pair<vec3_t, double> difec(const std::vector<agent_meas_t>& agent_meass, const ros::Time& now, const double fil_freq, visualization_t& vis)
    {
      // prepare the necessary constants
      const double k_e = m_drmgr_ptr->config.control__proportional_constant/fil_freq;
    
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
        const rads_t psi_md = m.psi - d.psi;
        // rotation matrix corresponding to the heading difference
        const mat3_t R_dpsi( anax_t(psi_md.value(), vec3_t::UnitZ()) );
        // position difference between measured and desired position
        const vec3_t p_md = m.p - d.p;
        // just a helper matrix (could be replaced with a cross product)
        const mat3_t S = skew_symmetric(vec3_t::UnitZ());

        ROS_INFO_STREAM("[FormationControl]: Target ID: " << m.id << " has heading sigma: " << m.sig << " rad");
    
        // | --------------------- calculate u_1 ---------------------- |
        // TODO: proper inverse calculation and checking
        const vec3_t u_1 = p_md;
    
        // | --------------------- calculate u_2 ---------------------- |
        const vec3_t p_dR = R_dpsi*d.p;
        const vec3_t u_2 = m.p - p_dR;
    
        // | -------------------- calculate psi_1 --------------------- |
        const rads_t psi_1 = double(d.p.transpose()*S.transpose()*m.p);
    
        // | -------------------- calculate psi_2 --------------------- |
        const rads_t psi_2 = psi_md;

        u_accum_1 += u_1;
        u_accum_2 += u_2;
        omega_accum_1 += psi_1.value();
        omega_accum_2 += psi_2.value();
    
        /* visualization stuff //{ */
    
        if (vis.visualize)
        {
          // Positional components
          visualization_msgs::Marker u_1_vis = vector_vis(u_1, now, m_vis_p1_color);
          u_1_vis.id = 4*meas.detected.id;
          u_1_vis.ns = "u1";
    
          visualization_msgs::Marker u_2_vis = vector_vis(u_2, now, m_vis_pmd_color);
          u_2_vis.id = 4*meas.detected.id + 1;
          u_2_vis.ns = "u2";
    
          vis.p_cs.markers.push_back(u_1_vis);
          vis.p_cs.markers.push_back(u_2_vis);
    
          // Rotational components
          visualization_msgs::Marker psi1_vis = heading_vis(psi_1.value(), now, m_vis_psidc_color);
          psi1_vis.id = 4*meas.detected.id + 2;
          psi1_vis.ns = "psi1";
    
          visualization_msgs::Marker psi2_vis = heading_vis(2*psi_1.value(), now, m_vis_psidm_color);
          psi2_vis.id = 4*meas.detected.id + 3;
          psi2_vis.ns = "psi2";
    
          vis.psi_cs.markers.push_back(psi1_vis);
          vis.psi_cs.markers.push_back(psi2_vis);
        }
    
        //}
      }
    
      const vec3_t u = k_e*(u_accum_1 + u_accum_2);
      const double omega = k_e*(omega_accum_1 + 2.0*omega_accum_2);
    
      ROS_INFO_STREAM_THROTTLE(m_throttle_period, "[FormationControl]: Rotation action components: heading: " << omega_accum_2 << ", bearing: " << omega_accum_1);
    
      return {u, omega};
    }
    //}

    /* velocity_command_loop() method //{ */
    void velocity_command_loop([[maybe_unused]] const ros::TimerEvent& evt)
    {
      std::scoped_lock lck(m_last_vel_out_mtx);
      if (m_drmgr_ptr->config.control__enabled && m_drmgr_ptr->config.control__repeat_commands && m_last_vel_out_set)
      {
        mrs_msgs::VelocityReferenceStamped vel_out = m_last_vel_out;
        vel_out.header.stamp = ros::Time::now();
        m_pub_vel_ref.publish(vel_out);
      }
    }
    //}

    /* dynrec_publish_loop() method //{ */
    void dynrec_publish_loop([[maybe_unused]] const ros::TimerEvent& evt)
    {
      // periodically publish the dynamic reconfigure descriptions and values for nimbro
      m_drmgr_ptr->publish_descriptions();
      m_drmgr_ptr->update_config(m_drmgr_ptr->config);
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

    /* envelope_circular_gaussian() method //{ */
    std::pair<vec3_t, mat3_t> envelope_circular_gaussian(const vec3_t& mean, const double sig, const double bulgarian = 1.0, const double eps = 1e-9)
    {
      // if the standard deviation is so large that the desired certainty covers at least half of the circle, clamp it
      // cases where the deviation is larger are equivalent
      const double arc_hang = std::min(bulgarian*sig, M_PI_2); // this is half the angle of the arc
      // helper variables
      const double c = std::cos(arc_hang);
      const double s = std::sin(arc_hang);
      // calculate the new mean of the enveloping distribution
      const vec3_t nmean = mean*c;
      const double r = mean.norm();
      // construct the enveloping covariance matrix (so far aligned with the axes)
      const mat3_t C_a = (r*r*vec3_t( (1.0 - c)*(1.0 - c), s*s, eps )).asDiagonal();
      // a rotation matrix to rotate the ellipse so that it's correctly aligned with the original distribution
      const mat3_t R( anax_t(std::atan2(mean.y(), mean.x()), vec3_t::UnitZ()) );
      // now rotate the matrix to align it with the original distribution
      const mat3_t C = mrs_lib::geometry::rotateCovariance(C_a, R);
      return {nmean, C};
    }
    //}

    /* project_gaussian_to_line() method //{ */
    std::pair<double, double> project_gaussian_to_line(const vec3_t& mean, const mat3_t& C, const vec3_t& line_orig, const vec3_t& line_dir)
    {
      const vec3_t nmean = mean - line_orig;
      const vec3_t ndir = line_dir.normalized();
      const double sig = sqrt(ndir.transpose() * C * ndir);
      const double mu = ndir.transpose() * nmean;
      return {mu, sig};
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
    visualization_msgs::Marker heading_vis(const double hdg, const ros::Time& time, const std_msgs::ColorRGBA& color) const
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
      pnt.z = hdg;
      mkr.points.push_back(pnt);
      return mkr;
    }
    //}

    /* formation_vis() method //{ */
    visualization_msgs::MarkerArray formation_vis(const std::vector<agent_t>& formation, const ros::Time& time) const
    {
      visualization_msgs::MarkerArray ret;
      if (formation.empty())
        return ret;

      /* arrows //{ */

      for (const auto& agent : formation)
      {
        visualization_msgs::Marker arrow;
        
        arrow.id = agent.id;
        arrow.ns = agent.uav_name;
        arrow.header.frame_id = m_uav_frame_id;
        arrow.header.stamp = time;
        arrow.frame_locked = true;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.pose.orientation.w = 1.0;
        arrow.scale.x = 0.05; // shaft diameter
        arrow.scale.y = 0.15; // head diameter
        arrow.color.r = 1.0;
        arrow.color.g = 1.0;
        arrow.color.a = 1.0;
        
        const vec3_t& des_pos = agent.desired_relative_pose.p;
        arrow.points.push_back(mrs_lib::geometry::fromEigen(des_pos));
        
        const vec3_t des_pos_ori = des_pos + anax_t(agent.desired_relative_pose.psi.value(), vec3_t::UnitZ())*vec3_t::UnitX();
        arrow.points.push_back(mrs_lib::geometry::fromEigen(des_pos_ori));
        
        ret.markers.push_back(arrow);
      }
        
      //}

      /* lines //{ */
      visualization_msgs::Marker lines;
      lines.id = -1;
      lines.ns = "formation";
      lines.header.frame_id = m_uav_frame_id;
      lines.header.stamp = time;
      lines.frame_locked = true;
      lines.type = visualization_msgs::Marker::LINE_STRIP;
      lines.pose.orientation.w = 1.0;
      lines.scale.x = 0.03; // line segment width
      lines.color.r = 1.0;
      lines.color.g = 1.0;
      lines.color.a = 1.0;

      // add the positions of the agents
      for (const auto& agent : formation)
        lines.points.push_back(mrs_lib::geometry::fromEigen(agent.desired_relative_pose.p));
      lines.points.push_back(mrs_lib::geometry::fromEigen(formation.front().desired_relative_pose.p));

      ret.markers.push_back(lines);
      
      //}

      return ret;
    }
    //}

    template <typename T>
    T normal_pdf(const T x, const T mu, const T sig)
    {
      static const T inv_sqrt_2pi = 0.3989422804014327;
      const T a = (x - mu) / sig;
      return inv_sqrt_2pi / sig * std::exp(-T(0.5) * a * a);
    }

    /* heading_uncertainty_vis() method //{ */
    visualization_msgs::Marker heading_uncertainty_vis(const vec3_t& mean, const double sig, const ros::Time& time, const std_msgs::ColorRGBA& color)
    {
      visualization_msgs::Marker mkr;
      mkr.header.frame_id = m_uav_frame_id;
      mkr.header.stamp = time;
      mkr.type = visualization_msgs::Marker::LINE_LIST;
      mkr.pose.orientation.w = 1.0;
      mkr.scale.x = 0.05; // line width
      std_msgs::ColorRGBA col = color;
    
      const double r = mean.head<2>().norm();
      const double ang_center = std::atan2(mean.y(), mean.x());
      const double pdf_norm = normal_pdf(0.0, 0.0, sig);
    
      constexpr int N = 10;
      const double ang_hrange = 3*sig;
      const double ang_step = ang_hrange/N;
      for (int it = -N; it < N; it++)
      {
        const double ang = it*ang_step + ang_center;
        const double ang1 = ang - ang_step/2.0;
        const double ang2 = ang + ang_step/2.0;
        const double intensity = normal_pdf(ang, ang_center, sig)/pdf_norm;
        geometry_msgs::Point pnt;
    
        pnt.x = r*std::cos(ang1);
        pnt.y = r*std::sin(ang1);
        pnt.z = mean.z();
        mkr.points.push_back(pnt);
        col.a = intensity;
        mkr.colors.push_back(col);
    
        pnt.x = r*std::cos(ang2);
        pnt.y = r*std::sin(ang2);
        pnt.z = mean.z();
        mkr.points.push_back(pnt);
        col.a = intensity;
        mkr.colors.push_back(col);
    
        pnt.x = r*std::cos(ang1);
        pnt.y = r*std::sin(ang1);
        pnt.z = mean.z() + normal_pdf(ang1, ang_center, sig)/pdf_norm;
        mkr.points.push_back(pnt);
        col.a = intensity;
        mkr.colors.push_back(col);
    
        pnt.x = r*std::cos(ang2);
        pnt.y = r*std::sin(ang2);
        pnt.z = mean.z() + normal_pdf(ang2, ang_center, sig)/pdf_norm;
        mkr.points.push_back(pnt);
        col.a = intensity;
        mkr.colors.push_back(col);
      }

      return mkr;
    }
    //}

    // Local function to force the axis to be right handed for 3D.
    // Taken from https://github.com/ros-visualization/rviz/blob/1f622b8c95b8e188841b5505db2f97394d3e9c6c/src/rviz/default_plugin/covariance_visual.cpp#L100
    std::pair<vec3_t, mat3_t> make_right_handed(const vec3_t& eigenvalues, const mat3_t& eigenvectors)
    {
      // Note that sorting of eigenvalues may end up with left-hand coordinate system.
      // So here we correctly sort it so that it does end up being right-handed and normalised.
      const vec3_t c0 = eigenvectors.col(0).normalized();
      const vec3_t c1 = eigenvectors.col(1).normalized();
      const vec3_t c2 = eigenvectors.col(2).normalized();
      const vec3_t cc = c0.cross(c1);
      if (cc.dot(c2) < 0)
      {
        const mat3_t rh_evecs( (mat3_t() << c1, c0, c2).finished() );
        const vec3_t rh_evals(eigenvalues.y(), eigenvalues.x(), eigenvalues.z());
        return {rh_evals, rh_evecs};
      }
      else
      {
        const mat3_t rh_evecs( (mat3_t() << c0, c1, c2).finished() );
        return {eigenvalues, rh_evecs};
      }
    }

    /* covariance_vis() method //{ */
    visualization_msgs::Marker covariance_vis(const vec3_t& mean, const mat3_t& cov, const ros::Time& time, const std_msgs::ColorRGBA& color)
    {
      visualization_msgs::Marker mkr;
      mkr.header.frame_id = m_uav_frame_id;
      mkr.header.stamp = time;
      mkr.type = visualization_msgs::Marker::SPHERE;
      mkr.color = color;
      mkr.color.a = 0.2;
    
      Eigen::SelfAdjointEigenSolver<mat3_t> esol(cov);
      // check success of the solver
      if (esol.info() != Eigen::Success)
      {
        ROS_WARN_THROTTLE(1, "Failed to compute eigen vectors/values for position. Is the covariance matrix correct?");
        return mkr;
      }

      const vec3_t evals_raw = esol.eigenvalues();
      const mat3_t evecs_raw = esol.eigenvectors();
      const auto [evals, evecs] = make_right_handed(evals_raw, evecs_raw);

      const quat_t rot(evecs);
      mkr.pose.position = mrs_lib::geometry::fromEigen(mean);
      mkr.pose.orientation = mrs_lib::geometry::fromEigen(rot);
      // according to the Rviz implementation, this should be doubled for some reason...
      // https://github.com/ros-visualization/rviz/blob/1f622b8c95b8e188841b5505db2f97394d3e9c6c/src/rviz/default_plugin/covariance_visual.cpp#L133
      mkr.scale.x = 2.0 * std::sqrt(evals.x());
      mkr.scale.y = 2.0 * std::sqrt(evals.y());
      mkr.scale.z = 2.0 * std::sqrt(evals.z());

      return mkr;
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
    std::optional<std::vector<agent_t>> load_formation(const std::string& this_uav_name, const std::string& filename)
    {
      ROS_INFO_STREAM("[FormationControl]: Parsing formation from file '" << filename << "'."); 
      std::ifstream fs(filename);
      if (!fs.is_open())
      {
        ROS_ERROR_STREAM("[FormationControl]: Couldn't open the formation file!");
        return std::nullopt;
      }
      // ignore the first line that only contains the csv columns description
      std::string line;
      std::getline(fs, line);

      bool all_ok = true;
      std::vector<agent_t> formation;
      while (std::getline(fs, line))
      {
        // Ignore empty lines
        if (line.empty())
          continue;

        // Tokenize the line
        boost::trim(line);
        /* const std::vector<std::string> st = split(line, ' '); */
        std::vector<std::string> st;
        boost::split(st, line, boost::is_any_of(",;"), boost::token_compress_on);

        if (st.size() < 5)
        {
          ROS_WARN_STREAM("[FormationControl]: Read line with a wrong number of elements: " << st.size() << " (expected exactly 5). The line: '" << line << "'."); 
          continue;
        }

        size_t it = 0;
        const std::string uav_name = st.at(it++);
        ROS_INFO_STREAM("[FormationControl]: Loading formation data for UAV " << uav_name);
        try
        {
          // try to parse the expected values
          const uint64_t id = std::stoul(st.at(it++));
          const std::array<double, 3> coords = {std::stod(st.at(it++)), std::stod(st.at(it++)), std::stod(st.at(it++))};
          const rads_t heading = std::stod(st.at(it++));

          // create the agent and add it to the formation
          const vec3_t position(coords[0], coords[1], coords[2]);
          const pose_t agent_pose = {position, heading};
          const agent_t agent = {id, uav_name, agent_pose};
          formation.emplace_back(agent);
        }
        catch (const std::exception& e)
        {
          ROS_WARN_STREAM("[FormationControl]: Couldn't load all necessary formation information about UAV " << uav_name << ", ignoring. Expected fields:\nuav_name (string), uvdar_id (int), position_x (double), position_y (double), position_z (double), heading (double)\nThe line:\n'" << line << "'.");
          all_ok = false;
          continue;
        }
      }

      // find this UAV in the formation specification
      const auto this_agent_it = std::find_if(std::begin(formation), std::end(formation), [&this_uav_name](const auto& agent){return agent.uav_name == this_uav_name;});
      if (this_agent_it == std::end(formation))
      {
        ROS_ERROR_STREAM("[FormationControl]: The current UAV " << this_uav_name << " not found in the specified formation!");
        return std::nullopt;
      }
      const agent_t& this_agent = *this_agent_it;
      m_uvdar_id = this_agent.id;
      const pose_t this_pose = this_agent.desired_relative_pose;

      // now recalculate the agent poses to be relative to the current UAV
      const mat3_t rot( anax_t(-this_pose.psi.value(), vec3_t::UnitZ()) );
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

    /* dynparam_callback() method //{ */
    void dynparam_callback(difec_ron::FormationControlParamsConfig& new_config, uint32_t level)
    {
      // signals that the formation filename was changed
      if (level == 8 && new_config.formation__update)
      {
        std::scoped_lock lck(m_mutex);
        const auto formation_opt = load_formation(m_uav_name, new_config.formation__filename);
        // CHECK LOADING STATUS
        if (!formation_opt.has_value())
          NODELET_ERROR("Failed to load the new formation! Ignoring.");
        else
          m_formation = formation_opt.value();

        m_pub_vis_formation.publish(formation_vis(m_formation, ros::Time::now()));
      }
    }
    //}

  private:

    std::mutex m_mutex;
    std::thread m_main_thread;
    uint64_t m_uvdar_id;
    boost::circular_buffer<double> m_period_buffer;
    std_msgs::ColorRGBA m_vis_u_color;
    std_msgs::ColorRGBA m_vis_p1_color;
    std_msgs::ColorRGBA m_vis_pmd_color;
    std_msgs::ColorRGBA m_vis_p2_color;
    std_msgs::ColorRGBA m_vis_pmdR_color;

    std_msgs::ColorRGBA m_vis_omega_color;
    std_msgs::ColorRGBA  m_vis_psidc_color;
    std_msgs::ColorRGBA  m_vis_psidm_color;
    std_msgs::ColorRGBA  m_vis_psi_bearing_r_color;
    std_msgs::ColorRGBA  m_vis_psi_bearing_o_color;

    std::mutex m_last_vel_out_mtx;
    bool m_last_vel_out_set = false;
    mrs_msgs::VelocityReferenceStamped m_last_vel_out;

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
    ros::Publisher m_pub_vis_psi_cs;
    ros::Publisher m_pub_vis_p_c1s;
    ros::Publisher m_pub_vis_u;
    ros::Publisher m_pub_vis_omega;
    ros::Publisher m_pub_vel_ref;

    ros::Timer m_tim_vel_cmd;
    ros::Timer m_tim_pub_dynrec;

    //}

  private:
    // --------------------------------------------------------------
    // |                 Parameters, loaded from ROS                |
    // --------------------------------------------------------------

    /* Parameters, loaded from ROS //{ */

    std::string m_uav_name;
    std::string m_world_frame_id;
    std::string m_uav_frame_id;
    ros::Duration m_transform_lookup_timeout;
    double m_throttle_period;

    std::vector<agent_t> m_formation;

    //}

  };  // class FormationControl
};    // namespace difec_ron

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(difec_ron::FormationControl, nodelet::Nodelet)
