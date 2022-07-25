#include "main.h"
#include <difec_ron/FormationState.h>
#include <dynamic_reconfigure/Config.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen_conversions/eigen_msg.h>


namespace e = Eigen;

namespace difec_ron {


  struct pose_t
  {
    vec3_t p; // position
    rads_t psi; // heading
  };

  struct agent_t
  {
    uint64_t id; // ID of this agent
    std::string uav_name;
    pose_t pose; // the desired pose in the formation relative to the current agent
  };

  using rads_t = mrs_lib::geometry::sradians;
  using vec3_t = Eigen::Vector3d;

  class FormationErrorCalculator {
    public: 
      FormationErrorCalculator(ros::NodeHandle& nh) {
        ROS_INFO("[FormationErrorCalculator]: Initializing formation error calculator...");

        mrs_lib::ParamLoader param_loader(nh, "FormationErrorCalculator");

        param_loader.loadParam("common_frame", _common_frame_, std::string("global_gps"));

        /* param_loader.loadParam("original_username", _original_username_, std::string("mrs")); */
        /* param_loader.loadParam("local_username", _local_username_); */
        /* param_loader.loadParam("formation_file_location", _formation_file_location_); */
        std::string initial_formation_file;
        param_loader.loadParam("initial_formation_file", initial_formation_file);
        /* initial_formation_file = _formation_file_location_ + "/" + initial_formation_file; */

        param_loader.loadParam("uav_list", _uav_list_, _uav_list_);

        param_loader.loadParam("process_rate", _process_rate_, double(20.0));

        if (_uav_list_.empty()) {
          ROS_ERROR("[FormationErrorCalculator]: No uav names were supplied. Returning.");
          ros::shutdown();
          return;
        }

        auto initial_formation = load_formation(initial_formation_file);
        if (!initial_formation){
          ROS_ERROR("[FormationErrorCalculator]: Failed to pre-laod initial formation file. Returning.");
          ros::shutdown();
          return;
        }
        int i = 0;
        for (auto uav_name : _uav_list_){
          agent_t agent_curr;
          agent_curr.id = getUavIDFromName(initial_formation.value(), uav_name);
          agent_curr.uav_name = uav_name;
          agent_curr.pose = {e::Vector3d(0,0,0),mrs_lib::geometry::sradians(0)};
          formation_curr_.push_back(agent_curr);
          formation_desired_.push_back(agent_curr);
          i++;
        }

        param_loader.loadParam("formation_controller_node", _formation_controler_node_, std::string("swarm_control"));

        mrs_lib::SubscribeHandlerOptions shopts(nh);
        shopts.no_message_timeout = ros::Duration(5.0);
        shopts.use_thread_timer = true;

        mrs_lib::construct_object(m_sh_formation_control_updates, shopts, "/"+_uav_list_.at(0)+"/"+_formation_controler_node_+"/parameter_updates");

        mrs_lib::construct_object(m_sh_formation_visualization, shopts, "/"+_uav_list_.at(0)+"/"+_formation_controler_node_+"/visualization_formation");
        

        acceleration_pos_ = e::VectorXd(_uav_list_.size());
        acceleration_rot_ = e::VectorXd(_uav_list_.size());
        velocity_ang_ = e::VectorXd(_uav_list_.size());
        int u = 0;
        for (auto uav : _uav_list_){
          m_sh_odometry_updates.push_back(mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "/"+uav+"/odometry/uav_state"));
          acceleration_pos_(u) =  std::nan("1");
          acceleration_rot_(u) =  std::nan("1");
          velocity_ang_(u) =  std::nan("1");

          m_sh_relative_localization.push_back(mrs_lib::SubscribeHandler<mrs_msgs::PoseWithCovarianceArrayStamped>(shopts, "/"+uav+"/uvdar/filteredPoses"));
          rel_localizations_.push_back(mrs_msgs::PoseWithCovarianceArrayStamped());
          u++;
        }
        
        laplacian_matrix_ = e::MatrixXd::Zero(_uav_list_.size(),_uav_list_.size());



        m_transformer = mrs_lib::Transformer(nh,"FormationErrorCalculator");
        m_transformer.retryLookupNewest(true);

        pub_formation_error_ = nh.advertise<difec_ron::FormationState>("formation_error", 1);

        timer_main_ = nh.createTimer(ros::Rate(_process_rate_), &FormationErrorCalculator::MainTimer, this, false);
      }

      void MainTimer([[maybe_unused]] const ros::TimerEvent& te)
      {
        ros::Time now = ros::Time::now();
        const ros::WallDuration timeout(1.0/10.0);

        int i = 0;
        for (auto uav_name : _uav_list_){
          const auto tf_opt = m_transformer.getTransform(uav_name+"/fcu", _common_frame_, now);
          if (!tf_opt.has_value())
          {
            ROS_ERROR_STREAM_THROTTLE(1, "[FormationControl]: Could not lookup transformation from \"" << uav_name+"/fcu" << "\" to \"" << _common_frame_ );
            return;
          }
          const auto tf_uav = tf_opt.value();
          const Eigen::Isometry3d tf = tf2::transformToEigen(tf_uav);
          const mat3_t tf_rot = tf.rotation();
          const rads_t tf_hdg = mrs_lib::geometry::headingFromRot(tf_rot);
          const vec3_t p = tf*e::Vector3d(0,0,0);
          /* const vec3_t p = tf.translation(); */
          formation_curr_.at(i).pose = {p,tf_hdg};
          formation_curr_.at(i).uav_name = uav_name;

          /* ROS_INFO_STREAM("[FormationControl]: For " << uav_name << "(" << i << ") the pose is: p: " << p.transpose() << ", psi: " << tf_hdg); */

          i++;
        }

        int u = 0;
        for (auto uav_name : _uav_list_){
          const auto msg_ptr = m_sh_odometry_updates.at(u).waitForNew(timeout);
          if (msg_ptr){
            e::Vector3d accel_lin;
            e::Vector3d accel_rot;
            e::Vector3d veloc_ang;
            tf::vectorMsgToEigen(msg_ptr->acceleration.linear,accel_lin);
            tf::vectorMsgToEigen(msg_ptr->acceleration.angular,accel_rot);
            tf::vectorMsgToEigen(msg_ptr->velocity.angular,veloc_ang);
            acceleration_pos_(u) = accel_lin.norm();
            /* ROS_INFO_STREAM("[FormationControl]: " << accel_rot.transpose()); */
            acceleration_rot_(u) = e::AngleAxisd(
                e::AngleAxisd(accel_rot.x(), e::Vector3d::UnitX())
              * e::AngleAxisd(accel_rot.y(), e::Vector3d::UnitY())
              * e::AngleAxisd(accel_rot.z(), e::Vector3d::UnitZ())).angle();
            velocity_ang_(u) = e::AngleAxisd(
                e::AngleAxisd(veloc_ang.x(), e::Vector3d::UnitX())
              * e::AngleAxisd(veloc_ang.y(), e::Vector3d::UnitY())
              * e::AngleAxisd(veloc_ang.z(), e::Vector3d::UnitZ())).angle();
          }
          else if (isnan(acceleration_pos_(u)) || isnan(acceleration_rot_(u))) {
            ROS_ERROR_STREAM_THROTTLE(1, "[FormationControl]: Could not retrieve acceleration of " << uav_name << ", nor was it retrieved before. Returning.");
            return;
          }
          u++;
        }

        formation_curr_relative_ = formationRelative(formation_curr_);


        const auto msg_ptr = m_sh_formation_control_updates.waitForNew(timeout);

        /* double fiedler; */
        if (msg_ptr){

          std::string orig_filename = msg_ptr->strs[0].value;
          std::string base_filename = orig_filename.substr(orig_filename.find_last_of("/\\") + 1);
          /* size_t index = 0; */
          /* index = filename.find(_original_username_, index); */
          /* if (index == std::string::npos){ */
          /*   ROS_ERROR_STREAM_THROTTLE(1, "[FormationControl]: Could not replace username in file " << msg_ptr->strs[0].value << ". Returning."); */
          /*   return; */
          /* }; */
          /* filename.replace(index, _original_username_.length(), _local_username_); */
          /* std::string filename = _formation_file_location_+"/"+base_filename; */
          std::string filename = base_filename;

          if (filename != last_formation_file_){
            /* auto opt = load_formation(filename); */
            ROS_INFO_STREAM("[FormationControl]: Retrieving formation for file '" << filename << "'."); 
            auto opt = retrieve_current_formation();
            if (opt.has_value()){
              formation_desired_ = opt.value();
              last_formation_file_ = filename;
              /* ROS_INFO_STREAM("[FormationControl]: des. relative"); */ 
              formation_desired_relative_ = formationRelative(formation_desired_);
            }
            else {
              ROS_ERROR_STREAM_THROTTLE(1, "[FormationControl]: Could not retrieve formation corresponding to file " << filename << ". Returning.");
              return;
            }
          }

          laplacian_matrix_ = e::MatrixXd::Zero(_uav_list_.size(),_uav_list_.size());
          /* e::VectorXd degree_matrix = e::VectorXd::Zero(_uav_list_.size()) */
          int v = 0;
          for (auto uav_name : _uav_list_){
            /* ROS_INFO_STREAM("[FormationControl]: v: " << v); */ 
            /* ROS_INFO_STREAM("[FormationControl]: uav_name " << uav_name); */ 
            const auto msg_ptr = m_sh_relative_localization.at(v).waitForNew(ros::WallDuration(timeout));
            if (msg_ptr){
              rel_localizations_.at(v) = *msg_ptr;
            }
            /* ROS_INFO_STREAM("[FormationControl]: obs. count: " << rel_localizations_.at(v).poses.size()); */ 
            for (const auto rl : rel_localizations_.at(v).poses){
              int index = getUavIndexFromID(formation_desired_,rl.id);
              if ((index >= 0) && (index != v)){
                laplacian_matrix_(v,index) = -1;
                laplacian_matrix_(index,v) = -1;
                /* laplacian_matrix_(index,v) = -1; */
                /* laplacian_matrix_(index,index) +=1; */
              }
              }
            v++;
        }
          for (int y = 0; y < (int)(_uav_list_.size()); y++){
            for (int z = 0; z < (int)(_uav_list_.size()); z++){
              if (y != z){
                laplacian_matrix_(y,y) -= laplacian_matrix_(y,z);
              }
            }
          }

          //Get the Fiedler eigenvalue
          e::VectorXd eigs = laplacian_matrix_.eigenvalues().real();
          /* ROS_INFO_STREAM("[FormationControl]: eigs: " << eigs.transpose()); */ 
          double smallest = std::nan("1");
          fiedler_ = std::nan("1"); //whatever
          for (int i = 0; i< (int)(_uav_list_.size()); i++){
            double candidate = eigs(i);
            if ((candidate < (smallest+0.1)) || (std::isnan(smallest))){
              fiedler_ = smallest;
              smallest = candidate;
            }
            else if ((fabs(candidate-smallest) > 0.1) && ((candidate < (fiedler_+0.1)) || (std::isnan(fiedler_)))){
              fiedler_ = candidate;
            }
            /* ROS_INFO_STREAM("[FormationControl]: smallest: \n" << smallest); */ 
            /* ROS_INFO_STREAM("[FormationControl]: fiedler: \n" << fiedler_); */ 
          }
          /* connected_ = (fiedler > 0.0); */


          restraining_factor_ = msg_ptr->doubles[0].value;
          proportional_constant_ = msg_ptr->doubles[1].value;

          restraining_enabled_ = msg_ptr->bools[1].value;
          control_enabled_ = msg_ptr->bools[0].value;
        }
        else if (formation_desired_.size() == 0) {
          return;
        }


        if (formation_curr_.size() != formation_desired_.size()){
          ROS_ERROR_STREAM_THROTTLE(1, "[FormationControl]: Desired formation of size " << formation_desired_.size() << " does not match the current formation of size " << formation_curr_.size() << ". Returning.");
          return;
        }

        auto [formation_error, position_mean_error, orientation_mean_error] = formationError(formation_curr_relative_, formation_desired_relative_);

        difec_ron::FormationState msg_pub;
        msg_pub.header.stamp = now;
        msg_pub.formation_file.data = last_formation_file_;
        msg_pub.formation_error.data = formation_error;
        msg_pub.position_mean_error.data = position_mean_error;
        msg_pub.orientation_mean_error.data = orientation_mean_error;
        msg_pub.proportional_kef.data = proportional_constant_;
        msg_pub.restraining_l.data = restraining_factor_;
        msg_pub.restraining_enabled.data = restraining_enabled_;
        msg_pub.control_enabled.data = control_enabled_;
        msg_pub.velocity_angular.data = velocity_ang_.mean();
        msg_pub.acceleration_position.data = acceleration_pos_.mean();
        msg_pub.acceleration_rotation.data = acceleration_rot_.mean();
        /* ROS_INFO_STREAM("[FormationControl]: conn. matrix: \n" << laplacian_matrix_); */ 
        /* msg_pub.graph_connected.data = connected_; //one is True */
        msg_pub.fiedler_eigenvalue.data = fiedler_; //one is True
        pub_formation_error_.publish(msg_pub);

    }

    private: 

      std::vector<pose_t> formationRelative(std::vector<agent_t> formation_absolute){
        std::vector<pose_t> output;
        for (int i = 0; i<(int)(formation_absolute.size()); i++){
          /* ROS_INFO_STREAM("[FormationControl]: i:" << i << ": " << formation_absolute.at(i).pose.p.transpose() << ", - " << formation_absolute.at(i).pose.psi); */ 
          const mat3_t R_psiiT( anax_t(-(formation_absolute.at(i).pose.psi.value()), vec3_t::UnitZ()) );
          for (int j = 0; j<(int)(formation_absolute.size()); j++){
          /* ROS_INFO_STREAM("[FormationControl]: j:" << i << ": " << formation_absolute.at(j).pose.p.transpose() << ", - " << formation_absolute.at(j).pose.psi); */ 
            if (i != j){
              auto p_diff = R_psiiT*(formation_absolute.at(j).pose.p-formation_absolute.at(i).pose.p);
              auto psi_diff = formation_absolute.at(j).pose.psi-formation_absolute.at(i).pose.psi;
              /* ROS_INFO_STREAM("[FormationControl]: diff:" << p_diff.transpose() << ", - " << psi_diff); */ 
              pose_t pose_diff = {p_diff, psi_diff};
              output.push_back(pose_diff);
            }
          }
        }

        return output;
      }

      std::tuple<double,double,double> formationError(std::vector<pose_t> curr, std::vector<pose_t> desired){
        e::VectorXd formation_diff_vector(curr.size()*4);
        e::VectorXd position_diff_vector(curr.size()*1);
        e::VectorXd orientation_diff_vector(curr.size()*1);

        for ( int i = 0; i < (int)(curr.size()); i++){
          auto curr_p_diff = desired.at(i).p - curr.at(i).p;
          auto curr_psi_diff = rads_t::dist(desired.at(i).psi,curr.at(i).psi);
      /* ROS_INFO_STREAM("[FormationControl]: des: \n" << desired.at(i).p.transpose() << ", " << desired.at(i).psi); */ 
      /* ROS_INFO_STREAM("[FormationControl]: cur: \n" << curr.at(i).p.transpose() << ", " << curr.at(i).psi); */ 
          formation_diff_vector(i*4 + 0) = curr_p_diff.x();
          formation_diff_vector(i*4 + 1) = curr_p_diff.y();
          formation_diff_vector(i*4 + 2) = curr_p_diff.z();
          formation_diff_vector(i*4 + 3) = curr_psi_diff;

          position_diff_vector(i*1 + 0) = curr_p_diff.norm();
          /* position_diff_vector(i*3 + 1) = curr_p_diff.y(); */
          /* position_diff_vector(i*3 + 2) = curr_p_diff.z(); */

          orientation_diff_vector(i*1 + 0) = curr_psi_diff;
        }

      /* ROS_INFO_STREAM("[FormationControl]: Error function: \n" << formation_diff_vector.norm()); */ 
      /* ROS_INFO_STREAM("[FormationControl]: Partial position errors: \n" << position_diff_vector.transpose()); */ 


        return {formation_diff_vector.norm(),position_diff_vector.mean(),orientation_diff_vector.mean()};
      }

      int getUavIndexFromID(std::vector<agent_t> formation, int ID){
        int result = -1;

        for (auto ag : formation){
          /* ROS_INFO_STREAM("[FormationControl]: ag.id: " << ag.id); */ 
          /* ROS_INFO_STREAM("[FormationControl]: ag.uav_name: " << ag.uav_name); */ 
          if (ag.id == (uint64_t)ID){
            int j=0;
            for (auto uav : _uav_list_){
              if (ag.uav_name == uav){
                result = j;
                break;
              }
              j++;
            }

            if (result != -1){
              break;
            }
          }
        }

        return result;
      }

      int getUavIDFromName(std::vector<agent_t> formation, std::string name){
        int result = -1;

        for (auto ag : formation){
          /* ROS_INFO_STREAM("[FormationControl]: ag.id: " << ag.id); */ 
          /* ROS_INFO_STREAM("[FormationControl]: ag.uav_name: " << ag.uav_name); */ 
          if (ag.uav_name == name){
            return ag.id;
          }
        }

        return result;
      }

      std::optional<std::vector<agent_t>> load_formation(const std::string& filename)
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
            const uint64_t id = std::stoul(st.at(it++));
            // try to parse the expected values
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

        if (all_ok)
          return formation;
        else
          return std::nullopt;
      }

      std::optional<std::vector<agent_t>> retrieve_current_formation()
      {
        const ros::WallDuration timeout(2.0/10.0);
        const auto msg_ptr = m_sh_formation_visualization.waitForNew(timeout);
        
        if (msg_ptr){
          std::vector<agent_t> formation;
          for (const auto& m : msg_ptr->markers){
            if (m.id >= 0) {
              if (m.points.size() < 2){
                return std::nullopt;
              }
              const vec3_t position(m.points[0].x, m.points[0].y, m.points[0].z);
              const rads_t heading = atan2(m.points[1].y - m.points[0].y,m.points[1].x - m.points[0].x);
              const pose_t agent_pose = {position, heading};
              const agent_t agent = {(uint64_t)(m.id), m.ns, agent_pose};
              ROS_INFO_STREAM("[FormationControl]: adding p: " << position.transpose() << ", h: " << heading.value()); 
              formation.push_back(agent);
            }
          }
          return formation;
        }

        return std::nullopt;
      }

      mrs_lib::Transformer m_transformer;
      ros::Publisher pub_formation_error_;
      ros::Timer timer_main_;

      double _process_rate_;

      std::string _common_frame_;
      std::vector<std::string> _uav_list_;
      std::string _formation_controler_node_;

      /* std::string _original_username_; */
      /* std::string _formation_file_location_; */

      std::vector<agent_t> formation_curr_;
      std::vector<pose_t> formation_curr_relative_;

      mrs_lib::SubscribeHandler<dynamic_reconfigure::Config> m_sh_formation_control_updates;
      std::vector<mrs_lib::SubscribeHandler<mrs_msgs::UavState>> m_sh_odometry_updates;
      std::vector<mrs_lib::SubscribeHandler<mrs_msgs::PoseWithCovarianceArrayStamped>> m_sh_relative_localization;
      mrs_lib::SubscribeHandler<visualization_msgs::MarkerArray> m_sh_formation_visualization;
      std::vector<mrs_msgs::PoseWithCovarianceArrayStamped> rel_localizations_;

      bool control_enables_ = false;
      bool use_noise_ = false;
      std::string last_formation_file_;
      std::vector<agent_t> formation_desired_;
      std::vector<pose_t> formation_desired_relative_;

      double restraining_factor_, proportional_constant_;
      bool restraining_enabled_, control_enabled_;
      e::VectorXd acceleration_pos_, acceleration_rot_, velocity_ang_;
      e::MatrixXd laplacian_matrix_; //laplacian matrix
      /* bool connected_; */
      double fiedler_;

  };
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "formation_error_calculator");

  ros::NodeHandle nh("~");
  difec_ron::FormationErrorCalculator dfecl(nh);
  ROS_INFO("[FormationErrorCalculator]: Formation error calculator node initiated");
  ros::spin();
  return 0;
}
