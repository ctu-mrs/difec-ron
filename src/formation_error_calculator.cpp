#include "main.h"
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/Config.h>


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

        param_loader.loadParam("original_username", _original_username_, std::string("mrs"));
        param_loader.loadParam("local_username", _local_username_);

        param_loader.loadParam("uav_list", _uav_list_, _uav_list_);
        if (_uav_list_.empty()) {
          ROS_ERROR("[FormationErrorCalculator]: No uav names were supplied. Returning.");
          ros::shutdown();
          return;
        }

        int i = 0;
        for (auto uav_name : _uav_list_){
          agent_t agent_curr;
          agent_curr.id = i++;
          agent_curr.uav_name = uav_name;
          agent_curr.pose = {e::Vector3d(0,0,0),mrs_lib::geometry::sradians(0)};
          formation_curr_.push_back(agent_curr);
          formation_desired_.push_back(agent_curr);
        }

        param_loader.loadParam("formation_controller_node", _formation_controler_node_, std::string("swarm_control"));

        mrs_lib::SubscribeHandlerOptions shopts(nh);
        shopts.no_message_timeout = ros::Duration(5.0);

        mrs_lib::construct_object(m_sh_formation_control_updates, shopts, "/"+_uav_list_.at(0)+"/"+_formation_controler_node_+"/parameter_updates");



        m_transformer = mrs_lib::Transformer(nh,"FormationErrorCalculator");
        m_transformer.retryLookupNewest(true);

        pub_formation_error_ = nh.advertise<std_msgs::Float64>("formation_error", 1);

        timer_main_ = nh.createTimer(ros::Rate(20.0), &FormationErrorCalculator::MainTimer, this, false);
      }

      void MainTimer([[maybe_unused]] const ros::TimerEvent& te)
      {
        ros::Time now = ros::Time::now();
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

        formation_curr_relative_ = formationRelative(formation_curr_);


        const ros::WallDuration timeout(1.0/10.0);
        const auto msg_ptr = m_sh_formation_control_updates.waitForNew(timeout);
        if (msg_ptr){

          std::string filename = msg_ptr->strs[0].value;
          size_t index = 0;
          index = filename.find(_original_username_, index);
          if (index == std::string::npos){
            ROS_ERROR_STREAM_THROTTLE(1, "[FormationControl]: Could not replace username in file " << msg_ptr->strs[0].value << ". Returning.");
            return;
          };
          filename.replace(index, _original_username_.length(), _local_username_);

          if (filename != last_formation_file_){
            auto opt = load_formation(filename);
            if (opt.has_value()){
              formation_desired_ = opt.value();
              last_formation_file_ = filename;
              /* ROS_INFO_STREAM("[FormationControl]: des. relative"); */ 
              formation_desired_relative_ = formationRelative(formation_desired_);
            }
            else {
              ROS_ERROR_STREAM_THROTTLE(1, "[FormationControl]: Could not load formation file " << filename << ". Returning.");
              return;
            }
          }
        }
        else {
          return;
        }

        if (formation_curr_.size() != formation_desired_.size()){
          ROS_ERROR_STREAM_THROTTLE(1, "[FormationControl]: Desired formation of size " << formation_desired_.size() << " does not match the current formation of size " << formation_curr_.size() << ". Returning.");
          return;
        }

        double error_curr = formationError(formation_curr_relative_, formation_desired_relative_);

        std_msgs::Float64 msg_pub;
        msg_pub.data = error_curr;
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

      double formationError(std::vector<pose_t> curr, std::vector<pose_t> desired){
        e::VectorXd diff_vector(curr.size()*4);

        for ( int i = 0; i < (int)(curr.size()); i++){
          auto curr_p_diff = desired.at(i).p - curr.at(i).p;
          auto curr_psi_diff = desired.at(i).psi - curr.at(i).psi;
      /* ROS_INFO_STREAM("[FormationControl]: des: \n" << desired.at(i).p.transpose() << ", " << desired.at(i).psi); */ 
      /* ROS_INFO_STREAM("[FormationControl]: cur: \n" << curr.at(i).p.transpose() << ", " << curr.at(i).psi); */ 
          diff_vector(i*4 + 0) = curr_p_diff.x();
          diff_vector(i*4 + 1) = curr_p_diff.y();
          diff_vector(i*4 + 2) = curr_p_diff.z();
          diff_vector(i*4 + 3) = curr_psi_diff;
        }

      /* ROS_INFO_STREAM("[FormationControl]: Partial errors: \n" << diff_vector.transpose()); */ 


        return diff_vector.norm();
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

      mrs_lib::Transformer m_transformer;
      ros::Publisher pub_formation_error_;
      ros::Timer timer_main_;

      std::string _common_frame_;
      std::vector<std::string> _uav_list_;
      std::string _formation_controler_node_;

      std::string _original_username_;
      std::string _local_username_;

      std::vector<agent_t> formation_curr_;
      std::vector<pose_t> formation_curr_relative_;

      mrs_lib::SubscribeHandler<dynamic_reconfigure::Config> m_sh_formation_control_updates;
      double overshoot_probability_;
      double proportional_constant_;
      bool control_enables_ = false;
      bool use_noise_ = false;
      std::string last_formation_file_;
      std::vector<agent_t> formation_desired_;
      std::vector<pose_t> formation_desired_relative_;

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
