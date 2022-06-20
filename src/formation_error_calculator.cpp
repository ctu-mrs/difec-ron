#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>


namespace e = Eigen;

namespace difec_ron {

  class FormationErrorCalculator {
    public: 
      FormationErrorCalculator(ros::NodeHandle& nh) {
        ROS_INFO("[FormationErrorCalculator]: Initializing formation error calculator...");

        param_loader.loadParam("common_frame", _common_frame_, string("global_gps"));

        mrs_lib::SubscribeHandlerOptions shopts(nh);
        shopts.no_message_timeout = ros::Duration(5.0);

        mrs_lib::construct_object(m_sh_odometry, shopts, "odometry_in");

        m_transformer = mrs_lib::Transformer(nh, m_node_name);
        m_transformer.retryLookupNewest(true);

        nh.advertise<std_msgs::Float64>("measuredPoses"+std::to_string(i+1), 1)
      }
    private: 
      mrs_lib::Transformer m_transformer;
      ros::Publisher formation_error;
      mrs_lib::SubscribeHandler<mrs_msgs::PoseWithCovarianceArrayStamped> m_sh_odometry;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "formation_error_calculator");

  ros::NodeHandle nh("~");
  difec_ron::FormationErrorCalculator dfecl(nh);
  ROS_INFO("[FormationErrorCalculator]: Formation error calculator node initiated");
  ros::spin();
  return 0;
}

