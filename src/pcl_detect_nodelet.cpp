/* includes etc. //{ */

#include "main.h"

#include <cmath>
#include <thread>
#include <algorithm>

#include <nodelet/nodelet.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/registration/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/surface/poisson.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <uav_detect/DetectionParamsConfig.h>

#include <eigen_conversions/eigen_msg.h>
/* #include <PointXYZt.h> */
#include <ouster_ros/OSConfigSrv.h>

#include <mrs_lib/scope_timer.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <uav_detect/Detections.h>
#include <uav_detect/Status.h>

#include "ros/duration.h"
#include "ros/wall_timer.h"
#include "uav_detect/voxel_map.h"
#include "uav_detect/voxel_grid_weighted.h"
#include "uav_detect/pc_loader.h"
#include "uav_detect/point_types.h"

//}

namespace uav_detect
{
  /* some defines //{ */
  
  // shortcut type to the dynamic reconfigure manager template instance
  using drmgr_t = mrs_lib::DynamicReconfigureMgr<uav_detect::DetectionParamsConfig>;

  struct xyz_lut_t
  {
    vec3s_t directions; // a matrix of normalized direction column vectors
    vec3s_t offsets;    // a matrix of offset vectors
  };
  
  /* enum cluster_class_t //{ */
  
  namespace cluster_class
  {
    enum cluster_class_t
    {
      mav,
      unknown,
      invalid
    };
  }
  using cluster_class_t = cluster_class::cluster_class_t;
  
  //}
  
  // axis-aligned bounding box
  struct aabb_t
  {
    vec3_t min_pt;
    vec3_t max_pt;
  };
  
  // oriented bounding box
  struct obb_t
  {
    vec3_t min_pt;
    vec3_t max_pt;
    vec3_t center_pt;
    Eigen::Matrix3f orientation;
  };
  
  struct cluster_t
  {
    cluster_class_t cclass;
    aabb_t aabb;
    obb_t obb;
    float obb_size;
    VoxelMap submap;
    pc_XYZR_t::ConstPtr pc;
    PointIndices::ConstPtr pc_indices;
  };
  
  struct detection_t
  {
    int id;
    aabb_t aabb;
    obb_t obb;
    mat3_t covariance;
    double confidence;
    double detection_probability;
  };
  
  //}

  class PCLDetector : public nodelet::Nodelet
  {
  public:
    /* onInit() method //{ */
    void onInit()
    {
      ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
      ROS_INFO("[PCLDetector]: Waiting for valid time...");
      ros::Time::waitForValid();

      m_node_name = "PCLDetector";

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
      const auto uav_name = pl.loadParam2<std::string>("uav_name");
      pl.loadParam("sensor/simulation", m_sensor_simulation);
      pl.loadParam("world_frame_id", m_world_frame_id);
      pl.loadParam("transform_lookup_timeout", m_transform_lookup_timeout);
      pl.loadParam("separate_cluster_removal_period", m_bgclusters_period);
      pl.loadParam("pointcloud_threads", m_n_pc_threads);
      pl.loadParam("throttle_period", m_throttle_period);

      const std::string static_cloud_filename = pl.loadParam2<std::string>("static_cloud_filename", "");

      pl.loadParam("voxel_map/voxel_size", m_vmap_voxel_size);
      pl.loadParam("voxel_map/scores/init", m_vmap_init_score);

      pl.loadParam("voxel_map/thresholds/apriori_map", m_vmap_threshold_apriori_map);
      
      pl.loadParam("voxel_map/colors/apriori_map", m_vmap_color_apriori_map);
      pl.loadParam("voxel_map/colors/new_obstacles", m_vmap_color_new_obstacles);
      pl.loadParam("voxel_map/colors/sure_obstacles", m_vmap_color_sure_obstacles);
      pl.loadParam("voxel_map/colors/frontiers", m_vmap_color_frontiers);
      pl.loadParam("voxel_map/colors/candidates", m_vmap_color_candidates);

      pl.loadParam("voxel_flags/colors/background", m_vflags_color_background);
      pl.loadParam("voxel_flags/colors/unknown", m_vflags_color_unknown);

      pl.loadParam("raycast/mask_filename", m_sensor_mask_fname, std::string(""));
      pl.loadParam("raycast/mask_rows", m_sensor_mask_rows, 0);
      pl.loadParam("raycast/min_intensity", m_raycast_min_intensity);

      pl.loadParam("exclude_box/offset/x", m_exclude_box_offset_x);
      pl.loadParam("exclude_box/offset/y", m_exclude_box_offset_y);
      pl.loadParam("exclude_box/offset/z", m_exclude_box_offset_z);
      pl.loadParam("exclude_box/size/x", m_exclude_box_size_x);
      pl.loadParam("exclude_box/size/y", m_exclude_box_size_y);
      pl.loadParam("exclude_box/size/z", m_exclude_box_size_z);
      m_exclude_box_offset_z = m_exclude_box_offset_z + m_exclude_box_size_z / 2.0f;

      pl.loadParam("operation_area/offset/x", m_oparea_offset_x);
      pl.loadParam("operation_area/offset/y", m_oparea_offset_y);
      pl.loadParam("operation_area/offset/z", m_oparea_offset_z);
      pl.loadParam("operation_area/size/x", m_oparea_size_x);
      pl.loadParam("operation_area/size/y", m_oparea_size_y);
      pl.loadParam("operation_area/size/z", m_oparea_size_z);
      m_oparea_offset_z = m_oparea_offset_z + m_oparea_size_z / 2.0f;

      Eigen::Affine3f apriori_map_tf = Eigen::Affine3f::Identity();
      {
        const vec3_t translation(pl.loadParam2<double>("apriori_map/tf/x"), pl.loadParam2<double>("apriori_map/tf/y"), pl.loadParam2<double>("apriori_map/tf/z"));
        const Eigen::Matrix3f rotation = anax_t(pl.loadParam2<double>("apriori_map/tf/yaw")/180.0*M_PI, vec3_t::UnitZ()).toRotationMatrix();

        const vec3_t sim_correction(pl.loadParam2<double>("apriori_map/sim_correction/x"), pl.loadParam2<double>("apriori_map/sim_correction/y"), pl.loadParam2<double>("apriori_map/sim_correction/z"));
        m_oparea_offset_x += sim_correction.x();
        m_oparea_offset_y += sim_correction.y();
        m_oparea_offset_z += sim_correction.z();

        apriori_map_tf.rotate(rotation);
        apriori_map_tf.translate(translation + sim_correction);
      }

      const auto background_sufficient_points_ratio = pl.loadParam2<float>("background_sufficient_points_ratio");
      const auto n_voxels_xy = m_oparea_size_x/m_vmap_voxel_size * m_oparea_size_y/m_vmap_voxel_size;
      m_background_min_sufficient_pts = n_voxels_xy*background_sufficient_points_ratio;

      // CHECK LOADING STATUS
      if (!pl.loadedSuccessfully())
      {
        NODELET_ERROR("Some compulsory parameters were not loaded successfully, ending the node");
        ros::shutdown();
        return;
      }
      //}

      /* Create publishers and subscribers //{ */
      // Initialize transform listener
      m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(m_tf_buffer);
      mrs_lib::SubscribeHandlerOptions shopts(nh);
      shopts.no_message_timeout = ros::Duration(5.0);
      // Initialize subscribers
      mrs_lib::construct_object(m_sh_pc, shopts, "pointcloud");
      mrs_lib::construct_object(m_sh_rangefinder, shopts, "height_rangefinder");
      // Initialize publishers
      m_pub_filtered_input_pc = nh.advertise<sensor_msgs::PointCloud2>("filtered_input_pc", 1);
      m_pub_weighted_input_pc = nh.advertise<sensor_msgs::PointCloud2>("weighted_input_pc", 1);
      m_pub_background_pc = nh.advertise<sensor_msgs::PointCloud2>("background_pc", 1);
      m_pub_vmap = nh.advertise<visualization_msgs::Marker>("voxel_map", 1);
      m_pub_update_flags = nh.advertise<visualization_msgs::Marker>("update_flags", 1);
      m_pub_oparea = nh.advertise<visualization_msgs::Marker>("operation_area", 1, true);
      m_pub_apriori_pc = nh.advertise<sensor_msgs::PointCloud2>("apriori_pc", 1, true);
      m_pub_background_clusters_pc = nh.advertise<sensor_msgs::PointCloud2>("background_clusters_pc", 1);
      m_pub_sepclusters_pc = nh.advertise<sensor_msgs::PointCloud2>("sepclusters_pc", 1);

      m_pub_frontiers_mks = nh.advertise<visualization_msgs::MarkerArray>("frontiers_mks", 1);
      m_pub_sure_air_pc = nh.advertise<sensor_msgs::PointCloud2>("sure_air_pc", 1);

      m_pub_classif_max_dist = nh.advertise<mrs_msgs::Sphere>("classification_max_distance", 1);
      m_pub_detections = nh.advertise<uav_detect::Detections>("detections", 1);
      m_pub_detections_pc = nh.advertise<sensor_msgs::PointCloud2>("detections_pc", 1);
      m_pub_detections_dbg = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("detections_dbg", 1);
      m_pub_detections_mks = nh.advertise<visualization_msgs::MarkerArray>("detections_mks", 1);
      m_pub_status = nh.advertise<uav_detect::Status>("status", 1);

      m_pub_lidar_fov = nh.advertise<visualization_msgs::Marker>("lidar_fov", 1, true);
      m_pub_lidar_raycast = nh.advertise<visualization_msgs::Marker>("lidar_raycast", 1);
      /* m_pub_evil_voxels = nh.advertise<visualization_msgs::Marker>("evil_voxels", 1); */

      m_reset_server = nh.advertiseService("reset", &PCLDetector::reset_callback, this);
      //}

      reset();

      // initialize the apriori map
      m_sure_background_sufficient = false;
      m_background_pts_sufficient = false;
      m_apriori_map_initialized = false;
      std::thread apriori_load_thread(&PCLDetector::initialize_apriori_map, this, static_cloud_filename, apriori_map_tf);
      apriori_load_thread.detach();

      // initialize the sensor information
      m_sensor_initialized = false;
      m_sensor_params_checked = false;
      m_sensor_params_ok = false;
      std::thread sensor_load_thread(&PCLDetector::initialize_sensor, this);
      sensor_load_thread.detach();

      m_last_detection_id = 0;

      m_main_thread = std::thread(&PCLDetector::main_loop, this);
      m_main_thread.detach();

      std::cout << "----------------------------------------------------------" << std::endl;
    }
    //}

    /* initialize_apriori_map() method //{ */
    void initialize_apriori_map(std::string filename, const Eigen::Affine3f& transformation)
    {
      pc_XYZ_t::Ptr apriori_cloud = boost::make_shared<pc_XYZ_t>();
      // if the filename is not specified, just leave the apriori cloud clear
      if (filename.empty())
      {
        NODELET_WARN("Apriori static map filename is empty. Not using apriori map.");
        m_apriori_map_initialized = true;
      }
      // otherwise, try to load it from the file
      else
      {
        NODELET_INFO("Loading the apriori static map from file '%s'", filename.c_str());
        const pc_XYZ_t::Ptr loaded_cloud = load_cloud(filename);
        if (loaded_cloud == nullptr)
        {
          NODELET_ERROR("Failed to load the static pointcloud! Ending the node.");
          ros::shutdown();
          return;
        } else
        {
          apriori_cloud = loaded_cloud;
          pcl::transformPointCloud(*apriori_cloud, *apriori_cloud, transformation);
          NODELET_INFO("Loaded a static cloud with %lu points.", apriori_cloud->size());

          pc_XYZ_t::Ptr tmp_cloud = boost::make_shared<pc_XYZ_t>();
          pcl::VoxelGrid<pt_XYZ_t> vg;
          vg.setInputCloud(apriori_cloud);
          vg.setLeafSize(m_vmap_voxel_size, m_vmap_voxel_size, m_vmap_voxel_size);
          vg.filter(*tmp_cloud);
          std::swap(apriori_cloud, tmp_cloud);
          NODELET_INFO("Downsampled the static cloud to %lu points.", apriori_cloud->size());

          for (const auto& pt : *apriori_cloud)
            if (m_voxel_map.inLimits(pt.x, pt.y, pt.z))
              m_voxel_map.at(pt.x, pt.y, pt.z) = std::numeric_limits<float>::infinity();

          m_sure_background_sufficient = true;
          m_background_pts_sufficient = true;
        }
        m_apriori_map_initialized = true;
      }

      // publish the result
      apriori_cloud->header.frame_id = m_world_frame_id;
      const ros::Time stamp = ros::Time::now();
      pcl_conversions::toPCL(stamp, apriori_cloud->header.stamp);
      m_pub_apriori_pc.publish(apriori_cloud);
    }
    //}

    /* initialize_sensor() method //{ */
		void initialize_sensor_lut(const size_t w, const size_t h,
                               const std::vector<double>& azimuth_angles_deg, const std::vector<double>& altitude_angles_deg,
                               const double range_unit = 0.001, const double lidar_origin_to_beam_origin_mm = 0.0,
                               const ouster::mat4d& tf = ouster::mat4d::Identity())
		{
      ouster::XYZLut xyz_lut;
      xyz_lut = ouster::make_xyz_lut(w, h, range_unit, lidar_origin_to_beam_origin_mm, tf, azimuth_angles_deg, altitude_angles_deg);
      if (xyz_lut.direction.cols() != xyz_lut.offset.cols())
        NODELET_ERROR_STREAM("[PCLDetector]: XYZ LUT doesn't have the correct number of elements (number of direction vectors " << xyz_lut.direction.cols() << " is not equal to the number of offset vectors " << xyz_lut.offset.cols() << ")!");
      
      m_sensor_xyz_lut = {xyz_lut.direction.cast<float>().transpose(), xyz_lut.offset.cast<float>().transpose()};
      m_sensor_xyz_lut.directions.colwise().normalize();
      NODELET_INFO_STREAM("[PCLDetector]: Initialized XYZ LUT table with " << m_sensor_xyz_lut.directions.cols() << " elements.");
    }

    // copied directly from the simulation plugin
		void initialize_sensor_lut_simulation(const size_t w, const size_t h)
		{
      const int rangeCount = w;
      const int verticalRangeCount = h;
      std::vector<std::tuple<double, double, double>> coord_coeffs;
      const double minAngle = 0.0;
      const double maxAngle = 2.0*M_PI;

      const double verticalMinAngle = -m_sensor_vfov/2.0;
      const double verticalMaxAngle = m_sensor_vfov/2.0;

      const double yDiff = maxAngle - minAngle;
      const double pDiff = verticalMaxAngle - verticalMinAngle;

      const double yAngle_step = yDiff / (rangeCount - 1);
      const double pAngle_step = pDiff / (verticalRangeCount - 1);

      coord_coeffs.resize(verticalRangeCount * rangeCount);
      for (int row = 0; row < verticalRangeCount; row++)
      {
        for (int col = 0; col < rangeCount; col++)
        {
          // Get angles of ray to get xyz for point
          const double yAngle = col * yAngle_step + minAngle;
          const double pAngle = row * pAngle_step + verticalMinAngle;

          const double x_coeff = cos(pAngle) * cos(yAngle);
          const double y_coeff = cos(pAngle) * sin(yAngle);
          const double z_coeff = sin(pAngle);
          coord_coeffs.at(col + row * rangeCount) = {x_coeff, y_coeff, z_coeff};
        }
      }

      int it = 0;
      m_sensor_xyz_lut.directions.resize(3, rangeCount*verticalRangeCount);
      m_sensor_xyz_lut.offsets.resize(3, rangeCount*verticalRangeCount);
      for (int row = 0; row < verticalRangeCount; row++)
      {
        for (int col = 0; col < rangeCount; col++)
        {
          const auto [x_coeff, y_coeff, z_coeff] = coord_coeffs.at(col + row * rangeCount);
          m_sensor_xyz_lut.directions.col(it) = vec3_t(x_coeff, y_coeff, z_coeff);
          m_sensor_xyz_lut.offsets.col(it) = vec3_t(0, 0, 0);
          it++;
        }
      }
    }

    void initialize_sensor_rosparam()
    {
      ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
      mrs_lib::ParamLoader pl(nh, m_node_name);
      pl.loadParam("sensor/vertical_fov_angle", m_sensor_vfov);
      pl.loadParam("sensor/vertical_rays", m_sensor_vrays);
      pl.loadParam("sensor/horizontal_rays", m_sensor_hrays);

      // CHECK LOADING STATUS
      if (pl.loadedSuccessfully())
      {
        if (m_sensor_simulation)
          initialize_sensor_lut_simulation(m_sensor_hrays, m_sensor_vrays);
        else
          initialize_sensor_lut(m_sensor_hrays, m_sensor_vrays, ouster::sensor::gen1_azimuth_angles, ouster::sensor::gen1_altitude_angles);
        m_sensor_initialized = true;
      }
      else
      {
        NODELET_ERROR("Could not load sensor parameters from rosparam server, ending the node");
        ros::shutdown();
      }
    }

    void initialize_sensor()
    {
      ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
      auto client = nh.serviceClient<ouster_ros::OSConfigSrv>("os_config");
      std::vector<int> pixel_shift_by_row;

      NODELET_INFO_STREAM("[PCLDetector]: Waiting 15s for service \"" << client.getService() << "\" to become available.");
      if (m_sensor_simulation || !client.waitForExistence(ros::Duration(25.0)))
      {
        if (m_sensor_simulation)
          NODELET_ERROR("[PCLDetector]: Using a simulated sensor! Loading data from rosparam server.");
        else
          NODELET_ERROR("[PCLDetector]: OS config service is not ready in 10s! Loading data from rosparam server (DATA WILL BE UNCALIBRATED!!).");
        initialize_sensor_rosparam();
        pixel_shift_by_row.resize(m_sensor_hrays, 0);
      }
      else
      {
        ouster_ros::OSConfigSrv cfg;
        if (!client.call(cfg))
        {
          NODELET_ERROR("[PCLDetector]: Calling OS config service failed! Loading data from rosparam server (DATA WILL BE UNCALIBRATED!!).");
          initialize_sensor_rosparam();
          pixel_shift_by_row.resize(m_sensor_hrays, 0);
        }
        else
        {
          const auto info = ouster::sensor::parse_metadata(cfg.response.metadata);
          const auto H = info.format.pixels_per_column;
          const auto W = info.format.columns_per_frame;
          pixel_shift_by_row = info.format.pixel_shift_by_row;

          NODELET_INFO("[PCLDetector]: Calling OS config service succeeded! Initializing sensor parameters from the received response.");
          m_sensor_vrays = H;
          m_sensor_hrays = W;
      
		/* void initialize_sensor_lut(size_t w, size_t h, */
                               /* const std::vector<double>& azimuth_angles_deg, const std::vector<double>& altitude_angles_deg, */
                               /* const double range_unit = 0.001, const double lidar_origin_to_beam_origin_mm = 0.0, */
                               /* const ouster::mat4d& tf = ouster::mat4d::Identity()) */
          initialize_sensor_lut(W, H, info.beam_azimuth_angles, info.beam_altitude_angles,
                                ouster::sensor::range_unit, info.lidar_origin_to_beam_origin_mm,
                                info.lidar_to_sensor_transform);
          m_sensor_vfov = std::abs(info.beam_altitude_angles.back() - info.beam_altitude_angles.front());
        }
      }
      
      // Load the mask and print some info to the console
      m_sensor_mask = load_mask(m_sensor_mask_fname, m_sensor_hrays, m_sensor_vrays, pixel_shift_by_row);
      NODELET_INFO_STREAM("[PCLDetector]: Initialized using sensor parameters:" << std::endl
          << "\tvertical rays: " << m_sensor_vrays
          << "\tvertical FOV: " << m_sensor_vfov
          << "\thorizontal rays: " << m_sensor_hrays
          );
      m_sensor_initialized = true;
    }
    //}

    /* load_mask() method //{ */

    std::vector<uint8_t> load_mask(const std::string& fname, const size_t exp_cols, const size_t exp_rows, const std::vector<int>& pixel_shift_by_row)
    {
      std::vector<uint8_t> ret;
      cv::Mat mask = cv::imread(fname, cv::IMREAD_GRAYSCALE);
      if (mask.data != nullptr)
      {
        if (mask.cols == (int)exp_cols && mask.rows == (int)exp_rows)
        {
          NODELET_INFO("[PCLDetector]: Loaded image mask file \"%s\" with dimensions %dx%d.", fname.c_str(), mask.cols, mask.rows);
          mask.rowRange(0, mask.rows - m_sensor_mask_rows) = 255;
          /* mask = mask.t(); */
          ret.resize(mask.cols*mask.rows);

          if (m_sensor_simulation)
          {
            for (int it = 0; it < mask.cols*mask.rows; it++)
              ret.at(it) = mask.at<uint8_t>(it);
          }
          else
          {
            const auto H = m_sensor_vrays;
            const auto W = m_sensor_hrays;
            for (int u = 0; u < H; u++)
            {
              for (int v = 0; v < W; v++)
              {
                const size_t vv = (v + pixel_shift_by_row.at(u)) % W;
                const size_t index = vv * H + u;
                ret.at(index) = mask.at<uint8_t>(u * W + v);
              }
            }
          }
        }
        else
        {
          NODELET_ERROR("[PCLDetector]: Image mask in file \"%s\" has wrong dimensions (%dx%d, expected %lux%lu)! Ignoring mask.", fname.c_str(), mask.cols, mask.rows, exp_cols, exp_rows);
        }
      }
      else
      {
        NODELET_WARN_STREAM("[PCLDetector]: Image mask file" << fname << "not found, ignoring mask.");
      }
      ret.resize(exp_cols*exp_rows, 1); // fill the rest of the mask with ones, if required
      return ret;
    }
    
    //}

    /* reset_callback() method //{ */

    bool reset_callback([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
    {
      reset();
      resp.message = "Detector reset.";
      resp.success = true;
      return true;
    }

    //}

  private:
    /* processMsg() method overloads //{ */

    /* overload for the sensor_msgs::Range message type //{ */

    void processMsg(const sensor_msgs::Range::ConstPtr msg)
    {
      // check validity of the measured range
      if (msg->range <= msg->min_range && msg->range >= msg->max_range)
        return;

      /* transform the measured point to the static world frame //{ */

      Eigen::Affine3d s2w_tf;
      bool tf_ok = get_transform_to_world(msg->header.frame_id, msg->header.stamp, s2w_tf);
      if (!tf_ok)
      {
        NODELET_ERROR_THROTTLE(1.0, "[PCLDetector]: Could not transform point to global, skipping.");
        return;
      }
      const vec3_t pt_tfd = s2w_tf.cast<float>() * vec3_t(msg->range, 0.0, 0.0);

      //}

      if (!m_voxel_map.inLimits(pt_tfd.x(), pt_tfd.y(), pt_tfd.z()))
      {
        NODELET_ERROR_THROTTLE(0.5, "[PCLDetector]: Range measurement is outside of the operational area! Cannot update ground map.");
        return;
      }

      {
        std::scoped_lock lck(m_voxels_mtx);
        auto& mapval = m_voxel_map.at(pt_tfd.x(), pt_tfd.y(), pt_tfd.z());
        mapval = (mapval + m_drmgr_ptr->config.voxel_map__scores__point) / 2.0;
      }
    }

    //}

    /* overload for the pc_XYZ_t message type //{ */

    /* filterAndTransform() method //{ */
    
    pc_XYZR_t::Ptr filterAndTransform(const pc_t::ConstPtr cloud, const Eigen::Affine3f& s2w_tf)
    {
      pc_t::Ptr cloud_filtered = boost::make_shared<pc_t>();
      /* filter by cropping points inside a box, relative to the sensor //{ */
      {
        const Eigen::Vector4f box_point1(m_exclude_box_offset_x + m_exclude_box_size_x / 2, m_exclude_box_offset_y + m_exclude_box_size_y / 2,
                                         m_exclude_box_offset_z + m_exclude_box_size_z / 2, 1);
        const Eigen::Vector4f box_point2(m_exclude_box_offset_x - m_exclude_box_size_x / 2, m_exclude_box_offset_y - m_exclude_box_size_y / 2,
                                         m_exclude_box_offset_z - m_exclude_box_size_z / 2, 1);
        pcl::CropBox<pt_t> cb;
        cb.setMax(box_point1);
        cb.setMin(box_point2);
        cb.setInputCloud(cloud);
        cb.setNegative(true);
        cb.filter(*cloud_filtered);
      }
      //}
      NODELET_INFO_STREAM_THROTTLE(1.0, "[PCLDetector]: Input PC after CropBox 1: " << cloud_filtered->size() << " points");
    
      pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, s2w_tf);
      cloud_filtered->header.frame_id = m_world_frame_id;
    
      /* filter by cropping points outside a box, relative to the global origin, (outside the operation area) //{ */
      {
        const Eigen::Vector4f box_point1(m_oparea_offset_x + m_oparea_size_x / 2, m_oparea_offset_y + m_oparea_size_y / 2,
                                         m_oparea_offset_z + m_oparea_size_z / 2, 1);
        const Eigen::Vector4f box_point2(m_oparea_offset_x - m_oparea_size_x / 2, m_oparea_offset_y - m_oparea_size_y / 2,
                                         m_oparea_offset_z - m_oparea_size_z / 2, 1);
        pcl::CropBox<pt_t> cb;
        cb.setMax(box_point1);
        cb.setMin(box_point2);
        cb.setInputCloud(cloud_filtered);
        cb.setNegative(false);
        cb.filter(*cloud_filtered);
      }
      //}
      NODELET_INFO_STREAM_THROTTLE(1.0, "[PCLDetector]: Input PC after CropBox 2: " << cloud_filtered->size() << " points");
    
      pcl::PointCloud<pt_XYZR_t>::Ptr cloud_weighted = boost::make_shared<pcl::PointCloud<pt_XYZR_t>>();
      {
        VoxelGridWeighted vgw;
        vgw.setInputCloud(cloud_filtered);
        vgw.setLeafSize(m_vmap_voxel_size, m_vmap_voxel_size, m_vmap_voxel_size);
        const auto [align_x, align_y, align_z] = m_voxel_map.idxToCoord(0, 0, 0);
        vgw.setVoxelAlign({align_x, align_y, align_z, 0.0f});
        vgw.filter(*cloud_weighted);
        cloud_weighted->header.frame_id = cloud_filtered->header.frame_id;
      }

      // publish some debug shit
      if (m_pub_filtered_input_pc.getNumSubscribers() > 0)
      {
        cloud_filtered->header.stamp = cloud->header.stamp;
        m_pub_filtered_input_pc.publish(cloud_filtered);
      }
      if (m_pub_weighted_input_pc.getNumSubscribers() > 0)
      {
        cloud_weighted->header.stamp = cloud->header.stamp;
        m_pub_weighted_input_pc.publish(cloud_weighted);
      }

      NODELET_INFO_STREAM_THROTTLE(1.0, "[PCLDetector]: Filtered input PC has " << cloud_weighted->size() << "/" << cloud->size() << " valid unique points (\033[1;31m" << 100.0f*float(cloud_weighted->size())/cloud->size() << "%\033[0m)");
      return cloud_weighted;
    }
    
    //}

    /* clusterCloud() method //{ */
    template<class T>
    std::vector<PointIndices> clusterCloud(const typename boost::shared_ptr<T> cloud, const float max_distance)
    {
      std::vector<PointIndices> ret;
      pcl::EuclideanClusterExtraction<typename T::PointType> ece;
      ece.setClusterTolerance(max_distance);
      ece.setInputCloud(cloud);
      ece.extract(ret);
      return ret;
    }
    //}

    /* findCloseFarClusters() method //{ */
    // separates clusters to those which are closer and further than a threshold to a pointcloud
    std::pair<std::vector<PointIndices::ConstPtr>, std::vector<PointIndices::ConstPtr>> findCloseFarClusters(const pc_XYZR_t::ConstPtr cloud, const std::vector<PointIndices>& clusters_indices)
    {
      std::vector<PointIndices::ConstPtr> close_clusters_indices;
      std::vector<PointIndices::ConstPtr> far_clusters_indices;
      close_clusters_indices.reserve(clusters_indices.size());
      far_clusters_indices.reserve(clusters_indices.size());
      const auto max_dist = m_drmgr_ptr->config.ground_points_max_distance;
      const auto threshold_new_obstacles = m_drmgr_ptr->config.voxel_map__thresholds__new_obstacles;
    
      std::lock_guard lck(m_voxels_mtx);

      // check the number of voxels, classified as background (used to decide whether to run detailed cluster classification later)
      const uint64_t n_bg_pts = m_voxel_map.nVoxelsOver(threshold_new_obstacles);
      if (n_bg_pts > m_background_min_sufficient_pts)
      {
        if (!m_background_pts_sufficient)
          NODELET_INFO_THROTTLE(1.0, "[PCLDetector]: Sufficient number of ground points achieved (%lu)!", n_bg_pts);
        m_background_pts_sufficient = true;
      }
      else
      {
        NODELET_WARN_THROTTLE(1.0, "[PCLDetector]: Insufficient number of ground points (%lu, require at least %lu)! Cluster classification is inactive.", n_bg_pts, m_background_min_sufficient_pts);
      }

      for (const auto& cluster_indices : clusters_indices)
      {
        bool is_close = false;
        // check if this cluster contains at least one close point
        for (const auto& idx : cluster_indices.indices)
        {
          const auto& pt = cloud->at(idx);
          // if a neighbor within the 'm_drmgr_ptr->config.ground_points_max_distance' radius was found, this cluster will be classified as a block of ground
          if (m_voxel_map.hasCloseTo(pt.x, pt.y, pt.z, max_dist, threshold_new_obstacles))
          {
            // set the corresponding local flag
            is_close = true;
            // no need to continue checking this cluster - break
            break;
          }
        }
    
        if (is_close)
          close_clusters_indices.push_back(boost::make_shared<PointIndices>(cluster_indices));
        else
          far_clusters_indices.push_back(boost::make_shared<PointIndices>(cluster_indices));
      }
      return {close_clusters_indices, far_clusters_indices};
    }
    //}

    /* extractClustersPoints() method //{ */
    pc_XYZR_t::Ptr extractClustersPoints(const pc_XYZR_t::ConstPtr cloud, const std::vector<PointIndices::ConstPtr>& clusters_indices)
    {
      pc_XYZR_t::Ptr ret = boost::make_shared<pc_XYZR_t>();
      size_t pts = 0;
      for (const auto& idcs : clusters_indices)
        pts += idcs->indices.size();
      ret->reserve(pts);
    
      for (const auto& idcs : clusters_indices)
      {
        pc_XYZR_t tmp_cloud;
        pcl::ExtractIndices<pt_XYZR_t> ei;
        ei.setInputCloud(cloud);
        ei.setIndices(idcs);
        ei.filter(tmp_cloud);
        ret->insert(std::end(*ret), std::begin(tmp_cloud), std::end(tmp_cloud));
      }
      ret->header.frame_id = cloud->header.frame_id;
      return ret;
    }
    //}

    /* updateVMaps() method //{ */
    void updateVMaps(const pc_XYZR_t::ConstPtr cloud, const std::vector<PointIndices::ConstPtr>& clusters_indices, const float vmap_score, const float vflags)
    {
      for (const auto& cluster_indices : clusters_indices)
      {
        for (const auto idx : cluster_indices->indices)
        {
          const auto pt = cloud->at(idx);
          const auto [xc, yc, zc] = m_voxel_map.coordToIdx(pt.x, pt.y, pt.z);
          /* if (!m_voxel_map.inLimitsIdx(xc, yc, zc)) */
          /* { */
          /*   const Eigen::Vector3f max_pt(m_oparea_offset_x + m_oparea_size_x / 2, m_oparea_offset_y + m_oparea_size_y / 2, */
          /*                                m_oparea_offset_z + m_oparea_size_z / 2); */
          /*   const Eigen::Vector3f min_pt(m_oparea_offset_x - m_oparea_size_x / 2, m_oparea_offset_y - m_oparea_size_y / 2, */
          /*                                m_oparea_offset_z - m_oparea_size_z / 2); */
          /*   ROS_ERROR("[PCLDetector]: POINT NOT WITHIN LIMITS! [%.2f, %.2f, %.2f] (indices: [%d, %d, %d]), oparea from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]", pt.x, pt.y, pt.z, xc, yc, zc, min_pt.x(), min_pt.y(), min_pt.z(), max_pt.x(), max_pt.y(), max_pt.z()); */
          /* } */

          auto& mapval = m_voxel_map.atIdx(xc, yc, zc);
          // pt.range is the weight of the point (how many times it should be applied)
          const float w = 1.0f/static_cast<float>(1lu << std::clamp(pt.range, 0u, 63u));
          /* if (w < 0.0f || (1.0f-w) < 0.0f) */
          /*   ROS_ERROR("[PCLDetector]: Invalid weight: w1 = %.2f, w2 = %.2f!", w, (1.0f-w)); */
          mapval = w*mapval + (1.0f-w)*vmap_score;
          // set the flag, indicating that this voxel was updated during this turn
          m_voxel_flags.atIdx(xc, yc, zc) = vflags;
        }
      }
    }
    //}

    /* classifyClusters() method //{ */
    std::vector<cluster_t> classifyClusters(const pc_XYZR_t::ConstPtr cloud, const std::vector<PointIndices::ConstPtr>& clusters_indices, const Eigen::Affine3f& s2w_tf)
    {
      std::vector<cluster_t> clusters;
      clusters.reserve(clusters_indices.size());
      // go through the detection candidates and classify them
      for (const auto& cluster_indices : clusters_indices)
      {
        const cluster_t cluster = classify_cluster(cloud, cluster_indices, s2w_tf);
        clusters.push_back(cluster);
      }
      return clusters;
    }
    //}

    /* extractDetections() method //{ */
    std::vector<detection_t> extractDetections(const std::vector<cluster_t>& clusters, const vec3_t& detector_pos)
    {
      std::vector<detection_t> detections;
      detections.reserve(clusters.size());
      // go through the detection candidates and classify them
      for (const auto& cluster : clusters)
      {
        if (cluster.cclass != cluster_class_t::mav)
          continue;
        const double det_dist = (detector_pos - cluster.obb.center_pt).norm();
        detection_t det;
        det.id = m_last_detection_id++;
        det.aabb = cluster.aabb;
        det.obb = cluster.obb;
        det.covariance = std::sqrt(det_dist)*m_drmgr_ptr->config.output__position_sigma*mat3_t::Identity();

        auto submap = m_voxel_map.getSubmapCopy(det.aabb.min_pt, det.aabb.max_pt, 2);
        // set the cluster points to air to ignore them in the score
        for (const auto& idx : cluster.pc_indices->indices)
        {
          const auto pt = cluster.pc->at(idx);
          submap.at(pt.x, pt.y, pt.z) = m_drmgr_ptr->config.voxel_map__scores__ray; // consider points from the cluster as free air
        }

        // calculate the total uncertainty score as the score of voxels in the neighborhood weighted by the number of cluster points
        double uncertainty_score = 0.0f;
        for (auto& val : submap)
          uncertainty_score += 1.0 - val/m_drmgr_ptr->config.voxel_map__scores__ray;
        // divide the weight by the number of 
        const auto n_pts = cluster.pc_indices->indices.size();
        uncertainty_score /= n_pts;
        // confidence is then an inverse exponential of the weight
        det.confidence = static_cast<float>(1.0/std::exp(uncertainty_score));

        const double vray_res = m_sensor_vfov/static_cast<double>(m_sensor_vrays);
        const double hray_res = 2*M_PI/static_cast<double>(m_sensor_hrays);
        const double pdet_vert = std::min( std::atan(1.0/det_dist)/(vray_res*m_drmgr_ptr->config.classification__min_points), 1.0);
        const double pdet_hori = std::min( std::atan(1.0/det_dist)/(hray_res), 1.0);
        const double pdet = pdet_vert * pdet_hori;
        det.detection_probability = pdet;

        detections.push_back(det);
      }
      return detections;
    }
    //}

    void processMsg(const pc_t::ConstPtr cloud, const int thread_n)
    {
      ros::Time msg_stamp;
      pcl_conversions::fromPCL(cloud->header.stamp, msg_stamp);
      mrs_lib::ScopeTimer stimer("pc proc #" + std::to_string(thread_n), {"msg stamp", msg_stamp}, m_throttle_period);

      if (m_sensor_xyz_lut.directions.cols() != m_sensor_xyz_lut.directions.cols())
      {
        NODELET_ERROR("[PCLDetector]: Invalid XYZ LUT! Number of direction vectors (%ld) does not equal the number of offsets (%ld)! Skipping.", m_sensor_xyz_lut.directions.cols(), m_sensor_xyz_lut.offsets.cols());
        return;
      }

      if (cloud->size() != (size_t)m_sensor_xyz_lut.directions.cols())
      {
        NODELET_ERROR("[PCLDetector]: Unexpected size of pointcloud! Expected: %ld (%d vert. x %d hor.), got: %lu. Skipping.", m_sensor_xyz_lut.directions.cols(), m_sensor_vrays, m_sensor_hrays, cloud->size());
        return;
      }

      NODELET_INFO_STREAM_THROTTLE(1.0, "[PCLDetector]: Processing new pointcloud in thead #" << thread_n);

      if (!m_sensor_params_checked || !m_sensor_params_ok)
        check_sensor_params(cloud);

      std::string cloud_frame_id = cloud->header.frame_id;  // cut off the first forward slash
      if (!cloud_frame_id.empty() &&  cloud_frame_id.at(0) == '/')
        cloud_frame_id = cloud_frame_id.substr(1);  // cut off the first forward slash
      std_msgs::Header header;
      header.stamp = msg_stamp;
      header.frame_id = m_world_frame_id;

      Eigen::Affine3f s2w_tf;
      {
        Eigen::Affine3d s2w_tfd;
        bool tf_ok = get_transform_to_world(cloud_frame_id, msg_stamp, s2w_tfd);
        if (!tf_ok)
        {
          NODELET_ERROR_THROTTLE(1.0, "[PCLDetector]: Could not transform cloud to global, skipping.");
          return;
        }
        s2w_tf = s2w_tfd.cast<float>();
      }
      stimer.checkpoint("tf lookup");

      /* filter input cloud and transform it to world */
      // reduce the pointcloud using a weighted VoxelGrid
      const auto cloud_weighted = filterAndTransform(cloud, s2w_tf);
      stimer.checkpoint("filtering");

      // separate the current cloud to clusters with max. point distance m_drmgr_ptr->config.ground_points_max_distance
      const std::vector<PointIndices> clusters_indices = clusterCloud(cloud_weighted, m_drmgr_ptr->config.ground_points_max_distance);
      stimer.checkpoint("clusterization");

      // find clusters, which are close enough to background points to be considered background as well, and clusters, which are further
      const auto [close_clusters_indices, far_clusters_indices] = findCloseFarClusters(cloud_weighted, clusters_indices);
      stimer.checkpoint("close X far");

      ros::WallTime vmap_lock_t;
      {
        // update the voxel map and flags map
        // prepare the map of flags
        std::scoped_lock lck(m_voxels_mtx);
        stimer.checkpoint("vmap lock");
        // add points from the current scan, which were classified as background, to the voxelmaps
        updateVMaps(cloud_weighted, close_clusters_indices, m_drmgr_ptr->config.voxel_map__scores__point, m_vflags_point);
        // go through the detection candidates and mark the corresponding voxels in voxel flags as unknown
        updateVMaps(cloud_weighted, far_clusters_indices, m_drmgr_ptr->config.voxel_map__scores__unknown, m_vflags_unknown);
        m_detection_its++;
      }
      m_detection_cv.notify_one();
      if (!m_raycast_running)
      {
        m_raycast_running = true;
        m_raycast_thread = std::thread(&PCLDetector::raycast_cloud, this, cloud, s2w_tf);
        m_raycast_thread.detach();
      }
      stimer.checkpoint("vmap update");

      // classify the clusters using the updated data
      const std::vector<cluster_t> clusters = classifyClusters(cloud_weighted, far_clusters_indices, s2w_tf);
      // filter out only the valid detections
      const std::vector<detection_t> detections = extractDetections(clusters, s2w_tf.translation());
      stimer.checkpoint("classification");

      // publish the main output ASAP - other stuff can wait
      {
        uav_detect::Detections msg;
        msg.header = header;
        msg.detections.reserve(detections.size());
        for (const auto& det : detections)
        {
          uav_detect::Detection tmp;
          tmp.id = det.id;
          tmp.confidence = det.confidence;
          tmp.detection_probability = det.detection_probability;
          tmp.position.x = det.obb.center_pt.x();
          tmp.position.y = det.obb.center_pt.y();
          tmp.position.z = det.obb.center_pt.z();
          for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
              tmp.covariance.at(3*r + c) = det.covariance(r, c);
          msg.detections.push_back(tmp);
        }
        NODELET_INFO_THROTTLE(1.0, "[PCLDetector]: Publishing %lu detections", detections.size());
        m_pub_detections.publish(msg);
      }

      /* Publish debug stuff //{ */
      
      if (m_pub_detections_mks.getNumSubscribers() > 0)
        m_pub_detections_mks.publish(clusters_visualization(clusters, header));
      
      if (m_pub_frontiers_mks.getNumSubscribers() > 0)
        m_pub_frontiers_mks.publish(frontier_visualization(clusters, header));
      
      if (m_pub_sure_air_pc.getNumSubscribers() > 0)
      {
        pc_XYZt_t::Ptr sure_air = m_voxel_map.voxelsAsPC(m_drmgr_ptr->config.voxel_map__thresholds__frontiers, false);
        sure_air->header.stamp = cloud->header.stamp;
        sure_air->header.frame_id = m_world_frame_id;
        m_pub_sure_air_pc.publish(sure_air);
      }

      if (m_pub_background_pc.getNumSubscribers() > 0)
      {
        pc_XYZt_t::Ptr bg_cloud = m_voxel_map.voxelsAsPC(m_drmgr_ptr->config.voxel_map__thresholds__new_obstacles, true);
        bg_cloud->header.stamp = cloud->header.stamp;
        bg_cloud->header.frame_id = m_world_frame_id;
        m_pub_background_pc.publish(bg_cloud);
      }
      
      if (m_pub_background_clusters_pc.getNumSubscribers() > 0)
      {
        pc_XYZR_t::Ptr background_cloud = extractClustersPoints(cloud_weighted, close_clusters_indices);
        background_cloud->header.stamp = cloud->header.stamp;
        m_pub_background_clusters_pc.publish(background_cloud);
      }
      
      if (m_pub_vmap.getNumSubscribers() > 0)
      {
        m_voxel_map.clearVisualizationThresholds();
        m_voxel_map.addVisualizationThreshold(m_drmgr_ptr->config.voxel_map__thresholds__new_obstacles, m_vmap_color_new_obstacles);
        m_voxel_map.addVisualizationThreshold(m_drmgr_ptr->config.voxel_map__thresholds__sure_obstacles, m_vmap_color_sure_obstacles);
        m_voxel_map.addVisualizationThreshold(m_drmgr_ptr->config.voxel_map__thresholds__apriori_map, m_vmap_color_apriori_map);
        m_pub_vmap.publish(m_voxel_map.visualization(header));
      }
      
      if (m_pub_update_flags.getNumSubscribers() > 0)
        m_pub_update_flags.publish(m_voxel_flags.visualization(header));

      if (m_pub_detections_dbg.getNumSubscribers() > 0)
      {
        mrs_msgs::PoseWithCovarianceArrayStamped msg;
        msg.header = header;
        msg.poses.reserve(detections.size());
        for (const auto& det : detections)
        {
          mrs_msgs::PoseWithCovarianceIdentified tmp;
          tmp.id = det.id;
          tmp.pose.position.x = det.obb.center_pt.x();
          tmp.pose.position.y = det.obb.center_pt.y();
          tmp.pose.position.z = det.obb.center_pt.z();
          tmp.pose.orientation.w = 1.0;
          for (int r = 0; r < 6; r++)
          {
            for (int c = 0; c < 6; c++)
            {
              if (r < 3 && c < 3)
                tmp.covariance[r * 6 + c] = det.covariance(r, c);
              else if (r == c)
                tmp.covariance[r * 6 + c] = 666;
              else
                tmp.covariance[r * 6 + c] = 0.0;
            }
          }
          msg.poses.push_back(tmp);
        }
        m_pub_detections_dbg.publish(msg);
      }

      if (m_pub_detections_pc.getNumSubscribers() > 0)
      {
        sensor_msgs::PointCloud2 msg;
        msg.header = header;
        sensor_msgs::PointCloud2Modifier pcm(msg);
        pcm.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                    "y", 1, sensor_msgs::PointField::FLOAT32,
                                    "z", 1, sensor_msgs::PointField::FLOAT32,
                                    "confidence", 1, sensor_msgs::PointField::FLOAT32);
        pcm.resize(detections.size());
        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_c(msg, "confidence");
        for (const auto& det : detections)
        {
          *iter_x = det.obb.center_pt.x();
          *iter_y = det.obb.center_pt.y();
          *iter_z = det.obb.center_pt.z();
          *iter_c = det.confidence;
          ++iter_x;
          ++iter_y;
          ++iter_z;
          ++iter_c;
        }
        m_pub_detections_pc.publish(msg);
      }
      
      //}

      const double delay = (ros::Time::now() - msg_stamp).toSec();
      NODELET_INFO_STREAM_THROTTLE(1.0, "[ProcessData]: Done processing data with delay " << delay << "s ---------------------------------------------- ");
    }

    //}

    //}

    void rangefinder_loop()
    {
      const ros::WallDuration timeout(1.0/10.0);
      while (ros::ok())
      {
        const auto msg_ptr = m_sh_rangefinder.waitForNew(timeout);
        if (msg_ptr)
          processMsg(msg_ptr);
      }
    }

    void pointcloud_loop(const int thread_n)
    {
      const ros::WallDuration timeout(1.0/10.0);
      while (ros::ok())
      {
        const auto msg_ptr = m_sh_pc.waitForNew(timeout);
        if (msg_ptr)
          processMsg(msg_ptr, thread_n);
      }
    }

    /* updateSeparatedBGClusters() method //{ */
    // separates clusters to those which are closer and further than a threshold to a pointcloud
    void updateSeparatedBGClusters(VoxelMap& local_vmap)
    {
      if (m_drmgr_ptr->config.sepclusters__pause)
      {
        NODELET_WARN_THROTTLE(1.0, "[SepClusters]: Separate background clusters removal disabled, skipping.");
        return;
      }
      const int start_detection_its = m_detection_its;
      mrs_lib::ScopeTimer tim("sep bg clusters", m_throttle_period);

      // prepare some parameters
      const auto max_dist = m_drmgr_ptr->config.ground_points_max_distance;
      const auto threshold_new_obstacles = m_drmgr_ptr->config.voxel_map__thresholds__new_obstacles;
      const auto threshold_sure_obstacles = m_drmgr_ptr->config.voxel_map__thresholds__sure_obstacles;
      const unsigned n_pts_sure_cluster = m_drmgr_ptr->config.sepclusters__min_sure_points;
      const VoxelMap::coord_t max_dist_idx = max_dist/m_vmap_voxel_size;
      const VoxelMap::idx_t max_voxel_dist = std::ceil(max_dist_idx);
    
      {
        std::lock_guard lck(m_voxels_mtx);
        tim.checkpoint("mutex lock1");
        local_vmap.copyDataIdx(m_voxel_map);
        tim.checkpoint("vmap copy");
      }

      // convert the voxelmap to a pointcloud
      VoxelMap::pc_t::Ptr vmap_pc = local_vmap.voxelsAsVoxelPC(threshold_new_obstacles);
      tim.checkpoint("pc create");
      if (vmap_pc->empty())
      {
        NODELET_WARN_THROTTLE(1.0, "[SepClusters]: Voxelmap pointcloud is empty, skipping.");
        return;
      }

      const float lsz = static_cast<float>(std::max(max_voxel_dist - 1, 0));
      pcl::VoxelGrid<VoxelMap::pt_t> vg;
      vg.setInputCloud(vmap_pc);
      vg.setLeafSize(lsz, lsz, lsz);
      vg.filter(*vmap_pc);
      tim.checkpoint("downsample");

      // clusterize voxels (reprezented as points), classified as background
      const std::vector<PointIndices> clusters_indices = clusterCloud(vmap_pc, max_voxel_dist);
      tim.checkpoint("clustering");

      // find the number of sure voxels in each cluster
      std::vector<unsigned> clusters_n_sure;
      for (const auto& cluster_indices : clusters_indices)
      {
        unsigned cur_n_sure = 0;
        for (const auto& idx : cluster_indices.indices)
        {
          const auto& pt = vmap_pc->points[idx];
          if (pt.intensity > threshold_sure_obstacles)
            cur_n_sure++;
        }
        clusters_n_sure.push_back(cur_n_sure);
      }
    
      // check if a sufficient number of sure background cluster was detected
      const size_t n_clusters = clusters_n_sure.size();
      const size_t n_sure_clusters = std::count_if(std::begin(clusters_n_sure), std::end(clusters_n_sure), [n_pts_sure_cluster](const auto a){return a > n_pts_sure_cluster;});
      NODELET_WARN_THROTTLE(1.0, "[SepClusters]: Found %lu background clusters, %lu unsure.", n_clusters, n_clusters-n_sure_clusters);
      if (n_sure_clusters == 0)
      {
        // if not, disable classification and return
        m_sure_background_sufficient = false;
        const auto most_sure_voxels = *std::max_element(std::begin(clusters_n_sure), std::end(clusters_n_sure));
        NODELET_WARN_THROTTLE(1.0, "[SepClusters]: No sure background clusters detected yet (most sure voxels: %u, required: %u)! Cluster classification is inactive.", most_sure_voxels, n_pts_sure_cluster);
        return;
      }
      else
      {
        // if yes, we can enable classification and update the other clusters
        if (!m_sure_background_sufficient)
          NODELET_INFO_THROTTLE(1.0, "[SepClusters]: Sure background cluster detected!");
        m_sure_background_sufficient = true;
      }
      tim.checkpoint("surest score");

      // update disconnected background clusters as unknown
      std::lock_guard lck(m_voxels_mtx);
      tim.checkpoint("mutex lock2");
      const float detection_its_diff = std::max(m_detection_its - start_detection_its, 1); // has to be at least one!

      const bool publish = m_pub_sepclusters_pc.getNumSubscribers() > 0;
      pc_XYZR_t pc;
      pc.header.frame_id = m_world_frame_id;
      pcl_conversions::toPCL(ros::Time::now(), pc.header.stamp);

      // generate relative indices to update for each downsampled voxel
      const VoxelMap::vec3i_t from_inds(-max_voxel_dist, -max_voxel_dist, -max_voxel_dist);
      const VoxelMap::vec3i_t to_inds(max_voxel_dist, max_voxel_dist, max_voxel_dist);
      // setup mask of voxels in range for the for loop
      std::vector<vec3i_t> index_offsets;
      for (VoxelMap::idx_t x_it = from_inds.x(); x_it <= to_inds.x(); x_it++)
      {
        for (VoxelMap::idx_t y_it = from_inds.y(); y_it <= to_inds.y(); y_it++)
        {
          for (VoxelMap::idx_t z_it = from_inds.z(); z_it <= to_inds.z(); z_it++)
          {
            const VoxelMap::vec3i_t cur_inds = vec3i_t(x_it, y_it, z_it);
            if (cur_inds.norm() <= max_dist_idx)
            {
              index_offsets.push_back(cur_inds);
            }
          }
        }
      }

      const float update_val = m_drmgr_ptr->config.voxel_map__scores__ray;
      const float w_update_single = 0.5f;
      const float w1 = std::clamp(std::pow(1.0f - w_update_single, detection_its_diff), 0.0f, 1.0f);
      const float w2 = 1.0f - w1;

      for (size_t it = 0; it < clusters_indices.size(); it++)
      {
        const unsigned cur_n_sure = clusters_n_sure.at(it);
        if (cur_n_sure <= n_pts_sure_cluster)
        {
          const auto cluster_indices = clusters_indices.at(it);
          for (const auto& idx : cluster_indices.indices)
          {
            const vec3i_t pos = vmap_pc->points[idx].getVector3fMap().cast<int>();
            for (const auto& offset : index_offsets)
            {
              const vec3i_t pt = pos + offset;
              if (!m_voxel_map.inLimitsIdx(pt))
                continue;
              auto& mapval = m_voxel_map.atIdx(pt.x(), pt.y(), pt.z());
              mapval = w1*mapval + w2*update_val;

              if (publish)
              {
                const vec3_t coords = local_vmap.idxToCoord(pt);
                pt_XYZR_t pub_pt;
                pub_pt.getVector3fMap() = coords;
                pub_pt.range = it;
                pc.push_back(pub_pt);
              }
            }
          }
        }
      }
      tim.checkpoint("score update");
      if (publish)
        m_pub_sepclusters_pc.publish(pc);
    }
    //}

    void bgclusters_loop()
    {
      // a local copy of the current global voxelmap (to avoid waiting for mutexes)
      VoxelMap local_vmap;
      {
        // resize the voxel maps
        std::lock_guard lck(m_voxels_mtx);
        local_vmap.resizeAs(m_voxel_map);
      }
      while (ros::ok())
      {
        updateSeparatedBGClusters(local_vmap);
        m_bgclusters_period.sleep();
      }
    }

    std::thread m_main_thread;
    /* main_loop() method //{ */
    void main_loop()
    {
      const ros::WallDuration timeout(1.0/10.0);
      /* wait for apriori map and sensor initialization //{ */
      
      while (ros::ok())
      {
        timeout.sleep();
      
        if (!m_apriori_map_initialized)
        {
          NODELET_INFO_ONCE("[PCLDetector]: Waiting for intialization of the apriori static map...");
          continue;
        }
      
        if (!m_sensor_initialized)
        {
          NODELET_INFO_ONCE("[PCLDetector]: Waiting for intialization of sensor configuration...");
          continue;
        }
      
        break;
      }
      
      //}

      std::thread bgclusters_thread(&PCLDetector::bgclusters_loop, this);
      std::thread rangefinder_thread(&PCLDetector::rangefinder_loop, this);
      std::vector<std::thread> pointcloud_threads;
      for (int it = 0; it < m_n_pc_threads; it++)
        pointcloud_threads.emplace_back(&PCLDetector::pointcloud_loop, this, it);

      // keep publishing some debug stuff
      while (ros::ok())
      {
        timeout.sleep();
        // publish the lidar FOV marker (only once)
        /*  //{ */

        static bool fov_published = false;
        if (m_sh_pc.hasMsg() && m_sensor_initialized && !fov_published)
        {
          const auto msg_ptr = m_sh_pc.peekMsg();
          std_msgs::Header header;
          header.frame_id = msg_ptr->header.frame_id;
          header.stamp = ros::Time::now();

          const auto msg = lidar_visualization(header);
          m_pub_lidar_fov.publish(msg);
          fov_published = true;
        }

        //}

        // publish the operational area borders marker (only once)
        /*  //{ */

        static bool oparea_published = false;
        if (!oparea_published)
        {
          std_msgs::Header header;
          header.frame_id = m_world_frame_id;
          header.stamp = ros::Time::now();
          const auto msg_ptr = m_voxel_map.borderVisualization(header);
          m_pub_oparea.publish(msg_ptr);
          oparea_published = true;
        }

        //}

        // publish the max. detection range (only once)
        if (m_sh_pc.hasMsg())
        {
          mrs_msgs::Sphere msg;
          msg.header.frame_id = m_sh_pc.peekMsg()->header.frame_id;
          msg.header.stamp = ros::Time::now();
          msg.radius = m_drmgr_ptr->config.classification__max_distance;
          m_pub_classif_max_dist.publish(msg);
        }

        // publish the current detection status
        {
          uav_detect::Status msg;
          msg.header.stamp = ros::Time::now();
          msg.detection_enabled = true;
          msg.detection_active = m_background_pts_sufficient;
          m_pub_status.publish(msg);
        }
      }

      bgclusters_thread.join();
      rangefinder_thread.join();
      for (auto& el : pointcloud_threads)
        el.join();
    }
    //}

  private:
    /* raycast_cloud() method //{ */
    void raycast_cloud(const pc_t::ConstPtr cloud, const Eigen::Affine3f& tf)
    {
      mrs_lib::AtomicScopeFlag run_flag(m_raycast_running); // will be automatically unset when scope ends
      if (m_drmgr_ptr->config.raycast__pause)
      {
        NODELET_WARN_THROTTLE(1.0, "[RaycastCloud]: Pointcloud raycasting disabled, skipping.");
        return;
      }

      if ((int)cloud->height != m_sensor_vrays || (int)cloud->width != m_sensor_hrays)
      {
        NODELET_ERROR_THROTTLE(1.0, "[RaycastCloud]: Pointcloud has wrong dimensions, cannot raycast, skipping (has: [%ux%u], expected: [%dx%d]).", cloud->width, cloud->height, m_sensor_hrays, m_sensor_vrays);
        return;
      }

      if (!m_sensor_params_checked)
      {
        NODELET_ERROR_THROTTLE(1.0, "[RaycastCloud]: Couldn't check validity of sensor's parameters (no valid points on pointcloud?), skipping.");
        return;
      }

      if (!m_sensor_params_ok)
      {
        NODELET_ERROR_THROTTLE(1.0, "[RaycastCloud]: Pointcloud doesn't correspond to the expected sensor parameters! Make sure they're set correctly (maybe you're using a different sensor?). Cannot raycast, skipping.");
        return;
      }

      const int start_detection_its = m_detection_its;
      mrs_lib::ScopeTimer tim("raycasting", m_throttle_period);;
      NODELET_INFO_STREAM_THROTTLE(1.0, "[RaycastCloud]: Started raycasting data with " << cloud->size() << " points ---------------------------------------------- ");
      const auto tf_rot = tf.rotation();
      const vec3_t origin_pt = tf.translation();  // origin of all rays of the lidar sensor
      m_voxel_raycast.clear(); // clear the helper voxelmap
      // do not raycast if the sensor is out of bounds
      if (m_voxel_raycast.inLimits(origin_pt.x(), origin_pt.y(), origin_pt.z()))
      {
        bool publish = m_pub_lidar_raycast.getNumSubscribers() > 0;

        // go through all points in the cloud and update voxels in the helper voxelmap that the rays
        // from the sensor origin to the point go through according to how long part of the ray
        // intersects the voxel
        for (int row = 0; row < (int)cloud->height; row++)
        {
          for (int col = 0; col < (int)cloud->width; col++)
          {
            const auto pt = cloud->at(col, row);
            const auto intensity = pt.intensity;
            const unsigned idx = row * cloud->width + col;

            if (intensity < m_raycast_min_intensity || !m_sensor_mask.at(idx))
              continue;

            const vec3_t dir1 = m_sensor_xyz_lut.directions.col(idx);
            const vec3_t dir = tf_rot*dir1;

            constexpr float range_to_meters = 0.001f;
            const float max_dist = (float)m_drmgr_ptr->config.raycast__max_distance;
            const float ray_dist = range_to_meters*float(pt.range);
            const float dist = ray_dist == 0.0f ? max_dist : std::min(ray_dist - m_vmap_voxel_size, max_dist);

            /* some checks (unused) //{ */
            
            /* const vec3_t pt_dir = (pt.getVector3fMap() - m_sensor_xyz_lut.offsets.col(idx)).normalized(); */
            /* const float pt_dist = (pt.getVector3fMap() - m_sensor_xyz_lut.offsets.col(idx)).norm(); */
            /* if (ray_dist > 0.0f) */
            /* { */
            /*   if ((pt_dir - dir1).norm() > 1e-3f) */
            /*     ROS_ERROR("[PCLDetector]: Point dir ([%f, %f, %f]m) and LUT dir ([%f, %f, %f]m) are different (diff: %fm)!", pt_dir.x(), pt_dir.y(), pt_dir.z(), dir1.x(), dir1.y(), dir1.z(), (pt_dir - dir1).norm()); */
            /*   if (std::abs(pt_dist - dist) > 1e-3f) */
            /*     ROS_ERROR("[PCLDetector]: Point dist (%fm) and range (%fm) are different!", pt_dist, ray_dist); */
            /*   if (1.0f - dir1.norm() > 1e-3f) */
            /*     ROS_ERROR("[PCLDetector]: Direction from LUT is not normalized (norm is %fm)!", dir1.norm()); */
            /*   if (1.0f - dir.norm() > 1e-3f) */
            /*     ROS_ERROR("[PCLDetector]: Transformed direction is not normalized (norm is %fm)!", dir.norm()); */
            /* } */
            
            //}

            const vec3_t start_pt = tf_rot*m_sensor_xyz_lut.offsets.col(idx) + origin_pt;
      
            // TODO: reduce to just one check with max. ray offset
            // check if the ray origin including the ray offset (intrinsic mechanical sensor parameter)
            // is still within bounds of the operational area
            if (m_voxel_raycast.inLimits(start_pt.x(), start_pt.y(), start_pt.z()))
            {
              m_voxel_raycast.forEachRay(start_pt, dir, dist,
                  [this](const VoxelMap::coord_t val, const VoxelMap::idx_t x_idx, const VoxelMap::idx_t y_idx, const VoxelMap::idx_t z_idx)
                  {
                    auto& raycastval = m_voxel_raycast.atIdx(x_idx, y_idx, z_idx);
                    raycastval += val;
                  });
            }
          }
        }

        // publish the visualization if requested
        if (publish)
        {
          std_msgs::Header header;
          header.frame_id = m_world_frame_id;
          pcl_conversions::fromPCL(cloud->header.stamp, header.stamp);
          m_pub_lidar_raycast.publish(m_voxel_raycast.visualization(header));
        }

        if (m_pub_lidar_fov.getNumSubscribers() > 0)
        {
          std_msgs::Header header;
          header.frame_id = cloud->header.frame_id;
          pcl_conversions::fromPCL(cloud->header.stamp, header.stamp);
          std::vector<float> lengths(cloud->size(), 0);
          for (int idx = 0; idx < (int)cloud->size(); idx++)
          {
            const auto pt = cloud->at(idx);
            const auto intensity = pt.intensity;
            if (intensity < m_raycast_min_intensity || !m_sensor_mask.at(idx))
              continue;
            auto range = 0.001f*pt.range;
            if (range == 0)
              range = m_drmgr_ptr->config.raycast__max_distance;
            lengths.at(idx) = range;
          }
          const auto msg = lidar_visualization(header, lengths);
          m_pub_lidar_fov.publish(msg);
        }
      } else
      {
        NODELET_ERROR_THROTTLE(1.0, "[RaycastCloud]: Sensor is outside of the operational area! Cannot update map (detection may misbehave).");
      }
      tim.checkpoint("raycasting");

      // wait for at least one detection to finish (no use raycasting if detection_its_diff is zero)
      std::unique_lock lck(m_voxels_mtx);
      if (
          m_detection_its == start_detection_its // first check if it's even necessary to wait (if this condition is false, the waiting will be skipped)
       && (m_detection_cv.wait_for(lck, std::chrono::milliseconds(800)) == std::cv_status::timeout || m_detection_its == start_detection_its)) // then try waiting and re-check if we got more detections
      {
        NODELET_ERROR_THROTTLE(1.0, "[RaycastCloud]: Raycast thread timed out after 0.8s waiting for a detection to finish. Ending.");
        return;
      }
      tim.checkpoint("wait for dets");
      const float detection_its_diff = m_detection_its - start_detection_its;
      
      // the maximal value in the helper voxelmap will be used to normalize the voxelmap to values in the range <0; 1>
      const float max_val = *std::max_element(std::begin(m_voxel_raycast), std::end(m_voxel_raycast));
      // if max_val is zero, there's nothing to update
      if (max_val == 0.0f)
      {
        NODELET_ERROR_THROTTLE(1.0, "[RaycastCloud]: Maximum raycasted value is zero. This is strange (empty pointcloud?). Skipping raycast.");
        return;
      }

      // update the main voxel map using values from the helper voxelmap
      const float ray_update_score = m_drmgr_ptr->config.voxel_map__scores__ray;        // this is the value that voxels will converge to if consistently raycasted as empty
      const float ray_update_weight = m_drmgr_ptr->config.raycast__weight_coefficient;  // used to slow down or speed up the convergence
      m_voxel_flags.forEachIdx(
            [detection_its_diff, ray_update_score, max_val, ray_update_weight, this](VoxelMap::data_t& flag, const VoxelMap::idx_t xc, const VoxelMap::idx_t yc, const VoxelMap::idx_t zc)
            {
              float raycastval;
              auto& mapval = m_voxel_map.atIdx(xc, yc, zc);
              if (flag == m_vflags_unmarked && (raycastval = m_voxel_raycast.atIdx(xc, yc, zc)) > 0.0f)
              {
                // normalize the raycast values to range <0; 1>
                const float norm_val = raycastval/max_val;
                // apply the non-linear transformation (fancy name for square root) and user-set weight
                // the square root ensures that close voxels with significantly higher value than far voxels are not updated too aggressively
                // (this counters the much higher ray density in around the sensors)
                const float w_update_single = ray_update_weight*std::sqrt(norm_val);
                // calculate weights as if the update was done detection_its_diff-times
                const float w1 = std::clamp(std::pow(1.0f - w_update_single, detection_its_diff), 0.0f, 1.0f);
                const float w2 = 1.0f - w1;
                /* if (w1 < 0.0f || w2 < 0.0f) */
                /*   ROS_ERROR("[PCLDetector]: Invalid weight: w1 = %.2f, w2 = %.2f!", w1, w2); */
                mapval = w1*mapval + w2*ray_update_score;
              }
            }
          );
      m_voxel_flags.clear();
      tim.checkpoint("vmap update");
    }
    //}

    /* reset() method //{ */

    void reset()
    {
      std::scoped_lock lck(m_voxels_mtx);
      if (m_raycast_thread.joinable())
        m_raycast_thread.join();

      m_voxel_map.resize(m_oparea_offset_x, m_oparea_offset_y, m_oparea_offset_z, m_oparea_size_x, m_oparea_size_y, m_oparea_size_z, m_vmap_voxel_size);
      m_voxel_map.setTo(m_vmap_init_score);
      const auto [vmap_size_x, vmap_size_y, vmap_size_z] = m_voxel_map.sizesIdx();
      NODELET_INFO_STREAM_THROTTLE(1.0, "[PCLDetector]: Voxel map reset to [" << vmap_size_x << ", " << vmap_size_y << ", " << vmap_size_z << "] size.");

      m_voxel_flags.resizeAs(m_voxel_map);
      m_voxel_flags.addVisualizationThreshold(m_vflags_point-0.1, m_vflags_color_background);
      m_voxel_flags.addVisualizationThreshold(m_vflags_unknown-0.1, m_vflags_color_unknown);
      m_voxel_flags.clear();

      m_voxel_raycast.resizeAs(m_voxel_map);
      m_voxel_raycast.addVisualizationThreshold(m_vflags_point-0.1, m_vflags_color_background);
      m_voxel_raycast.addVisualizationThreshold(m_vflags_unknown-0.1, m_vflags_color_unknown);
      m_detection_its = 0;

      NODELET_WARN_THROTTLE(1.0, "[PCLDetector]: Voxelmaps reset!");
    }

    //}

    /* cluster_centroid() method //{ */
    pt_XYZ_t cluster_centroid(const pc_XYZ_t& cloud, const PointIndices& cluster_indices)
    {
      vec4_t centroid;
      pcl::compute3DCentroid(cloud, cluster_indices.indices, centroid);
      pt_XYZ_t ret;
      ret.getVector4fMap() = centroid;
      return ret;
    }
    //}

    /* classify_cluster() method //{ */
    cluster_t classify_cluster(const pc_XYZR_t::ConstPtr& cloud, const PointIndices::ConstPtr& cluster_indices, const Eigen::Affine3f& s2w_tf)
    {
      // TODO: deal with voxels at the edge of the map separately (should never be classified as detections)
      cluster_t ret;
      ret.cclass = cluster_class_t::invalid;

      {
        pcl::MomentOfInertiaEstimation<pt_XYZR_t> moie;
        moie.setInputCloud(cloud);
        moie.setIndices(cluster_indices);
        moie.compute();
        pt_XYZR_t min_pt;
        pt_XYZR_t max_pt;
        pt_XYZR_t ctr_pt;

        // get the axis-aligned bounding box
        moie.getAABB(min_pt, max_pt);
        ret.aabb.min_pt = min_pt.getVector3fMap();
        ret.aabb.max_pt = max_pt.getVector3fMap();

        // get the oriented bounding box
        moie.getOBB(min_pt, max_pt, ctr_pt, ret.obb.orientation);
        ret.obb.min_pt = min_pt.getVector3fMap();
        ret.obb.max_pt = max_pt.getVector3fMap();
        ret.obb.center_pt = ctr_pt.getVector3fMap();
      }

      ret.pc = cloud;
      ret.pc_indices = cluster_indices;

      // skip clusters with too few points
      if ((int)cluster_indices->indices.size() < m_drmgr_ptr->config.classification__min_points)
        return ret;

      // skip clusters that are too distant
      const double dist = (s2w_tf.translation() - ret.obb.center_pt).norm();
      if (dist > m_drmgr_ptr->config.classification__max_distance)
        return ret;

      // skip too large clusters
      ret.obb_size = (ret.obb.max_pt - ret.obb.min_pt).norm();
      if (ret.obb_size > m_drmgr_ptr->config.classification__max_size)
        return ret;

      bool is_floating = true;
      // if ground is available, check floatingness of the cluster (otherwise it cannot be decided)
      if (m_background_pts_sufficient && m_sure_background_sufficient)
      {
        const int max_explore_voxel_size = (ret.obb_size + m_drmgr_ptr->config.classification__max_explore_distance)/m_vmap_voxel_size;
        // check if every point in the cluster is floating (all points in a 26-neighborhood
        // are known to be empty -> their value is <= m_drmgr_ptr->config.voxel_map__thresholds__frontiers)
        for (const auto idx : cluster_indices->indices)
        {
          const auto& pt = cloud->at(idx);
          const auto [is_connected, explored_idxs] = m_voxel_map.exploreToGround(pt.x, pt.y, pt.z, m_drmgr_ptr->config.voxel_map__thresholds__frontiers, m_drmgr_ptr->config.voxel_map__thresholds__new_obstacles, max_explore_voxel_size);
          // if it's connected to ground through unknown voxels, we're done here
          if (is_connected)
          {
            is_floating = false;
            break;
          }
          // if it's not connected to ground, update all the unknown voxels to air!
          else
          {
            for (const auto& idx : explored_idxs)
            {
              m_voxel_map.at(idx) = m_drmgr_ptr->config.voxel_map__thresholds__frontiers;
            }
          }
        }
      }
      else
      {
        is_floating = false;
      }


      if (is_floating)
        ret.cclass = cluster_class_t::mav;
      else
        ret.cclass = cluster_class_t::unknown;
      return ret;
    }
    //}

    /* /1* set_cov() methods //{ *1/ */
    /* using msg_cov_t = geometry_msgs::PoseWithCovarianceStamped::_pose_type::_covariance_type; */
    /* void set_cov(const Eigen::Matrix3f& e_cov, msg_cov_t& cov, int start_idx) */
    /* { */
    /*   for (unsigned r = start_idx; r < (unsigned)start_idx + 3; r++) */
    /*   { */
    /*     for (unsigned c = start_idx; c < (unsigned)start_idx + 3; c++) */
    /*     { */
    /*       cov[r * 6 + c] = e_cov(r - start_idx, c - start_idx); */
    /*     } */
    /*   } */
    /* } */
    /* void set_pos_cov(const Eigen::Matrix3f& e_cov, msg_cov_t& cov) */
    /* { */
    /*   set_cov(e_cov, cov, 0); */
    /* } */
    /* void set_rot_cov(const Eigen::Matrix3f& e_cov, msg_cov_t& cov) */
    /* { */
    /*   set_cov(e_cov, cov, 3); */
    /* } */
    /* //} */

    /* rotate_covariance() method //{ */
    Eigen::Matrix3f rotate_covariance(const Eigen::Matrix3f& covariance, const Eigen::Matrix3f& rotation)
    {
      return rotation * covariance * rotation.transpose();  // rotate the covariance to point in direction of est. position
    }
    //}

    /* to_pcl() method //{ */
    pt_XYZ_t to_pcl(const vec3_t& pt)
    {
      return {pt.x(), pt.y(), pt.z()};
    }
    //}

    /* to_marker_list_msg() method and helpers//{ */
    geometry_msgs::Point pcl2gmpt(const pcl::PointXYZ& pt0)
    {
      geometry_msgs::Point ret;
      ret.x = pt0.x;
      ret.y = pt0.y;
      ret.z = pt0.z;
      return ret;
    }

    using marker_pts_t = visualization_msgs::Marker::_points_type;
    void fill_marker_pts_lines(const pcl::Vertices& mesh_verts, const pc_XYZ_t& mesh_cloud, marker_pts_t& marker_pts)
    {
      geometry_msgs::Point prev_pt;
      bool prev_pt_set = false;
      for (const auto vert : mesh_verts.vertices)
      {
        const auto idx = vert;
        const auto pt = mesh_cloud.at(idx);
        const geometry_msgs::Point gmpt = pcl2gmpt(pt);
        if (prev_pt_set)
        {
          marker_pts.push_back(prev_pt);
          marker_pts.push_back(gmpt);
        }
        prev_pt = gmpt;
        prev_pt_set = true;
      }
      if (prev_pt_set)
      {
        marker_pts.push_back(prev_pt);
        const auto idx = mesh_verts.vertices.at(0);
        const auto pt = mesh_cloud.at(idx);
        const geometry_msgs::Point gmpt = pcl2gmpt(pt);
        marker_pts.push_back(gmpt);
      }
    }

    void fill_marker_pts_triangles(const pcl::Vertices& mesh_verts, const pc_XYZ_t& mesh_cloud, marker_pts_t& marker_pts)
    {
      if (mesh_verts.vertices.size() != 3)
        return;
      for (const auto vert : mesh_verts.vertices)
      {
        const auto idx = vert;
        const auto pt = mesh_cloud.at(idx);
        const geometry_msgs::Point gmpt = pcl2gmpt(pt);
        marker_pts.push_back(gmpt);
      }
    }

    visualization_msgs::Marker to_marker_list_msg(const pcl::PolygonMesh& mesh)
    {
      visualization_msgs::Marker ret;
      ret.header.frame_id = mesh.header.frame_id;
      pcl_conversions::fromPCL(mesh.header.stamp, ret.header.stamp);
      ret.ns = "uav_detect/mesh";
      ret.id = 666;
      ret.action = visualization_msgs::Marker::ADD;
      ret.lifetime = ros::Duration(0.0);
      ret.color.a = ret.color.r = ret.color.g = ret.color.b = 1.0;
      ret.scale.x = ret.scale.y = ret.scale.z = 1.0;
      if (mesh.polygons.empty())
        return ret;

      const auto n_verts = mesh.polygons.at(0).vertices.size();
      if (n_verts == 3)
      {
        ret.type = visualization_msgs::Marker::TRIANGLE_LIST;
      } else
      {
        ret.scale.x = ret.scale.y = ret.scale.z = 0.1;
        ret.type = visualization_msgs::Marker::LINE_LIST;
      }
      ret.points.reserve(mesh.polygons.size() * n_verts);
      pc_XYZ_t mesh_cloud;
      pcl::fromPCLPointCloud2(mesh.cloud, mesh_cloud);
      for (const auto& vert : mesh.polygons)
      {
        if (n_verts == 3)
        {
          if (vert.vertices.size() != n_verts)
            ROS_WARN_THROTTLE(0.1, "[PCLDetector]: Number of vertices in mesh is incosistent (expected: %lu, got %lu)!", n_verts, vert.vertices.size());
          fill_marker_pts_triangles(vert, mesh_cloud, ret.points);
        } else
          fill_marker_pts_lines(vert, mesh_cloud, ret.points);
      }
      /* ret.colors; */
      return ret;
    }
    //}

    /* valid_pt() method //{ */
    template <class Point_T>
    bool valid_pt(Point_T pt)
    {
      return (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z));
    }
    //}

    /* get_transform_to_world() method //{ */
    bool get_transform_to_world(const std::string& frame_id, ros::Time stamp, Eigen::Affine3d& tf_out) const
    {
      try
      {
        const ros::Duration timeout(m_transform_lookup_timeout);
        // Obtain transform from sensor into world frame
        geometry_msgs::TransformStamped transform;
        transform = m_tf_buffer.lookupTransform(m_world_frame_id, frame_id, stamp, timeout);
        tf_out = tf2::transformToEigen(transform.transform);
      }
      catch (tf2::TransformException& ex)
      {
        NODELET_WARN_THROTTLE(1.0, "[%s]: Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", m_node_name.c_str(), frame_id.c_str(),
                              m_world_frame_id.c_str(), ex.what());
        return false;
      }
      return true;
    }
    //}

    /* /1* coords_from_idx() method //{ *1/ */

    /* vec3_t coords_from_idx(const int coord) */
    /* { */
    /*   const float x = (coord % int(std::round(m_arena_bbox_size_y * m_arena_bbox_size_z))) + m_arena_bbox_offset_x/2.0; */
    /*   const float y = ((coord -  % int(std::round(m_arena_bbox_size_z))) + m_arena_bbox_offset_y/2.0; */
    /*   const float z = (coord % int(std::round(m_arena_bbox_size_y * m_arena_bbox_size_z))) + m_arena_bbox_offset_z; */
    /*   return {x, y, z}; */
    /* } */

    /* //} */

    /* check_sensor_params() method //{ */
    bool check_sensor_params(const pc_t::ConstPtr cloud)
    {
      std::scoped_lock lck(m_sensor_params_mtx);
      bool found_valid = false;
      bool params_ok = true;
      constexpr float range_to_meters = 0.001f;
      for (int row = 0; row < (int)cloud->height && !found_valid; row++)
      {
        for (int col = 0; col < (int)cloud->width && !found_valid; col++)
        {
          const auto pt = cloud->at(col, row);
          const unsigned idx = row * cloud->width + col;
    
          // ignore invalid points
          if (!m_sensor_mask.at(idx) || pt.range == 0)
            continue;
    
          const vec3_t lut_dir = m_sensor_xyz_lut.directions.col(idx);
          const float lut_dist = range_to_meters*float(pt.range);
    
          const vec3_t pt_dir = (pt.getVector3fMap() - m_sensor_xyz_lut.offsets.col(idx)).normalized();
          const float pt_dist = (pt.getVector3fMap() - m_sensor_xyz_lut.offsets.col(idx)).norm();
    
          if ((pt_dir - lut_dir).norm() > 1e-3f)
          {
            ROS_ERROR("[PCLDetector]: Point dir #%u [%f, %f, %f]m and LUT dir #%u [%f, %f, %f]m are different (diff: %fm)!", idx, pt_dir.x(), pt_dir.y(), pt_dir.z(), idx, lut_dir.x(), lut_dir.y(), lut_dir.z(), (pt_dir - lut_dir).norm());
            params_ok = false;
          }
          if (std::abs(pt_dist - lut_dist) > 1e-3f)
          {
            ROS_ERROR("[PCLDetector]: Point dist #%u %fm and range #%u %fm are different!", idx, pt_dist, idx, lut_dist);
            params_ok = false;
          }
          if (1.0f - lut_dir.norm() > 1e-3f)
          {
            ROS_ERROR("[PCLDetector]: Direction from LUT is not normalized (norm is %fm)!", lut_dir.norm());
            params_ok = false;
          }
    
          found_valid = true;
        }
      }
      if (found_valid)
      {
        m_sensor_params_checked = true;
        m_sensor_params_ok = params_ok;
      }
      return params_ok;
    }
    //}

    /* clusters_visualization() method //{ */
    geometry_msgs::Point from_eigen(const vec3_t& pt)
    {
      geometry_msgs::Point ret;
      ret.x = pt.x();
      ret.y = pt.y();
      ret.z = pt.z();
      return ret;
    }

    visualization_msgs::MarkerArray clusters_visualization(const std::vector<cluster_t>& clusters, const std_msgs::Header& header)
    {
      visualization_msgs::MarkerArray ret;

      {
        visualization_msgs::Marker det_obbs;
        det_obbs.header = header;
        det_obbs.pose.orientation.w = 1.0;
        det_obbs.color.a = 1.0;
        det_obbs.color.r = 1.0;
        det_obbs.ns = "detection oriented bounding boxes";
        det_obbs.id = 123;
        det_obbs.type = visualization_msgs::Marker::LINE_LIST;
        det_obbs.points.reserve(12 * clusters.size());
        det_obbs.scale.x = 0.1;

        visualization_msgs::Marker other_obbs;
        other_obbs.header = header;
        other_obbs.pose.orientation.w = 1.0;
        other_obbs.color.a = 0.5;
        other_obbs.color.b = 1.0;
        other_obbs.ns = "other oriented bounding boxes";
        other_obbs.id = 124;
        other_obbs.type = visualization_msgs::Marker::LINE_LIST;
        other_obbs.points.reserve(12 * clusters.size());
        other_obbs.scale.x = 0.1;

        visualization_msgs::Marker inv_obbs;
        inv_obbs.header = header;
        inv_obbs.pose.orientation.w = 1.0;
        inv_obbs.color.a = 0.3;
        inv_obbs.color.g = 1.0;
        inv_obbs.ns = "invalid clusters";
        inv_obbs.id = 125;
        inv_obbs.type = visualization_msgs::Marker::LINE_LIST;
        inv_obbs.points.reserve(12 * clusters.size());
        inv_obbs.scale.x = 0.03;

        /*  //{ */
        
        for (const auto& cluster : clusters)
        {
          /* if (cluster.cclass == cluster_class_t::invalid) */
          /*   continue; */
        
          const geometry_msgs::Point A =
              from_eigen(cluster.obb.orientation * vec3_t(cluster.obb.min_pt.x(), cluster.obb.min_pt.y(), cluster.obb.min_pt.z()) + cluster.obb.center_pt);
          const geometry_msgs::Point B =
              from_eigen(cluster.obb.orientation * vec3_t(cluster.obb.max_pt.x(), cluster.obb.min_pt.y(), cluster.obb.min_pt.z()) + cluster.obb.center_pt);
          const geometry_msgs::Point C =
              from_eigen(cluster.obb.orientation * vec3_t(cluster.obb.max_pt.x(), cluster.obb.max_pt.y(), cluster.obb.min_pt.z()) + cluster.obb.center_pt);
          const geometry_msgs::Point D =
              from_eigen(cluster.obb.orientation * vec3_t(cluster.obb.min_pt.x(), cluster.obb.max_pt.y(), cluster.obb.min_pt.z()) + cluster.obb.center_pt);
        
          const geometry_msgs::Point E =
              from_eigen(cluster.obb.orientation * vec3_t(cluster.obb.min_pt.x(), cluster.obb.min_pt.y(), cluster.obb.max_pt.z()) + cluster.obb.center_pt);
          const geometry_msgs::Point F =
              from_eigen(cluster.obb.orientation * vec3_t(cluster.obb.max_pt.x(), cluster.obb.min_pt.y(), cluster.obb.max_pt.z()) + cluster.obb.center_pt);
          const geometry_msgs::Point G =
              from_eigen(cluster.obb.orientation * vec3_t(cluster.obb.max_pt.x(), cluster.obb.max_pt.y(), cluster.obb.max_pt.z()) + cluster.obb.center_pt);
          const geometry_msgs::Point H =
              from_eigen(cluster.obb.orientation * vec3_t(cluster.obb.min_pt.x(), cluster.obb.max_pt.y(), cluster.obb.max_pt.z()) + cluster.obb.center_pt);
        
          // point the pts reference to the correct vector based on the class
          std::vector<geometry_msgs::Point>& pts = 
            cluster.cclass == cluster_class_t::unknown ? // if the class is "unknown", point it to other_obbs.points
              other_obbs.points :
                (cluster.cclass == cluster_class_t::invalid ? // if the class is "invalid", point it to inv_obbs.points
                 inv_obbs.points :
                 det_obbs.points); // otherwise the class is "mav", point it to det_obbs.points
        
          pts.push_back(A);
          pts.push_back(B);
          pts.push_back(B);
          pts.push_back(C);
          pts.push_back(C);
          pts.push_back(D);
          pts.push_back(D);
          pts.push_back(A);
        
          pts.push_back(E);
          pts.push_back(F);
          pts.push_back(F);
          pts.push_back(G);
          pts.push_back(G);
          pts.push_back(H);
          pts.push_back(H);
          pts.push_back(E);
        
          pts.push_back(A);
          pts.push_back(E);
          pts.push_back(B);
          pts.push_back(F);
          pts.push_back(C);
          pts.push_back(G);
          pts.push_back(D);
          pts.push_back(H);
        }
        
        //}

        if (det_obbs.points.empty())
          det_obbs.action = visualization_msgs::Marker::DELETE;
        if (other_obbs.points.empty())
          other_obbs.action = visualization_msgs::Marker::DELETE;
        if (inv_obbs.points.empty())
          inv_obbs.action = visualization_msgs::Marker::DELETE;

        ret.markers.push_back(det_obbs);
        ret.markers.push_back(other_obbs);
        ret.markers.push_back(inv_obbs);
      }

      return ret;
    }
    //}

    /* frontier_visualization() method //{ */
    visualization_msgs::MarkerArray frontier_visualization(const std::vector<cluster_t>& clusters, const std_msgs::Header& header)
    {
      visualization_msgs::MarkerArray ret;
    
      static int prev_max_id = 0;
      int id = 0;
      for (const auto& cluster : clusters)
      {
        if (cluster.cclass == cluster_class_t::invalid || cluster.cclass == cluster_class_t::mav)
          continue;

        auto mkr = cluster.submap.visualization(header);
        if (!mkr.points.empty())
        {
          mkr.color.a = 0.2;
          mkr.id = id++;
          ret.markers.push_back(mkr);
        }
        mkr = cluster.submap.borderVisualization(header);
        mkr.id = id++;
        ret.markers.push_back(mkr);
      }

      // delete previous markers if necessary
      for (int it = id; it < prev_max_id; it++)
      {
        visualization_msgs::Marker mkr;
        mkr.header = header;
        mkr.pose.orientation.w = 1.0;
        mkr.type = visualization_msgs::Marker::CUBE_LIST;
        mkr.action = visualization_msgs::Marker::DELETE;
        mkr.id = it;
        ret.markers.push_back(mkr);
      }
      prev_max_id = id;
    
      return ret;
    }
    //}

    /* lidar_visualization() method //{ */
    // from https://www.codespeedy.com/hsv-to-rgb-in-cpp/
    std::tuple<float, float, float> HSVtoRGB(float H, float S, float V)
    {
      const float s = S/100.0f;
      const float v = V/100.0f;
      const float C = s*v;
      const float X = C*(1-std::abs(std::fmod(H/60.0f, 2)-1));
      const float m = v-C;
      float r, g, b;
      if (H >= 0 && H < 60)
          r = C, g = X, b = 0;
      else if (H >= 60 && H < 120)
          r = X, g = C, b = 0;
      else if (H >= 120 && H < 180)
          r = 0, g = C, b = X;
      else if (H >= 180 && H < 240)
          r = 0, g = X, b = C;
      else if (H >= 240 && H < 300)
          r = X, g = 0, b = C;
      else
          r = C, g = 0, b = X;
      const float R = (r+m)*255;
      const float G = (g+m)*255;
      const float B = (b+m)*255;
      return {R, G, B};
    }

    visualization_msgs::Marker lidar_visualization(const std_msgs::Header& header, const std::vector<float>& lengths)
    {
      visualization_msgs::Marker ret;
      if (!m_sensor_initialized)
        return ret;

      ret.header = header;
      ret.color.a = 0.1;
      ret.color.r = 1.0;
      ret.scale.x = 0.01;
      ret.ns = "lidar FOV";
      ret.pose.orientation.w = 1;
      ret.pose.orientation.x = 0;
      ret.pose.orientation.y = 0;
      ret.pose.orientation.z = 0;
      ret.pose.position.x = 0;
      ret.pose.position.y = 0;
      ret.pose.position.z = 0;

      ret.type = visualization_msgs::Marker::LINE_LIST;

      ret.points.reserve(m_sensor_xyz_lut.directions.cols()*2);
      for (size_t it = 0; it < lengths.size(); it++)
      {
        const auto off = m_sensor_xyz_lut.offsets.col(it);
        const auto vec = m_sensor_xyz_lut.directions.col(it);
        const auto len = lengths.at(it);
        {
          geometry_msgs::Point pt0;
          pt0.x = off.x();
          pt0.y = off.y();
          pt0.z = off.z();
          ret.points.push_back(pt0);
        }
        {
          geometry_msgs::Point pt;
          pt.x = off.x() + len*vec.x();
          pt.y = off.y() + len*vec.y();
          pt.z = off.z() + len*vec.z();
          ret.points.push_back(pt);
        }
        const auto [R, G, B] = HSVtoRGB(360.0f*it/float(lengths.size()), 100.0f, 100.0f);
        std_msgs::ColorRGBA cur_color;
        cur_color.r = R/255;
        cur_color.g = G/255;
        cur_color.b = B/255;
        cur_color.a = 1.0;
        ret.colors.push_back(cur_color);
        ret.colors.push_back(cur_color);
      }

      return ret;
    }

    visualization_msgs::Marker lidar_visualization(const std_msgs::Header& header)
    {
      std::vector<float> lengths(m_sensor_xyz_lut.directions.cols());
      std::fill(std::begin(lengths), std::end(lengths), 2.0f);
      return lidar_visualization(header, lengths);
    }
    //}

  private:
    // --------------------------------------------------------------
    // |                ROS-related member variables                |
    // --------------------------------------------------------------

    /* ROS related variables (subscribers, timers etc.) //{ */
    std::unique_ptr<drmgr_t> m_drmgr_ptr;
    tf2_ros::Buffer m_tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
    mrs_lib::SubscribeHandler<pc_t> m_sh_pc;
    mrs_lib::SubscribeHandler<sensor_msgs::Range> m_sh_rangefinder;

    ros::Publisher m_pub_vmap;
    ros::Publisher m_pub_update_flags;
    ros::Publisher m_pub_oparea;

    ros::Publisher m_pub_filtered_input_pc;
    ros::Publisher m_pub_weighted_input_pc;
    ros::Publisher m_pub_background_pc;
    ros::Publisher m_pub_apriori_pc;
    ros::Publisher m_pub_background_clusters_pc;
    ros::Publisher m_pub_sure_air_pc;
    ros::Publisher m_pub_sepclusters_pc;

    ros::Publisher m_pub_detections_pc;
    ros::Publisher m_pub_detections_mks;
    ros::Publisher m_pub_frontiers_mks;

    ros::Publisher m_pub_classif_max_dist;
    ros::Publisher m_pub_detections_dbg;
    ros::Publisher m_pub_detections;
    ros::Publisher m_pub_status;

    ros::Publisher m_pub_lidar_fov;
    ros::Publisher m_pub_lidar_raycast;

    /* ros::Publisher m_pub_evil_voxels; */

    ros::ServiceServer m_reset_server;

    ros::Timer m_main_loop_timer;
    ros::Timer m_info_loop_timer;
    std::string m_node_name;
    //}

  private:
    // --------------------------------------------------------------
    // |                 Parameters, loaded from ROS                |
    // --------------------------------------------------------------

    /* Parameters, loaded from ROS //{ */

    std::string m_world_frame_id;
    ros::Duration m_transform_lookup_timeout;
    ros::Duration m_bgclusters_period;
    int m_n_pc_threads;

    float m_raycast_min_intensity;
    std::string m_sensor_mask_fname;
    int m_sensor_mask_rows;
    bool m_sensor_simulation;

    float m_vmap_voxel_size;
    float m_vmap_init_score;
    float m_vmap_threshold_apriori_map;

    std_msgs::ColorRGBA m_vmap_color_apriori_map;
    std_msgs::ColorRGBA m_vmap_color_new_obstacles;
    std_msgs::ColorRGBA m_vmap_color_sure_obstacles;
    std_msgs::ColorRGBA m_vmap_color_frontiers;
    std_msgs::ColorRGBA m_vmap_color_candidates;

    std_msgs::ColorRGBA m_vflags_color_background;
    std_msgs::ColorRGBA m_vflags_color_unknown;

    float m_exclude_box_offset_x;
    float m_exclude_box_offset_y;
    float m_exclude_box_offset_z;
    float m_exclude_box_size_x;
    float m_exclude_box_size_y;
    float m_exclude_box_size_z;

    float m_oparea_offset_x;
    float m_oparea_offset_y;
    float m_oparea_offset_z;
    float m_oparea_size_x;
    float m_oparea_size_y;
    float m_oparea_size_z;

    ros::Duration m_throttle_period;

    //}

  private:
    // --------------------------------------------------------------
    // |                  Sensor-related variables                  |
    // --------------------------------------------------------------

    xyz_lut_t m_sensor_xyz_lut;
    std::vector<uint8_t> m_sensor_mask;
    float m_sensor_vfov;
    int m_sensor_vrays;
    int m_sensor_hrays;

  private:
    // --------------------------------------------------------------
    // |                   Other member variables                   |
    // --------------------------------------------------------------

    bool m_apriori_map_initialized;
    bool m_sensor_initialized;
    std::mutex m_sensor_params_mtx;
    bool m_sensor_params_checked;
    bool m_sensor_params_ok;
    uint32_t m_last_detection_id;
    std::atomic<bool> m_background_pts_sufficient;
    std::atomic<bool> m_sure_background_sufficient;
    uint64_t m_background_min_sufficient_pts;

    std::mutex m_voxels_mtx;
    std::condition_variable m_detection_cv;
    std::thread m_raycast_thread;
    std::atomic<bool> m_raycast_running;
    std::atomic<int> m_detection_its;
    VoxelMap m_voxel_map;

    static constexpr float m_vflags_unmarked = 0.0f;
    static constexpr float m_vflags_point = 2.0f;
    static constexpr float m_vflags_unknown = 3.0f;
    VoxelMap m_voxel_flags;
    VoxelMap m_voxel_raycast;

  };  // class PCLDetector
};    // namespace uav_detect

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uav_detect::PCLDetector, nodelet::Nodelet)
