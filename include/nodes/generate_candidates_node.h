#ifndef GENERATE_CANDIDATES_NODE_H_
#define GENERATE_CANDIDATES_NODE_H_

// system
#include <algorithm>
#include <vector>
#include <memory>

// ROS
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// GPG
#include <gpg/cloud_camera.h>

// GPG messages
#include <gpg/CloudSamples.h>
#include <gpg/GraspConfig.h>
#include <gpg/GraspConfigList.h>


typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;

class GenerateCandidatesNode
{
public:

    GenerateCandidatesNode(ros::NodeHandle& node);

    ~GenerateCandidatesNode()
    {}

    void run();

private:

    /**
   * \brief Callback function for the ROS topic that contains the input point cloud and a list of (x,y,z) samples.
   * \param msg the incoming ROS message
  */
    void cloud_samples_callback(const gpg::CloudSamples& msg);
    void cloud_pcd_callback(const sensor_msgs::PointCloud2);
    std::vector<Grasp> generate_candidates_from_cloud_camera(CloudCamera cloud_cam);
    std::shared_ptr<CandidatesGenerator> make_default_candidates_generator();
    gpg::GraspConfig convertToGraspMsg(const Grasp& hand);
    gpg::GraspConfigList createGraspListMsg(const std::vector<Grasp>& hands);

    int size_left_cloud_; ///< (input) size of the left point cloud (when using two point clouds as input)
    bool has_cloud_, has_normals_, has_samples_; ///< status variables for received (input) messages
    std::string frame_; ///< point cloud frame
    ros::Subscriber cloud_sub_; ///< ROS subscriber for point cloud messages
    ros::Subscriber samples_sub_; ///< ROS subscriber for samples messages
    ros::Publisher grasps_pub_; ///< ROS publisher for grasp list messages
    ros::Publisher grasps_rviz_pub_; ///< ROS publisher for grasps in rviz (visualization)

    bool use_importance_sampling_; ///< if importance sampling is used
    bool filter_grasps_; ///< if grasps are filtered on workspace and gripper aperture
    bool filter_half_antipodal_; ///< if half-antipodal grasps are filtered
    bool plot_filtered_grasps_; ///< if filtered grasps are plotted
    bool plot_selected_grasps_; ///< if selected grasps are plotted
    bool plot_normals_; ///< if normals are plotted
    bool plot_samples_; ///< if samples/indices are plotted
    bool use_rviz_; ///< if rviz is used for visualization instead of PCL
    int num_selected_; ///< number of selected highest-scoring grasp clusters
    std::vector<double> workspace_; ///< workspace limits

    std::shared_ptr<CloudCamera> cloud_camera_;
    std::shared_ptr<CandidatesGenerator> candidates_generator_; ///< used to run the grasp pose detection
    // Set the camera pose.
    Eigen::Matrix3Xd camera_1_view_point_;


    /** constants for input point cloud types */
    static const int POINT_CLOUD_2; ///< sensor_msgs/PointCloud2
    static const int CLOUD_INDEXED; ///< gpd/CloudIndexed
    static const int CLOUD_SAMPLES; ///< gpd/CloudSamples
};


#endif /* GRASP_DETECTION_NODE_H_ */
