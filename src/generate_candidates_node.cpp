// System
#include <sstream>
#include <string>
#include <vector>

// Custom
#include <gpg/candidates_generator.h>
#include <gpg/hand_search.h>
#include <gpg/config_file.h>

#include "../../gpg/include/nodes/generate_candidates_node.h"


// function to read in a double array from a single line of a configuration file
//std::vector<double> stringToDouble(const std::string &str) {
//    std::vector<double> values;
//    std::stringstream ss(str);
//    double v;
//
//    while (ss >> v) {
//        values.push_back(v);
//        if (ss.peek() == ' ') {
//            ss.ignore();
//        }
//    }
//
//    return values;
//}

GenerateCandidatesNode::GenerateCandidatesNode(ros::NodeHandle &node) : has_cloud_(false), has_normals_(false),
                                                                        size_left_cloud_(0), has_samples_(true),
                                                                        frame_("") {

    std::cout << "test" << std::endl;
//    cloud_camera_ = NULL;

    // this is just for testing
    camera_1_view_point_ = Eigen::Matrix3Xd(3,1);
    camera_1_view_point_ << -0.5347446985255242, 0.7120088432392226, 0.7387054207879453;
    std::vector<double> tote_workspace  = {-0.12345150113105774, 0.27654850482940674, -0.30238577723503113,
                                          0.19761420786380768, 0.005564459599554539, 0.28556445240974426};

    std::cout << "test2" << std::endl;
    workspace_ = tote_workspace;

    // set camera viewpoint to default origin
    std::vector<double> camera_position;
//  node.getParam("camera_position", camera_position);
//  view_point_ << camera_position[0], camera_position[1], camera_position[2];

    // choose sampling method for grasp detection
    use_importance_sampling_ = false;

//    grasp_detector_ = new GraspDetector(node);

    // uses ROS topics to publish grasp candidates, antipodal grasps, and grasps after clustering
//    grasps_pub_ = node.advertise<gpg::GraspConfigList>("clustered_grasps", 10);

//    node.getParam("workspace", workspace_);
    std::string cloud_topic = "/cloud_pcd";
    std::cout << "making subscriber" << std::endl;
    cloud_sub_ = node.subscribe(cloud_topic, 1, &GenerateCandidatesNode::cloud_pcd_callback, this);
    grasps_pub_ = node.advertise<gpg::GraspConfigList>("/candidate_grasps", 10);
}

void GenerateCandidatesNode::cloud_samples_callback(const gpg::CloudSamples& msg){
    return;
}

void GenerateCandidatesNode::cloud_pcd_callback(const sensor_msgs::PointCloud2 msg) {
    ROS_INFO("got a pointcloud on /cloud_pcd");

    cloud_camera_.reset();

    if (msg.fields.size() == 6 && msg.fields[3].name == "normal_x" && msg.fields[4].name == "normal_y"
        && msg.fields[5].name == "normal_z")
    {
        PointCloudPointNormal::Ptr cloud(new PointCloudPointNormal);
        pcl::fromROSMsg(msg, *cloud);
        cloud_camera_ = std::make_shared<CloudCamera>(cloud, 0, camera_1_view_point_);
        ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points and normals.");
    }
    else
    {
        PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
        pcl::fromROSMsg(msg, *cloud);
        cloud_camera_ = std::make_shared<CloudCamera>(cloud, 0, camera_1_view_point_);
        ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points.");
    }

    ROS_INFO("Finished making CloudCamera object");
    this->generate_candidates_from_cloud_camera(*cloud_camera_);

}

std::vector<Grasp> GenerateCandidatesNode::generate_candidates_from_cloud_camera(CloudCamera cloud_cam){
    candidates_generator_ = this->make_default_candidates_generator();
    candidates_generator_->preprocessPointCloud(cloud_cam);
    ROS_INFO("Finished pre-processing point cloud");
    std::vector<Grasp> candidates = candidates_generator_->generateGraspCandidates(cloud_cam);
    ROS_INFO("Finished generating candidates");



    // now we should publish them out as well
    gpg::GraspConfigList msg = this->createGraspListMsg(candidates);
    ROS_INFO("Finished making candidates msg");

    grasps_pub_.publish(msg);
    return candidates;

}

gpg::GraspConfig GenerateCandidatesNode::convertToGraspMsg(const Grasp& hand)
{
    gpg::GraspConfig msg;
    tf::pointEigenToMsg(hand.getGraspBottom(), msg.bottom);
    tf::pointEigenToMsg(hand.getGraspTop(), msg.top);
    tf::pointEigenToMsg(hand.getGraspSurface(), msg.surface);
    tf::vectorEigenToMsg(hand.getApproach(), msg.approach);
    tf::vectorEigenToMsg(hand.getBinormal(), msg.binormal);
    tf::vectorEigenToMsg(hand.getAxis(), msg.axis);
    msg.width.data = hand.getGraspWidth();
    msg.score.data = hand.getScore();
    tf::pointEigenToMsg(hand.getSample(), msg.sample);
    msg.full_antipodal = hand.isFullAntipodal();
    return msg;
}

gpg::GraspConfigList GenerateCandidatesNode::createGraspListMsg(const std::vector<Grasp>& hands)
{
    gpg::GraspConfigList msg;

    for (int i = 0; i < hands.size(); i++)
        msg.grasps.push_back(this->convertToGraspMsg(hands[i]));

    msg.header.stamp = ros::Time::now();

    return msg;
}

std::shared_ptr<CandidatesGenerator> GenerateCandidatesNode::make_default_candidates_generator() {
    // Create object to generate grasp candidates.
    CandidatesGenerator::Parameters generator_params;
    generator_params.num_samples_ = 100;
    generator_params.num_threads_ = 4;
    generator_params.plot_normals_ = false;
    generator_params.plot_grasps_ = false;
    generator_params.remove_statistical_outliers_ = true;
    generator_params.voxelize_ = true;

    std::vector<double> workspace = {-1.0, 1.0, -1.0, 1.0, -1.0, 1.0};
    generator_params.workspace_ = workspace;


    HandSearch::Parameters hand_search_params;
    hand_search_params.finger_width_ = 0.01;
    hand_search_params.hand_outer_diameter_ = 0.12;
    hand_search_params.hand_depth_ = 0.06;
    hand_search_params.hand_height_ = 0.02;
    hand_search_params.init_bite_ = 0.01;
    hand_search_params.nn_radius_frames_ = 0.01;
    hand_search_params.num_orientations_ = 8;
    hand_search_params.num_samples_ = 100;
    hand_search_params.num_threads_ = 4;
    hand_search_params.rotation_axis_ = 2; // not sure exactly what this mean
    return std::make_shared<CandidatesGenerator>(generator_params, hand_search_params);
//    CandidatesGenerator candidates_generator(generator_params, hand_search_params);
//    return candidates_generator;
}

void GenerateCandidatesNode::run() {
    ros::Rate rate(100);
    ROS_INFO("Waiting for point cloud to arrive ...");

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    std::cout << "initializing node " << std::endl;
    // seed the random number generator
    std::srand(std::time(0));

    // initialize ROS
    ros::init(argc, argv, "generate_candidates");
    ros::NodeHandle node("~");


    std::cout << "constructing generate candidates node " << std::endl;
    GenerateCandidatesNode generate_candidates_node(node);
    std::cout << "finished making generate_candidates_node " << std::endl;
    generate_candidates_node.run();

    return 0;
}
