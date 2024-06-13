#include "stereo-slam-node.hpp"

#include<opencv2/core/core.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const std::string &camera_name)
:   Node("ORB_SLAM3_ROS2_" + camera_name),
    m_SLAM(pSLAM),
    camera_name_(camera_name)
{
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(camera_name + "/pose", 10);

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), camera_name + "/infra1/image_rect_raw");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), camera_name + "/infra2/image_rect_raw");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
}

StereoSlamNode::~StereoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
     Sophus::SE3f currentPose =  m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));
    
            // Publish the current pose
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = "camera_frame";

            // Set the pose data
            Eigen::Vector3f translation = currentPose.translation();
            Eigen::Quaternionf rotation(currentPose.unit_quaternion());

            pose_msg.pose.position.x = translation.x();
            pose_msg.pose.position.y = translation.y();
            pose_msg.pose.position.z = translation.z();
            pose_msg.pose.orientation.x = rotation.x();
            pose_msg.pose.orientation.y = rotation.y();
            pose_msg.pose.orientation.z = rotation.z();
            pose_msg.pose.orientation.w = rotation.w();

            pose_publisher_->publish(pose_msg);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
    
}
