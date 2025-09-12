#pragma once

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>


#include <librealsense2/rs.hpp>


class RealSensePolledCameraNode : public rclcpp::Node
{
  public:
    RealSensePolledCameraNode(const rclcpp::NodeOptions& optionsA);
    ~RealSensePolledCameraNode();

  private:
    rclcpp::Node::SharedPtr m_nodeHandlePtr;
    image_transport::ImageTransport m_imageTransport;

    bool m_enableColorImage;
    bool m_enableDepthImage;
    bool m_enablePointCloud;
    bool m_alignDepthToColor;
    std::string m_colorFormat;
    std::string m_serialNumber;
    rs2::device m_device;
    rs2::pipeline m_pipe;
    rs2::config m_rsConfig;
    rs2::context m_ctx;

    std::string m_cameraName;
    std::string m_colorFramename;
    std::string m_depthFramename;
    std::string m_infra1Framename;
    std::string m_infra2Framename;

    rs2::pipeline_profile m_profile;
    std::shared_ptr<rs2::align> m_alignPtr;

    image_transport::Publisher m_colorImagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_colorImageInfoPubPtr;
    image_transport::Publisher m_depthImagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_depthImageInfoPubPtr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointCloudPubPtr;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_triggerServer;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_staticTfBroadcaster;
    

    void initialize();
    void initializeCamera();

    void pollService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void processColorFrame(rs2::video_frame colorFrameA, builtin_interfaces::msg::Time timestampA);
    void processDepthFrame(rs2::depth_frame depthFrameA, bool publishPointCloudA, builtin_interfaces::msg::Time timestampA);
    void getIntrinsics(sensor_msgs::msg::CameraInfo& infoA, rs2_stream streamIdA);

    // camera setup functions
    void getDevice(rs2::device_list list);

    void publishTfFrames();

    // utilities
    bool getParamsFromStreamProfile(std::string profileStrA, int32_t& widthA, int32_t& heightA, int32_t& fpsA);
};

RCLCPP_COMPONENTS_REGISTER_NODE(RealSensePolledCameraNode)