/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include "realsense_polled_camera/RealSensePolledCameraNode.hpp"

RealSensePolledCameraNode::RealSensePolledCameraNode(const rclcpp::NodeOptions& optionsA)
  : Node("realsense_polled_camera", optionsA)
  , m_nodeHandlePtr(std::shared_ptr<RealSensePolledCameraNode>(this, [](auto*) {}))
  , m_imageTransport(m_nodeHandlePtr)
  , m_enableColorImage(true)
  , m_enablePointCloud(true)
  , m_alignDepthToColor(true)
  , m_colorFramename("depth_optical_frame")
  , m_depthFramename("depth_optical_frame")
  , m_infra1Framename("infra1_optical_frame")
  , m_infra2Framename("infra2_optical_frame")
{
  initialize();
  initializeCamera();
}

RealSensePolledCameraNode::~RealSensePolledCameraNode()
{
}

void RealSensePolledCameraNode::initialize()
{
  RCLCPP_DEBUG(this->get_logger(), "%s : %d", __FUNCTION__, __LINE__);
  declare_parameter("camera_name", "camera");
  declare_parameter("enable_color", true);
  declare_parameter("enable_depth", true);
  declare_parameter("pointcloud.enable", false);
  declare_parameter("align_depth.enable", false);
  declare_parameter("serial_no", "");
  declare_parameter("rgb_camera.color_profile", "");
  declare_parameter("rgb_camera.color_format", "BGR8");
  declare_parameter("publish_tf", true);

  m_cameraName = get_parameter("camera_name").as_string();
  m_serialNumber = get_parameter("serial_no").as_string();
  m_enableColorImage = get_parameter("enable_color").as_bool();
  m_enableDepthImage = get_parameter("enable_depth").as_bool();
  m_enablePointCloud = get_parameter("pointcloud.enable").as_bool();
  m_alignDepthToColor = get_parameter("align_depth.enable").as_bool();
  m_colorFormat = get_parameter("rgb_camera.color_format").as_string();

  RCLCPP_INFO(this->get_logger(), "PARAM: camera_name               %s",
              get_parameter("camera_name").as_string().c_str());
  RCLCPP_INFO(this->get_logger(), "PARAM: serial_no                 %s", get_parameter("serial_no").as_string().c_str());
  RCLCPP_INFO(this->get_logger(), "PARAM: enable_color              %s",
              get_parameter("enable_color").as_bool() ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "PARAM: enable_depth              %s",
              get_parameter("enable_depth").as_bool() ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "PARAM: pointcloud.enable         %s",
              get_parameter("pointcloud.enable").as_bool() ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "PARAM: align_depth.enable        %s",
              get_parameter("align_depth.enable").as_bool() ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "PARAM: publish_tf                %s",
              get_parameter("publish_tf").as_bool() ? "true" : "false");

  RCLCPP_INFO(this->get_logger(), "PARAM: rgb_camera.color_profile   %s",
              get_parameter("rgb_camera.color_profile").as_string().c_str());
  RCLCPP_INFO(this->get_logger(), "PARAM: rgb_camera.color_format    %s",
              get_parameter("rgb_camera.color_format").as_string().c_str());

  m_colorFramename = m_cameraName + "_color_optical_frame";
  m_depthFramename = m_cameraName + "_depth_optical_frame";
  m_infra1Framename = m_cameraName + "infra1_optical_frame";
  m_infra2Framename = m_cameraName + "infra2_optical_frame";

  if (m_enableColorImage)
  {
    m_colorImagePublisher = image_transport::create_publisher(this, "~/color/image_raw");
    m_colorImageInfoPubPtr = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/color/camera_info", 1);
  }
  if (m_enableDepthImage)
  {
    m_depthImagePublisher = image_transport::create_publisher(this, "~/depth/image_raw");
    m_depthImageInfoPubPtr = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/depth/camera_info", 1);
  }

  if (m_enablePointCloud)
  {
    m_pointCloudPubPtr = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/pointcloud", 1);
  }

  // if(get_parameter("publish_tf").as_bool())
  // {
  //     m_staticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  // }
  m_triggerServer = this->create_service<std_srvs::srv::Trigger>(
      "~/request_images",
      std::bind(&RealSensePolledCameraNode::pollService, this, std::placeholders::_1, std::placeholders::_2));
}

void RealSensePolledCameraNode::initializeCamera()
{
  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();
  std::string selected_serial;
  if (m_serialNumber.empty())
  {
    m_device = devices.front();
    selected_serial = m_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  }
  else
  {
    for (rs2::device dev : devices)
    {
      std::string sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      printf("Serial number = %s\n", sn.c_str());
      fflush(stdout);
      if (sn == m_serialNumber)
      {
        m_device = dev;
        selected_serial = sn;
        break;
      }
    }
  }
  if (!m_device)
  {
    RCLCPP_FATAL(this->get_logger(), "Camera with serial number %s not found", m_serialNumber.c_str());
  }
  m_rsConfig.enable_device(selected_serial);

  if (get_parameter("enable_color").as_bool())
  {
    std::string colorStreamFormat = get_parameter("rgb_camera.color_profile").as_string();
    bool gotProfile = false;
    if (!colorStreamFormat.empty())
    {
      int32_t w = 0;
      int32_t h = 0;
      int32_t fps = 0;
      gotProfile = getParamsFromStreamProfile(colorStreamFormat, w, h, fps);
      if (gotProfile)
      {
        if (m_colorFormat == "BGR8")
        {
          m_rsConfig.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, fps);
        }
        else
        {
          m_rsConfig.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_RGB8, fps);
        }
      }
      else
      {
        RCLCPP_FATAL(this->get_logger(), "Invalid color stream format %s",
                     get_parameter("rgb_camera.color_profile").as_string().c_str());
      }
    }
    if (!gotProfile)
    {
      m_rsConfig.enable_stream(RS2_STREAM_COLOR);
    }
  }
  if (get_parameter("enable_depth").as_bool())
  {
    m_rsConfig.enable_stream(RS2_STREAM_DEPTH);
  }
  m_profile = m_pipe.start(m_rsConfig);

  if (get_parameter("align_depth.enable").as_bool())
  {
    m_alignPtr = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
  }

  // if(get_parameter("publish_tf").as_bool())
  // {
  //     publishTfFrames();
  // }
}

void RealSensePolledCameraNode::pollService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "%s : %d", __FUNCTION__, __LINE__);
  cv::Mat rgbImage;
  cv::Mat depthImage;
  bool haveFrames = true;
  response->success = false;
  response->message = "";
  rs2::frameset frames;
  std::shared_ptr<rs2::depth_frame> depthFramePtr;
  std::shared_ptr<rs2::video_frame> colorFramePtr;
  try
  {
    frames = m_pipe.wait_for_frames();
  }
  catch (const std::exception& e)
  {
    response->message = e.what();
    haveFrames = false;
  }
  if (haveFrames)
  {
    builtin_interfaces::msg::Time timestamp = this->now();

    if (m_alignDepthToColor && m_alignPtr)
    {
      frames = m_alignPtr->process(frames);
    }
    if (m_enableDepthImage)
    {
      depthFramePtr = std::make_shared<rs2::depth_frame>(frames.get_depth_frame());
    }
    if (m_enableColorImage)
    {
      colorFramePtr = std::make_shared<rs2::video_frame>(frames.get_color_frame());
    }

    if (colorFramePtr)
    {
      processColorFrame(*colorFramePtr, timestamp);
      response->success = true;
      response->message += " color image retrieved,";
    }
    if (depthFramePtr)
    {
      processDepthFrame(*depthFramePtr, m_enablePointCloud, timestamp);
      response->success = true;
      response->message += " depth image retrieved,";
    }
    /*
            // If one of them is unavailable, continue iteration
            if(aligned_depth_frame && color_frame)
            {

                const int w = aligned_depth_frame.as<rs2::video_frame>().get_width();
                const int h = aligned_depth_frame.as<rs2::video_frame>().get_height();
                cv::Mat depthImage(cv::Size(w, h), CV_16UC1, (void*)aligned_depth_frame.get_data());
                cv::minMaxIdx(depthImage, &min, &max);
                depthImage.convertTo(normDepth, CV_8UC1, 255.0 / (max - min), -min * 255.0 / (max - min));
                cv::applyColorMap(normDepth, depthImageA, cv::COLORMAP_JET);    // Or other colormaps
                response->success = true;
                response->message = "images retrieved";
            }
    */
  }
}

void RealSensePolledCameraNode::processColorFrame(rs2::video_frame frameA, builtin_interfaces::msg::Time timestampA)
{
  RCLCPP_DEBUG(this->get_logger(), "%s : %d", __FUNCTION__, __LINE__);
  const int w = frameA.as<rs2::video_frame>().get_width();
  const int h = frameA.as<rs2::video_frame>().get_height();
  cv::Mat colorImage = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)frameA.get_data());

  if (m_colorFormat == "RGB8")
  {
    cv::cvtColor(colorImage, colorImage, cv::COLOR_RGB2BGR);
  }

  std_msgs::msg::Header header;
  sensor_msgs::msg::CameraInfo infoMsg;
  header.stamp = timestampA;
  header.frame_id = m_colorFramename;

  getIntrinsics(infoMsg, RS2_STREAM_COLOR);
  sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", colorImage).toImageMsg();

  infoMsg.header = header;

  m_colorImagePublisher.publish(msg);
  m_colorImageInfoPubPtr->publish(infoMsg);
}

void RealSensePolledCameraNode::processDepthFrame(rs2::depth_frame frameA, bool /*publishPointCloudA*/,
                                                  builtin_interfaces::msg::Time timestampA)
{
  RCLCPP_DEBUG(this->get_logger(), "%s : %d", __FUNCTION__, __LINE__);
  const int w = frameA.as<rs2::video_frame>().get_width();
  const int h = frameA.as<rs2::video_frame>().get_height();
  cv::Mat image = cv::Mat(cv::Size(w, h), CV_16UC1, (void*)frameA.get_data());

  std_msgs::msg::Header header;
  sensor_msgs::msg::CameraInfo infoMsg;
  header.stamp = timestampA;
  if (m_alignDepthToColor)
  {
    header.frame_id = m_colorFramename;
  }
  else
  {
    header.frame_id = m_depthFramename;
  }

  getIntrinsics(infoMsg, RS2_STREAM_DEPTH);
  sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "mono16", image).toImageMsg();

  m_depthImagePublisher.publish(msg);
  m_depthImageInfoPubPtr->publish(infoMsg);
  RCLCPP_INFO(this->get_logger(), "%s : %d", __FUNCTION__, __LINE__);
}

void RealSensePolledCameraNode::getIntrinsics(sensor_msgs::msg::CameraInfo& infoA, rs2_stream streamIdA)
{
  RCLCPP_DEBUG(this->get_logger(), "%s : %d", __FUNCTION__, __LINE__);
  rs2_intrinsics camInfo =
      m_pipe.get_active_profile().get_stream(streamIdA).as<rs2::video_stream_profile>().get_intrinsics();
  infoA.width = camInfo.width;
  infoA.height = camInfo.height;
  memset(&infoA.k, 0, sizeof(infoA.k));
  infoA.k[0] = camInfo.fx;
  infoA.k[2] = camInfo.ppx;
  infoA.k[4] = camInfo.fy;
  infoA.k[5] = camInfo.ppy;
  infoA.k[8] = 1.0;
  infoA.p[0] = camInfo.fx;
  infoA.p[2] = camInfo.ppx;
  infoA.p[5] = camInfo.fy;
  infoA.p[6] = camInfo.ppy;
  infoA.p[10] = 1.0;
  infoA.d.resize(5);
  for (int32_t idx = 0; idx < 5; ++idx)
  {
    infoA.d[idx] = camInfo.coeffs[idx];
  }
  switch (camInfo.model)
  {
    case RS2_DISTORTION_NONE:
      infoA.distortion_model = "none";
      break;
    case RS2_DISTORTION_MODIFIED_BROWN_CONRADY:
      infoA.distortion_model = "modified_brown_conrady";
      break;
    case RS2_DISTORTION_INVERSE_BROWN_CONRADY:
      infoA.distortion_model = "inverse_brown_conrady";
      break;
    case RS2_DISTORTION_FTHETA:
      infoA.distortion_model = "f-theta_fish-eye";
      break;
    case RS2_DISTORTION_BROWN_CONRADY:
      infoA.distortion_model = "brown_conrady";
      break;
    case RS2_DISTORTION_KANNALA_BRANDT4:
      infoA.distortion_model = "kannala_brandt4";
      break;
    case RS2_DISTORTION_COUNT:
    default:
      infoA.distortion_model = "unknown";
  }
}

bool RealSensePolledCameraNode::getParamsFromStreamProfile(std::string profileStrA, int32_t& widthA, int32_t& heightA,
                                                           int32_t& fpsA)
{
  RCLCPP_DEBUG(this->get_logger(), "%s : %d", __FUNCTION__, __LINE__);
  bool success = true;
  char delimiter = ',';
  std::vector<std::string> tokens;

  std::istringstream pstr(profileStrA);  // Create an input string stream from the string
  std::string token;

  // Loop to extract tokens using std::getline
  while (std::getline(pstr, token, delimiter))
  {
    tokens.push_back(token);
  }
  if ((tokens.size() == 3) && (std::stoi(tokens[0]) > 0) && (std::stoi(tokens[1]) > 0) && (std::stoi(tokens[2]) > 0))
  {
    widthA = std::stoi(tokens[0]);
    heightA = std::stoi(tokens[1]);
    fpsA = std::stoi(tokens[2]);
  }
  else
  {
    success = false;
  }
  return success;
}

// void RealSensePolledCameraNode::publishTfFrames()
// {
//     std::vector<geometry_msgs::msg::TransformStamped> transforms;
//     // color_frame to color_optical_frame
//     geometry_msgs::msg::TransformStamped t;

//     // Set header information
//     t.header.stamp = this->get_clock()->now();
//     t.header.frame_id = m_cameraName + "_link"; // Parent frame
//     t.child_frame_id = m_cameraName + "_color_frame"; // Child frame

//     // Set translation//0 ${d435_cam_depth_to_color_offset} 0
//     t.transform.translation.x = 0.0;
//     t.transform.translation.y = 0.015;
//     t.transform.translation.z = 0.0;

//     t.transform.rotation.x = 0.0;
//     t.transform.rotation.y = 0.0;
//     t.transform.rotation.z = 0.0;
//     t.transform.rotation.w = 1.0;
//     transforms.push_back(t);

//     // Set header information
//     t.header.stamp = this->get_clock()->now();
//     t.header.frame_id = m_cameraName + "_color_frame"; // Parent frame
//     t.child_frame_id = m_cameraName + "_color_optical_frame"; // Child frame

//     // Set translation (example: 1m in x, 0m in y, 0.5m in z)
//     t.transform.translation.x = 0.0;
//     t.transform.translation.y = 0.0;
//     t.transform.translation.z = 0.0;

//     t.transform.rotation.x = -0.5;
//     t.transform.rotation.y =  0.5;
//     t.transform.rotation.z = -0.5;
//     t.transform.rotation.w =  0.5;
//     transforms.push_back(t);

//     // Set header information
//     t.header.stamp = this->get_clock()->now();
//     t.header.frame_id = m_cameraName + "_link"; // Parent frame
//     t.child_frame_id = m_cameraName + "_depth_frame"; // Child frame

//     // Set translation (example: 1m in x, 0m in y, 0.5m in z)
//     t.transform.translation.x = 0.0;
//     t.transform.translation.y = 0.0;
//     t.transform.translation.z = 0.0;

//     t.transform.rotation.x = 0.0;
//     t.transform.rotation.y = 0.0;
//     t.transform.rotation.z = 0.0;
//     t.transform.rotation.w = 1.0;
//     transforms.push_back(t);

//     // Set header information
//     t.header.stamp = this->get_clock()->now();
//     t.header.frame_id = m_cameraName + "_depth_frame"; // Parent frame
//     t.child_frame_id = m_cameraName + "_depth_optical_frame"; // Child frame

//     // Set translation (example: 1m in x, 0m in y, 0.5m in z)
//     t.transform.translation.x = 0.0;
//     t.transform.translation.y = 0.0;
//     t.transform.translation.z = 0.0;

//     t.transform.rotation.x = -0.5;
//     t.transform.rotation.y =  0.5;
//     t.transform.rotation.z = -0.5;
//     t.transform.rotation.w =  0.5;
//     transforms.push_back(t);

//     m_staticTfBroadcaster->sendTransform(transforms);
// }
