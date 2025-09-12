cmake_minimum_required(VERSION 3.21)
project(realsense_polled_camera VERSION 0.1.0 LANGUAGES CXX)

# this makes pcl play nice
add_definitions(-DEIGEN_MAX_ALIGN_BYTES=32)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
# find_package(tf2_ros REQUIRED)


########################## OpenCV ###################################

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

########################## libRealSense ##############################

find_package(realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIRS})
####################################################################
find_package(Boost REQUIRED COMPONENTS date_time)

include_directories(include)
  
add_library(realsense_polled_camera_component SHARED src/RealSensePolledCameraNode.cpp)
ament_target_dependencies(realsense_polled_camera_component 
                                      rclcpp
                                      rclcpp_components
                                      std_msgs
                                      std_srvs
                                      sensor_msgs
                                      image_transport
                                      cv_bridge
                                      image_geometry
                                      pcl_conversions
                                      pcl_ros
                                      # tf2_ros
)
target_link_libraries(realsense_polled_camera_component ${OPENCV_LIBS} ${realsense2_LIBRARY} Boost::date_time)

rclcpp_components_register_node(realsense_polled_camera_component
  PLUGIN "RealSensePolledCameraNode"
  EXECUTABLE "realsense_polled_camera"
)

ament_export_targets(export_realsense_polled_camera_component)


install(TARGETS realsense_polled_camera_component
        EXPORT export_realsense_polled_camera_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY
              launch
              DESTINATION share/${PROJECT_NAME}/)

ament_environment_hooks(${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH})              

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
  
  # Create VERSION File
add_custom_command(OUTPUT "VERSION" WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} COMMAND ${CMAKE_COMMAND} -DPROJECT_NAME=${PROJECT_NAME} -P version.cmake )
add_custom_target(WRITE_VERSION ALL DEPENDS "VERSION" )
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" DESTINATION share/${PROJECT_NAME}/)
