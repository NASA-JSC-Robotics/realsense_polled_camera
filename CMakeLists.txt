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
find_package(MPI REQUIRED)
if(NOT TARGET MPI::MPI_C)
  add_library(MPI::MPI_C INTERFACE IMPORTED)
  set_target_properties(MPI::MPI_C PROPERTIES
    INTERFACE_COMPILE_OPTIONS "${MPI_C_COMPILE_FLAGS}"
    INTERFACE_INCLUDE_DIRECTORIES "${MPI_C_INCLUDE_PATH}"
    INTERFACE_LINK_LIBRARIES "${MPI_C_LIBRARIES}"
  )
endif()
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
# find_package(tf2_ros REQUIRED)


########################## OpenCV ###################################

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Version 3.2.1 is the last version of the cv_bridge humble drivers, so check
# if that is the case and set a variable to indicate it to the compiler macros.
if(cv_bridge_VERSION VERSION_LESS_EQUAL "3.3")
  message(STATUS "Using Humble cv_bridge headers")
  add_definitions(-DCV_BRIDGE_HUMBLE)
endif()

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
endif()

ament_package()
