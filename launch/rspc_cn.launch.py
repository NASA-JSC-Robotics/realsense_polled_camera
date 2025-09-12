import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

output = 'screen'

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
    

rspc_node_class= 'RealSensePolledCameraNode'

rspc_configurable_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'camera_namespace',             'default': 'camera', 'description': 'namespace for camera'},
                           {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},
                           {'name': 'initial_reset',                'default': 'false', 'description': "''"},
                           {'name': 'accelerate_gpu_with_glsl',     'default': "false", 'description': 'enable GPU acceleration with GLSL'},
                           {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'rosbag_loop',                  'default': 'false', 'description': 'Enable loop playback when playing a bagfile'},
                           {'name': 'log_level',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'rgb_camera.color_profile',     'default': '0,0,0', 'description': 'color stream profile'},
                           {'name': 'rgb_camera.color_format',      'default': 'RGB8', 'description': 'color stream format'},
                           {'name': 'rgb_camera.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for color image'},
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'enable_infra',                 'default': 'false', 'description': 'enable infra0 stream'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'depth_module.depth_profile',   'default': '0,0,0', 'description': 'depth stream profile'},
                           {'name': 'depth_module.depth_format',    'default': 'Z16', 'description': 'depth stream format'},
                           {'name': 'depth_module.infra_profile',   'default': '0,0,0', 'description': 'infra streams (0/1/2) profile'},
                           {'name': 'depth_module.infra_format',    'default': 'RGB8', 'description': 'infra0 stream format'},
                           {'name': 'depth_module.infra1_format',   'default': 'Y8', 'description': 'infra1 stream format'},
                           {'name': 'depth_module.infra2_format',   'default': 'Y8', 'description': 'infra2 stream format'},
                           {'name': 'depth_module.color_profile',   'default': '0,0,0', 'description': 'Depth module color stream profile for d405'},
                           {'name': 'depth_module.color_format',    'default': 'RGB8', 'description': 'color stream format for d405'},
                           {'name': 'depth_module.exposure',        'default': '8500', 'description': 'Depth module manual exposure value'},
                           {'name': 'depth_module.gain',            'default': '16', 'description': 'Depth module manual gain value'},
                           {'name': 'depth_module.hdr_enabled',     'default': 'false', 'description': 'Depth module hdr enablement flag. Used for hdr_merge filter'},
                           {'name': 'depth_module.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for depth image'},
                           {'name': 'depth_module.exposure.1',      'default': '7500', 'description': 'Depth module first exposure value. Used for hdr_merge filter'},
                           {'name': 'depth_module.gain.1',          'default': '16', 'description': 'Depth module first gain value. Used for hdr_merge filter'},
                           {'name': 'depth_module.exposure.2',      'default': '1', 'description': 'Depth module second exposure value. Used for hdr_merge filter'},
                           {'name': 'depth_module.gain.2',          'default': '16', 'description': 'Depth module second gain value. Used for hdr_merge filter'},
                           {'name': 'enable_sync',                  'default': 'false', 'description': "'enable sync mode'"},
                           {'name': 'depth_module.inter_cam_sync_mode',               'default': "0", 'description': '[0-Default, 1-Master, 2-Slave]'},
                           {'name': 'enable_rgbd',                  'default': 'false', 'description': "'enable rgbd topic'"},
                           {'name': 'enable_gyro',                  'default': 'false', 'description': "'enable gyro stream'"},
                           {'name': 'enable_accel',                 'default': 'false', 'description': "'enable accel stream'"},
                           {'name': 'gyro_fps',                     'default': '0', 'description': "''"},
                           {'name': 'enable_motion',                'default': 'false', 'description': "'enable motion stream (IMU) for DDS devices'"},
                           {'name': 'accel_fps',                    'default': '0', 'description': "''"},
                           {'name': 'unite_imu_method',             'default': "0", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                           {'name': 'clip_distance',                'default': '-2.', 'description': "''"},
                           {'name': 'angular_velocity_cov',         'default': '0.01', 'description': "''"},
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': "''"},
                           {'name': 'diagnostics_period',           'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
                           {'name': 'publish_tf',                   'default': 'true', 'description': '[bool] enable/disable publishing static & dynamic TF'},
                           {'name': 'tf_publish_rate',              'default': '0.0', 'description': '[double] rate in Hz for publishing dynamic TF'},
                           {'name': 'pointcloud.enable',            'default': 'false', 'description': ''},
                           {'name': 'pointcloud.stream_filter',     'default': '2', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'pointcloud.ordered_pc',        'default': 'false', 'description': ''},
                           {'name': 'pointcloud.allow_no_texture_points', 'default': 'false', 'description': "''"},
                           {'name': 'align_depth.enable',           'default': 'false', 'description': 'enable align depth filter'},
                           {'name': 'colorizer.enable',             'default': 'false', 'description': 'enable colorizer filter'},
                           {'name': 'decimation_filter.enable',     'default': 'false', 'description': 'enable_decimation_filter'},
                           {'name': 'rotation_filter.enable',       'default': 'false', 'description': 'enable rotation_filter'},
                           {'name': 'rotation_filter.rotation',     'default': '0.0',   'description': 'rotation value: 0.0, 90.0, -90.0, 180.0'},
                           {'name': 'spatial_filter.enable',        'default': 'false', 'description': 'enable_spatial_filter'},
                           {'name': 'temporal_filter.enable',       'default': 'false', 'description': 'enable_temporal_filter'},
                           {'name': 'disparity_filter.enable',      'default': 'false', 'description': 'enable_disparity_filter'},
                           {'name': 'hole_filling_filter.enable',   'default': 'false', 'description': 'enable_hole_filling_filter'},
                           {'name': 'hdr_merge.enable',             'default': 'false', 'description': 'hdr_merge filter enablement flag'},
                           {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
                           {'name': 'reconnect_timeout',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
                           {'name': 'base_frame_id',                'default': 'link', 'description': 'Root frame of the sensors transform tree'},
                          ]


camera_params = declare_configurable_parameters(rspc_configurable_parameters)
launch_args = [
    DeclareLaunchArgument(name="namespace", default_value="", description="namespace"),
    DeclareLaunchArgument(name="camera_name", default_value="D435", description="camera name"),
    DeclareLaunchArgument(name="use_config_file", default_value='', description="use a config file for these nodes"),
    DeclareLaunchArgument(
        name="camera_config_file",
        default_value="",
        description="realsense camera configuration parameters",
    ),
    DeclareLaunchArgument(
        name="camera_config_path",
        default_value="",
        description="path to realsense camera config. If not given, determined based on provided 'camera_config' above",
    ),
]

def launch_setup(context):

    use_config_file = LaunchConfiguration("use_config_file").perform(context)
    camera_name = LaunchConfiguration("camera_name").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    params_from_file = {}

    print("****************************** ", use_config_file)

    if use_config_file:
        camera_config_path = LaunchConfiguration("camera_config_path").perform(context)
        camera_config_file = LaunchConfiguration("camera_config_file").perform(context)


        configs_dir = os.path.join(get_package_share_directory("realsense_polled_camera"), "config")
        

        if not camera_config_file:
            camera_config_file = camera_name + ".yaml"

        if not camera_config_path:
            available_configs = os.listdir(configs_dir)
            if camera_config_file in available_configs:
                camera_config_path = os.path.join(configs_dir, camera_config_file)
            else:
                return [
                    LogInfo(
                        msg="ERROR: unknown config: '{}' - Available configs are: {} - not starting Realsense Polled Camera".format(
                            camera_config_file, ", ".join(available_configs)
                        )
                    )
                ]
        else:
            if not os.path.isfile(camera_config_path):
                return [
                    LogInfo(
                        msg="ERROR: config_path file: '{}' - does not exist. - not starting Realsense Camera".format(
                            camera_config_path)
                        )
                ]

        print(" ******************* Camera config path : ", camera_config_path)
        print(" ******************* Camera config file : ", camera_config_file)

        params_from_file = {} if camera_config_path == "''" else yaml_to_dict(camera_config_path)


    container = ComposableNodeContainer(
        name='perception_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='realsense_polled_camera',
                namespace=namespace,
                plugin=rspc_node_class,
                name=camera_name,
                parameters=[params_from_file, 
                            {'camera_name': camera_name},
                            {'serial_no': '207522074907'},
                            {'enable_depth': False},
                            {'publish_tf': True}],
#                extra_arguments=[{'use_intra_process_comms':True}]),  -- not yet supported
        ],
        output=output
    )
    return[container]

def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args + camera_params)
    ld.add_action(opfunc)
    return ld