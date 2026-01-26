# RealSense Polled Camera

ROS 2 wrapper of [Realsense Library](https://github.com/IntelRealSense/librealsense) that provides on-demand access to RGB and Depth images.
This type of behavior is more common for space applications than constant streaming from sensors, which can consume large amounts of CPU and network resources.

Images are published to familiar topics, namely

```bash
# RGB images/info
~/color/image_raw
~/color/camera_info

# Depth images/info
~/depth/image_raw
~/depth/camera_info
~/pointcloud
```

Consumers of this node can _reqest_ that images are polled and published by calling the `~/request_images` trigger service.

Parameters, launch configuration, etc, are all similar to the upstream node, and can be used as such.

For example,

```python
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


def generate_launch_description():

    wrist_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("realsense_polled_camera"), "launch", "rspc.launch.py")
        ),
        launch_arguments={
            "camera_name": "wrist_mounted_camera",
            "camera_namespace": "",
            "serial_no": "SERIAL_NO",
            "rgb_camera.color_profile": "1280,720,6",
            "depth_module.depth_profile": "320,180,6",
            "depth_module.infra_profile": "320,180,6",
            "initial_reset": "true",
            "pointcloud.enable": "true",
            "align_depth.enable": "true",
            "enable_depth": "true",
            "publish_tf": "true",
        }.items(),
    )

    return LaunchDescription([wrist_camera])
```
## Citation

This project falls under the purview of the iMETRO project.
If you use this in your own work, please cite the following paper:

```bibtex
@INPROCEEDINGS{imetro-facility-2025,
  author={Dunkelberger, Nathan and Sheetz, Emily and Rainen, Connor and Graf, Jodi and Hart, Nikki and Zemler, Emma and Azimi, Shaun},
  booktitle={2025 22nd International Conference on Ubiquitous Robots (UR)},
  title={Design of the iMETRO Facility: A Platform for Intravehicular Space Robotics Research},
  year={2025},
  volume={},
  number={},
  pages={390-397},
  keywords={NASA;Moon;Seals;Maintenance engineering;Maintenance;Robots;Standards;Open source software;Testing;Logistics},
  doi={10.1109/UR65550.2025.11077983}}
```
