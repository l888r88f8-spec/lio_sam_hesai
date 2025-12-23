# LIO_SAM_HESAI (ROS 2)

Adapted LIO-SAM for Livox MID-360. This fork renames the package to `lio_sam_mid360` (ROS package names must be lowercase) and updates configs/launch for MID-360 usage.

## Features
- Livox MID-360 point cloud and timing assumptions
- Clean ROS 2 launch (`run.launch.py`)
- Same node graph as LIO-SAM (image projection, feature extraction, IMU preintegration, map optimization)
- Directly supports ROS Bag Integration
- Supports both 6-axis and 9-axis IMUs

## Requirements
- Ubuntu 22.04 + ROS 2 Humble (other ROS 2 distros may work)
- GTSAM 4.x (`libgtsam-dev` and `libgtsam-unstable-dev`)
- PCL, OpenCV (installed via ROS packages below)

Install ROS packages (replace `<ros2>` with your distro, e.g., `humble`):
```
sudo apt update
sudo apt install \
  ros-<ros2>-perception-pcl \
  ros-<ros2>-pcl-msgs \
  ros-<ros2>-vision-opencv \
  libgtsam-dev libgtsam-unstable-dev
```

## Build
```
cd ~/ros2_ws/src
# this folder should be at ~/ros2_ws/src/LIO_SAM_MID360
cd ..
colcon build --symlink-install
source install/setup.bash
```

## Configure for Livox MID-360
Edit `config/params.yaml`:
- Set sensor to Livox settings (rings, horizon) appropriate for MID-360
- Set topics to match your Livox driver output
- Set IMU extrinsics so IMU -> lidar follows REP-105 (x forward, y left, z up)

Default static TFs in `launch/run.launch.py` assume co-located IMU and lidar; adjust if your mounting differs.

Edit the fov of the Lidar setting. For Livox MID360 (Default) it is -7 to 52 degree.

## Run
```
ros2 launch lio_sam_mid360 run.launch.py
```

Play a bag in another terminal:
```
ros2 bag play your_data
```

## Save map service
```
ros2 service call /lio_sam/save_map lio_sam_mid360/srv/SaveMap "{resolution: 0.2, destination: /tmp/LOAM}"
```
<img width="1900" height="938" alt="image" src="https://github.com/user-attachments/assets/71c4b066-bc69-43f3-b2f0-d133372c79ad" />
<img width="1885" height="931" alt="image_1" src="https://github.com/user-attachments/assets/e914fd49-680f-42c4-94af-ef969f65a800" />


Note: The service name `/lio_sam/save_map` follows the original namespace; the type is from this package.

## Notes
- Ensure Livox point type provides per-point time (relative within scan) and ring/channel index. Adapt `imageProjection.cpp` if your fields differ.
- DDS/QoS: tune if running over networks with Livox drivers.

## Issues
- There was a large bias when using Livox IMU. This lead to an inaccurate incremental odometry estimate.
  
## Credits
- Adaptation and maintenance: Vishnuraj A
- Based on the original LIO-SAM by Tixiao Shan et al. See `LICENSE` and original repository for citations.
