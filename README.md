# lidar-conversions

ROS 2 library that turns **`lidar_msgs/LidarInfo` + `lidar_msgs/LidarScan`** into a **dense XYZ point cloud** as `pcl::PCLPointCloud2`. It is meant for drivers or nodes that already publish the structured lidar messages and need Cartesian points for visualization, mapping, or downstream algorithms.

## What it provides

| Artifact | Description |
|----------|-------------|
| **`lidar_conversions/lidar_scan_conversions.h`** | Public API: `lidar_conversions::LidarScanToPointCloud(...)`, `LidarScanToPointCloudOptions` (range channel name, organized vs. flat cloud). |
| **`lidar_conversions_library`** | Shared C++ library implementing spherical projection from the scan’s **range** channel and geometry in `LidarInfo` (explicit per-row / per-column angles, optional per-beam azimuth offsets, or uniform FOV interpolation). Supports **planar** `LidarScan.data` layout (each channel is one contiguous block; see `lidar_msgs`). |
| **`lidar_conversions_py`** | Optional **pybind11** extension installed into this package’s `site-packages`. Exposes `lidar_scan_to_pointcloud2_data(info, scan, ...)` so **rclpy** nodes can call the same C++ routine and wrap the returned buffer in `sensor_msgs/PointCloud2`. |

Downstream C++ packages should `find_package(lidar-conversions)`, link `lidar_conversions_library`, and include the header above. Python code should `source` the workspace and `import lidar_conversions_py` after `lidar-conversions` is installed.

## Dependencies (high level)

`lidar_msgs`, `sensor_msgs`, `pcl_conversions`, plus **pybind11** / **Python** dev headers for the Python module. See `package.xml` for the exact manifest entries.
