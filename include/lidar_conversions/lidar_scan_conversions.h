/**
 * @file lidar_scan_conversions.h
 * @brief Convert discourse-style LidarScan + LidarInfo to PCL PointCloud2.
 */

#pragma once

#include <lidar_msgs/msg/lidar_info.hpp>
#include <lidar_msgs/msg/lidar_scan.hpp>
#include <pcl/PCLPointCloud2.h>

#include <string>

namespace ouster_ros {

struct LidarScanToPointCloudOptions {
    /// Channel name for range (case-insensitive match against LidarChannel.name).
    std::string range_channel_name = "range";
    /// If true, width/height match the scan grid; invalid pixels are NaN.
    bool organized = true;
};

/**
 * Project structured lidar range data to Cartesian XYZ using per-pixel geometry
 * from LidarInfo (per-beam / per-column angles or uniform FOV interpolation).
 *
 * Angular convention: elevation measured from the horizontal plane; azimuth in
 * the XY plane with zero along +X and positive CCY (same spirit as sensor_msgs/LaserScan).
 *
 * @param info Geometry and range scaling (SI metres after scaling).
 * @param scan Channel layout and raw buffer.
 * @param[out] cloud_out PCL PointCloud2 with x,y,z as float32 (PointField names x,y,z).
 * @param opts Optional range channel name and organized flag.
 * @throws std::invalid_argument on inconsistent dimensions or missing range channel.
 */
void LidarScanToPointCloud(
    const lidar_msgs::msg::LidarInfo& info,
    const lidar_msgs::msg::LidarScan& scan,
    pcl::PCLPointCloud2& cloud_out,
    const LidarScanToPointCloudOptions& opts = {});

}  // namespace ouster_ros
