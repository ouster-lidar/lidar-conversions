#include <gtest/gtest.h>
#include <lidar_conversions/lidar_scan_conversions.h>
#include <sensor_msgs/msg/point_field.hpp>

#include <cmath>
#include <cstring>

using lidar_msgs::msg::LidarChannel;
using lidar_msgs::msg::LidarInfo;
using lidar_msgs::msg::LidarScan;
using sensor_msgs::msg::PointField;

// Planar (non-interleaved) layout: each channel owns a contiguous block of
// height * width * count * sizeof(datatype) bytes starting at LidarChannel.offset.

TEST(LidarScanConversion, OnePixelAlongX) {
    LidarInfo info;
    info.range_multiplier = 1.0;
    info.range_offset = 0.0;
    info.vertical_fov_min = 0.F;
    info.vertical_fov_max = 0.F;
    info.horizontal_fov_min = 0.F;
    info.horizontal_fov_max = 0.F;

    LidarScan scan;
    scan.height = 1;
    scan.width = 1;
    scan.is_bigendian = 0;

    LidarChannel ch;
    ch.name = "range";
    ch.offset = 0;
    ch.datatype = PointField::FLOAT32;
    ch.count = 1;
    scan.channels = {ch};

    const float r = 2.5F;
    scan.data.resize(sizeof(float));
    std::memcpy(scan.data.data(), &r, sizeof(float));

    pcl::PCLPointCloud2 cloud;
    lidar_conversions::LidarScanToPointCloud(info, scan, cloud);

    ASSERT_EQ(cloud.width, 1u);
    ASSERT_EQ(cloud.height, 1u);
    ASSERT_EQ(cloud.point_step, 12u);
    float x, y, z;
    std::memcpy(&x, cloud.data.data() + 0, 4);
    std::memcpy(&y, cloud.data.data() + 4, 4);
    std::memcpy(&z, cloud.data.data() + 8, 4);
    EXPECT_NEAR(x, 2.5F, 1e-5);
    EXPECT_NEAR(y, 0.F, 1e-5);
    EXPECT_NEAR(z, 0.F, 1e-5);
    EXPECT_TRUE(cloud.is_dense);
}

TEST(LidarScanConversion, Uint16MillimetersToMeters) {
    LidarInfo info;
    info.range_multiplier = 0.001;
    info.range_offset = 0.0;
    info.vertical_angles = {0.F};
    info.horizontal_angles = {0.F};

    LidarScan scan;
    scan.height = 1;
    scan.width = 1;
    scan.is_bigendian = 0;

    LidarChannel ch;
    ch.name = "range";
    ch.offset = 0;
    ch.datatype = PointField::UINT16;
    ch.count = 1;
    scan.channels = {ch};

    const uint16_t mm = 1000;  // 1 m
    scan.data.resize(sizeof(uint16_t));
    std::memcpy(scan.data.data(), &mm, sizeof(uint16_t));

    pcl::PCLPointCloud2 cloud;
    lidar_conversions::LidarScanToPointCloud(info, scan, cloud);
    float x;
    std::memcpy(&x, cloud.data.data(), 4);
    EXPECT_NEAR(x, 1.F, 1e-4);
}

TEST(LidarScanConversion, UnorganizedWidth) {
    LidarInfo info;
    info.range_multiplier = 1.0;
    info.range_offset = 0.0;
    info.vertical_fov_min = 0.F;
    info.vertical_fov_max = 0.F;
    info.horizontal_fov_min = 0.F;
    info.horizontal_fov_max = 0.F;

    LidarScan scan;
    scan.height = 2;
    scan.width = 2;
    scan.is_bigendian = 0;

    LidarChannel ch;
    ch.name = "range";
    ch.offset = 0;
    ch.datatype = PointField::FLOAT32;
    ch.count = 1;
    scan.channels = {ch};

    // Planar block: 2 * 2 * 1 * sizeof(float32) = 16 bytes; all pixels = 1.0 m.
    scan.data.resize(4 * sizeof(float), 0);
    const float one = 1.F;
    for (int i = 0; i < 4; ++i) {
        std::memcpy(scan.data.data() + i * sizeof(float), &one, sizeof(float));
    }

    pcl::PCLPointCloud2 cloud;
    lidar_conversions::LidarScanToPointCloudOptions opts;
    opts.organized = false;
    lidar_conversions::LidarScanToPointCloud(info, scan, cloud, opts);
    EXPECT_EQ(cloud.height, 1u);
    EXPECT_EQ(cloud.width, 4u);
}

// Two channels (range + reflectivity) packed back-to-back as planar blocks:
// tests that LidarScanToPointCloud indexes its range channel via LidarChannel.offset
// rather than assuming range starts at data[0].
TEST(LidarScanConversion, PlanarTwoChannelsSecondIsRange) {
    LidarInfo info;
    info.range_multiplier = 1.0;
    info.range_offset = 0.0;
    info.vertical_angles = {0.F};
    info.horizontal_angles = {0.F};

    LidarScan scan;
    scan.height = 1;
    scan.width = 1;
    scan.is_bigendian = 0;

    LidarChannel refl;
    refl.name = "reflectivity";
    refl.offset = 0;
    refl.datatype = PointField::UINT8;
    refl.count = 1;

    LidarChannel range;
    range.name = "range";
    range.offset = 1;  // reflectivity occupies byte 0; range starts at byte 1.
    range.datatype = PointField::FLOAT32;
    range.count = 1;

    scan.channels = {refl, range};

    scan.data.resize(1 + sizeof(float), 0);
    scan.data[0] = 42;  // reflectivity value, ignored by this conversion
    const float r = 3.0F;
    std::memcpy(scan.data.data() + 1, &r, sizeof(float));

    pcl::PCLPointCloud2 cloud;
    lidar_conversions::LidarScanToPointCloud(info, scan, cloud);
    float x;
    std::memcpy(&x, cloud.data.data() + 0, sizeof(float));
    EXPECT_NEAR(x, 3.F, 1e-5);
}
