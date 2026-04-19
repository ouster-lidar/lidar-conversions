/**
 * @file lidar_scan_conversions.cpp
 * @brief Project LidarScan/LidarInfo into a pcl::PCLPointCloud2.
 */

#include "lidar_conversions/lidar_scan_conversions.h"

#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace ouster_ros {

namespace {

using lidar_msgs::msg::LidarChannel;
using lidar_msgs::msg::LidarInfo;
using lidar_msgs::msg::LidarScan;
using sensor_msgs::msg::PointField;

size_t point_field_datatype_size(uint8_t dt) {
    switch (dt) {
        case PointField::INT8:
        case PointField::UINT8:    return 1;
        case PointField::INT16:
        case PointField::UINT16:   return 2;
        case PointField::INT32:
        case PointField::UINT32:
        case PointField::FLOAT32:  return 4;
        case PointField::FLOAT64:  return 8;
        default:
            throw std::invalid_argument(
                "LidarScanToPointCloud: unknown LidarChannel.datatype");
    }
}

bool iequals_ascii(const std::string& a, const std::string& b) {
    return a.size() == b.size() &&
           std::equal(a.begin(), a.end(), b.begin(),
                      [](unsigned char x, unsigned char y) {
                          return std::tolower(x) == std::tolower(y);
                      });
}

// Load a POD scalar from the given byte pointer, swapping bytes when the source
// is big-endian on a little-endian host (ROS2 targets). Single-byte types skip swap.
template <typename T>
inline T load_scalar(const uint8_t* p, bool byte_swap) {
    T v;
    std::memcpy(&v, p, sizeof(T));
    if (byte_swap && sizeof(T) > 1) {
        auto* b = reinterpret_cast<uint8_t*>(&v);
        std::reverse(b, b + sizeof(T));
    }
    return v;
}

using RangeReader = double (*)(const uint8_t*, bool);

RangeReader make_range_reader(uint8_t datatype) {
    switch (datatype) {
        case PointField::UINT8:
            return [](const uint8_t* p, bool)    { return static_cast<double>(*p); };
        case PointField::INT8:
            return [](const uint8_t* p, bool)    {
                return static_cast<double>(*reinterpret_cast<const int8_t*>(p));
            };
        case PointField::UINT16:
            return [](const uint8_t* p, bool be) { return static_cast<double>(load_scalar<uint16_t>(p, be)); };
        case PointField::INT16:
            return [](const uint8_t* p, bool be) { return static_cast<double>(load_scalar<int16_t>(p, be)); };
        case PointField::UINT32:
            return [](const uint8_t* p, bool be) { return static_cast<double>(load_scalar<uint32_t>(p, be)); };
        case PointField::INT32:
            return [](const uint8_t* p, bool be) { return static_cast<double>(load_scalar<int32_t>(p, be)); };
        case PointField::FLOAT32:
            return [](const uint8_t* p, bool be) { return static_cast<double>(load_scalar<float>(p, be)); };
        case PointField::FLOAT64:
            return [](const uint8_t* p, bool be) { return load_scalar<double>(p, be); };
        default:
            throw std::invalid_argument(
                "LidarScanToPointCloud: unsupported range datatype");
    }
}

const LidarChannel* find_channel(const LidarScan& scan, const std::string& name) {
    const auto it = std::find_if(
        scan.channels.begin(), scan.channels.end(),
        [&](const LidarChannel& ch) { return iequals_ascii(ch.name, name); });
    return it == scan.channels.end() ? nullptr : &*it;
}

void validate_scan_geometry(const LidarScan& scan) {
    if (scan.height == 0 || scan.width == 0) {
        throw std::invalid_argument(
            "LidarScanToPointCloud: height and width must be non-zero");
    }
}

// Planar layout check: the channel's contiguous block [offset, offset + h*w*count*es)
// must fit inside data[]. Returns the per-pixel byte stride for convenience.
size_t validate_channel_fits(const LidarScan& scan, const LidarChannel& ch) {
    if (ch.count == 0) {
        throw std::invalid_argument(
            "LidarScanToPointCloud: channel count must be non-zero");
    }
    const size_t es = point_field_datatype_size(ch.datatype);
    const size_t bytes_per_pixel = es * static_cast<size_t>(ch.count);
    const uint64_t block_bytes = static_cast<uint64_t>(scan.height) *
                                 static_cast<uint64_t>(scan.width) *
                                 static_cast<uint64_t>(bytes_per_pixel);
    if (static_cast<uint64_t>(ch.offset) + block_bytes >
        static_cast<uint64_t>(scan.data.size())) {
        throw std::invalid_argument(
            "LidarScanToPointCloud: channel block extends past data[]");
    }
    return bytes_per_pixel;
}

// Return a per-row elevation table (radians). Either the explicit info table, or a
// linear interpolation of [vertical_fov_min, vertical_fov_max]. Size-validates once.
std::vector<float> build_vertical_angles(const LidarInfo& info, uint32_t height) {
    if (!info.vertical_angles.empty()) {
        if (info.vertical_angles.size() != height) {
            throw std::invalid_argument(
                "LidarScanToPointCloud: vertical_angles size must match scan height");
        }
        return {info.vertical_angles.begin(), info.vertical_angles.end()};
    }
    std::vector<float> out(height, info.vertical_fov_min);
    if (height >= 2) {
        const float span = info.vertical_fov_max - info.vertical_fov_min;
        const float inv = 1.F / static_cast<float>(height - 1);
        for (uint32_t i = 0; i < height; ++i) {
            out[i] = info.vertical_fov_min + static_cast<float>(i) * inv * span;
        }
    }
    return out;
}

std::vector<float> build_horizontal_angles(const LidarInfo& info, uint32_t width) {
    if (!info.horizontal_angles.empty()) {
        if (info.horizontal_angles.size() != width) {
            throw std::invalid_argument(
                "LidarScanToPointCloud: horizontal_angles size must match scan width");
        }
        return {info.horizontal_angles.begin(), info.horizontal_angles.end()};
    }
    std::vector<float> out(width, info.horizontal_fov_min);
    if (width >= 2) {
        const float span = info.horizontal_fov_max - info.horizontal_fov_min;
        const float inv = 1.F / static_cast<float>(width - 1);
        for (uint32_t i = 0; i < width; ++i) {
            out[i] = info.horizontal_fov_min + static_cast<float>(i) * inv * span;
        }
    }
    return out;
}

std::vector<float> build_beam_azimuth_offsets(const LidarInfo& info, uint32_t height) {
    if (info.beam_azimuth_angles.empty()) return std::vector<float>(height, 0.F);
    if (info.beam_azimuth_angles.size() != height) {
        throw std::invalid_argument(
            "LidarScanToPointCloud: beam_azimuth_angles size must match scan height");
    }
    return {info.beam_azimuth_angles.begin(), info.beam_azimuth_angles.end()};
}

}  // namespace

void LidarScanToPointCloud(const LidarInfo& info, const LidarScan& scan,
                           pcl::PCLPointCloud2& cloud_out,
                           const LidarScanToPointCloudOptions& opts) {
    validate_scan_geometry(scan);

    const LidarChannel* range_ch = find_channel(scan, opts.range_channel_name);
    if (!range_ch) {
        throw std::invalid_argument(
            "LidarScanToPointCloud: range channel not found: " + opts.range_channel_name);
    }
    const size_t range_bytes_per_pixel = validate_channel_fits(scan, *range_ch);

    const bool big_endian = scan.is_bigendian != 0;
    const uint32_t h = scan.height;
    const uint32_t w = scan.width;

    // Output layout: packed float32 xyz, organized (H*W) or unorganized (1x(H*W)).
    cloud_out.fields.resize(3);
    const char* kXyzNames[] = {"x", "y", "z"};
    for (size_t i = 0; i < 3; ++i) {
        cloud_out.fields[i].name = kXyzNames[i];
        cloud_out.fields[i].offset = static_cast<uint32_t>(4 * i);
        cloud_out.fields[i].datatype = PointField::FLOAT32;
        cloud_out.fields[i].count = 1;
    }
    cloud_out.point_step = 12;
    cloud_out.is_bigendian = false;
    cloud_out.is_dense = false;

    if (opts.organized) {
        cloud_out.height = h;
        cloud_out.width = w;
    } else {
        cloud_out.height = 1;
        cloud_out.width = h * w;
    }
    cloud_out.row_step = cloud_out.point_step * cloud_out.width;
    cloud_out.data.assign(static_cast<size_t>(cloud_out.row_step) * cloud_out.height, 0);

    // Hoisted once-per-scan work: angle tables, per-row/column trig, decoder.
    const auto elev     = build_vertical_angles(info, h);
    const auto azim_col = build_horizontal_angles(info, w);
    const auto azim_row = build_beam_azimuth_offsets(info, h);
    const RangeReader read_range = make_range_reader(range_ch->datatype);

    std::vector<float> cos_el(h), sin_el(h);
    for (uint32_t u = 0; u < h; ++u) {
        cos_el[u] = std::cos(elev[u]);
        sin_el[u] = std::sin(elev[u]);
    }
    std::vector<float> cos_az_col(w), sin_az_col(w);
    for (uint32_t v = 0; v < w; ++v) {
        cos_az_col[v] = std::cos(azim_col[v]);
        sin_az_col[v] = std::sin(azim_col[v]);
    }

    const double mult = info.range_multiplier;
    const double offs = info.range_offset;
    const float qnan = std::numeric_limits<float>::quiet_NaN();
    bool any_valid = false;
    bool any_invalid = false;

    // Base pointer of the range channel's planar block and its per-pixel stride.
    const uint8_t* range_base = scan.data.data() + range_ch->offset;

    for (uint32_t u = 0; u < h; ++u) {
        const float ce = cos_el[u];
        const float se = sin_el[u];
        // Apply per-row azimuth offset via the angle-sum identity so the inner
        // loop does no trig: cos(α+β)=cosα cosβ−sinα sinβ; sin(α+β)=sinα cosβ+cosα sinβ.
        const float row_cos = std::cos(azim_row[u]);
        const float row_sin = std::sin(azim_row[u]);

        const uint8_t* row_range =
            range_base + static_cast<size_t>(u) * w * range_bytes_per_pixel;
        const size_t out_row_off = opts.organized
            ? static_cast<size_t>(u) * cloud_out.row_step
            : static_cast<size_t>(u) * w * cloud_out.point_step;
        uint8_t* out_row = cloud_out.data.data() + out_row_off;

        for (uint32_t v = 0; v < w; ++v) {
            const double raw = read_range(
                row_range + static_cast<size_t>(v) * range_bytes_per_pixel, big_endian);
            const double range_m = raw * mult + offs;

            float x = qnan, y = qnan, z = qnan;
            if (range_m > 0.0 && std::isfinite(range_m)) {
                const float cos_az = cos_az_col[v] * row_cos - sin_az_col[v] * row_sin;
                const float sin_az = sin_az_col[v] * row_cos + cos_az_col[v] * row_sin;
                const float rf = static_cast<float>(range_m);
                x = rf * ce * cos_az;
                y = rf * ce * sin_az;
                z = rf * se;
                any_valid = true;
            } else {
                any_invalid = true;
            }

            uint8_t* dst = out_row + static_cast<size_t>(v) * cloud_out.point_step;
            std::memcpy(dst + 0, &x, sizeof(float));
            std::memcpy(dst + 4, &y, sizeof(float));
            std::memcpy(dst + 8, &z, sizeof(float));
        }
    }

    cloud_out.is_dense = any_valid && !any_invalid;
}

}  // namespace ouster_ros
