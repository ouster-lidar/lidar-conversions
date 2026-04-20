/**
 * @file python_bindings.cpp
 * @brief pybind11 module exposing LidarScanToPointCloud to rclpy consumers.
 *
 * The module is named `lidar_conversions_py` (Python identifiers can't contain
 * hyphens, so we don't mirror the ROS package name `lidar-conversions`). It
 * takes duck-typed rclpy LidarInfo / LidarScan objects — we read their
 * attributes directly — so it doesn't depend on rosidl_python type hooks.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <lidar_conversions/lidar_scan_conversions.h>
#include <lidar_msgs/msg/lidar_channel.hpp>
#include <lidar_msgs/msg/lidar_info.hpp>
#include <lidar_msgs/msg/lidar_scan.hpp>
#include <pcl/PCLPointCloud2.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace py = pybind11;

namespace {

// Copy a Python float32 sequence (rclpy exposes float32[] as array.array('f'))
// into a std::vector<float>. Buffer-protocol fast path for array.array / numpy;
// generic iteration fallback for plain lists.
std::vector<float> to_float_vector(const py::object& src) {
    if (src.is_none()) return {};
    try {
        py::buffer buf = py::reinterpret_borrow<py::buffer>(src);
        py::buffer_info info = buf.request();
        if (info.format == py::format_descriptor<float>::format()) {
            const auto* p = static_cast<const float*>(info.ptr);
            size_t n = 1;
            for (auto d : info.shape) n *= static_cast<size_t>(d);
            return std::vector<float>(p, p + n);
        }
    } catch (...) {
        // Not a buffer; fall through to generic iteration.
    }
    std::vector<float> out;
    for (auto h : src) out.push_back(h.cast<float>());
    return out;
}

// scan.data is rclpy's array.array('B', ...) — buffer-protocol compatible.
// We copy once into the C++ message; for a typical 2048x128 scan (~2 MB) this
// is well below 1 ms at DDR bandwidth and keeps the binding simple.
std::vector<uint8_t> to_byte_vector(const py::object& src) {
    if (src.is_none()) return {};
    py::buffer buf = py::reinterpret_borrow<py::buffer>(src);
    py::buffer_info info = buf.request();
    size_t n = static_cast<size_t>(info.itemsize);
    for (auto d : info.shape) n *= static_cast<size_t>(d);
    const auto* p = static_cast<const uint8_t*>(info.ptr);
    return std::vector<uint8_t>(p, p + n);
}

lidar_msgs::msg::LidarInfo py_to_info(const py::object& info_py) {
    lidar_msgs::msg::LidarInfo info;
    info.range_multiplier = info_py.attr("range_multiplier").cast<double>();
    info.range_offset = info_py.attr("range_offset").cast<double>();
    info.vertical_fov_min = info_py.attr("vertical_fov_min").cast<float>();
    info.vertical_fov_max = info_py.attr("vertical_fov_max").cast<float>();
    info.horizontal_fov_min = info_py.attr("horizontal_fov_min").cast<float>();
    info.horizontal_fov_max = info_py.attr("horizontal_fov_max").cast<float>();
    info.vertical_angles = to_float_vector(info_py.attr("vertical_angles"));
    info.horizontal_angles = to_float_vector(info_py.attr("horizontal_angles"));
    info.beam_azimuth_angles =
        to_float_vector(info_py.attr("beam_azimuth_angles"));
    return info;
}

lidar_msgs::msg::LidarScan py_to_scan(const py::object& scan_py) {
    lidar_msgs::msg::LidarScan scan;
    scan.height = scan_py.attr("height").cast<uint32_t>();
    scan.width = scan_py.attr("width").cast<uint32_t>();
    scan.is_bigendian = scan_py.attr("is_bigendian").cast<uint8_t>();

    py::list channels_py = scan_py.attr("channels").cast<py::list>();
    scan.channels.reserve(channels_py.size());
    for (auto ch_py : channels_py) {
        lidar_msgs::msg::LidarChannel ch;
        ch.name = py::getattr(ch_py, "name").cast<std::string>();
        ch.offset = py::getattr(ch_py, "offset").cast<uint32_t>();
        ch.datatype = py::getattr(ch_py, "datatype").cast<uint8_t>();
        ch.count = py::getattr(ch_py, "count").cast<uint32_t>();
        scan.channels.push_back(std::move(ch));
    }
    scan.data = to_byte_vector(scan_py.attr("data"));
    return scan;
}

// Returns (xyz_bytes, height, width, is_dense). The caller builds the
// sensor_msgs/PointCloud2 message — we stay oblivious to rosidl Python types.
py::tuple lidar_scan_to_pointcloud2_data(const py::object& info_py,
                                         const py::object& scan_py,
                                         const std::string& range_channel_name,
                                         bool organized) {
    const auto info = py_to_info(info_py);
    const auto scan = py_to_scan(scan_py);

    lidar_conversions::LidarScanToPointCloudOptions opts;
    opts.range_channel_name = range_channel_name;
    opts.organized = organized;

    pcl::PCLPointCloud2 cloud;
    lidar_conversions::LidarScanToPointCloud(info, scan, cloud, opts);

    // py::bytes copies the buffer; the PCL cloud is destroyed on return so we
    // can't hand out a zero-copy view.
    py::bytes data(reinterpret_cast<const char*>(cloud.data.data()),
                   cloud.data.size());
    return py::make_tuple(std::move(data),
                          static_cast<uint32_t>(cloud.height),
                          static_cast<uint32_t>(cloud.width),
                          static_cast<bool>(cloud.is_dense));
}

}  // namespace

PYBIND11_MODULE(lidar_conversions_py, m) {
    m.doc() =
        "Python bindings for lidar_conversions::LidarScanToPointCloud. "
        "Accepts rclpy LidarInfo / LidarScan objects and returns the raw "
        "float32 xyz buffer that the caller wraps in a sensor_msgs/PointCloud2.";

    m.def("lidar_scan_to_pointcloud2_data", &lidar_scan_to_pointcloud2_data,
          py::arg("info"), py::arg("scan"),
          py::arg("range_channel_name") = std::string("range"),
          py::arg("organized") = true,
          R"pbdoc(
Project a LidarScan to a PointCloud2-compatible xyz float32 buffer.

Parameters
----------
info : lidar_msgs.msg.LidarInfo
    Geometry and range scaling for the scan.
scan : lidar_msgs.msg.LidarScan
    Planar-layout scan produced by the driver.
range_channel_name : str, default "range"
    Case-insensitive channel name used as the range source.
organized : bool, default True
    If True the output is (height, width); otherwise (1, height*width).

Returns
-------
(data, height, width, is_dense) : tuple(bytes, int, int, bool)
    Packed float32 xyz buffer (point_step = 12) and cloud dimensions.
)pbdoc");
}
