// rpicar
// Copyright (C) 2025 Konstantin Zhukov
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "nodes/camera_node.h"

#include "camera/camera.h"
#include "camera/transcoder.h"
#include "camera/v4l2_camera.h"
#include "nodes/camera_node_config.h"
#include "rpicar/msg/camera_image_frame.hpp"
#include "utils/logging.h"

#include <cstddef>
#include <libyuv/convert_argb.h>
#include <libyuv/convert_from_argb.h>
#include <sensor_msgs/image_encodings.hpp>
#include <string>
#include <utility>

namespace rpicar::nodes {

namespace {
constexpr std::string_view to_ros_encoding(const camera::ImageEncoding encoding) {
    using namespace std::string_view_literals;

    switch (encoding) {
        case camera::ImageEncoding::RGB24:
            return std::string_view{sensor_msgs::image_encodings::RGB8}; // NOLINT(hicpp-no-array-decay)
        case camera::ImageEncoding::BGR24:
            return std::string_view{sensor_msgs::image_encodings::BGR8}; // NOLINT(hicpp-no-array-decay)
        case camera::ImageEncoding::YUV420:
            return std::string_view{sensor_msgs::image_encodings::NV21}; // NOLINT(hicpp-no-array-decay)
        case camera::ImageEncoding::YUV422:
            return std::string_view{sensor_msgs::image_encodings::YUYV}; // NOLINT(hicpp-no-array-decay)
        case camera::ImageEncoding::MJPEG:
        default:
            return "Unknown"sv;
    }
}

constexpr int calculate_stride(const std::size_t buf_size, const std::size_t height) {
    return static_cast<int>(buf_size / height);
}

constexpr std::size_t
calculate_frame_size(const std::size_t channels, const std::size_t width, const std::size_t height) {
    return channels * width * height;
}

} // namespace

CameraNode::CameraNode(const rclcpp::NodeOptions& options) : rclcpp::Node("camera", options) {
    camera_ = camera::V4L2Camera::create_camera(CameraNodeConfig::CAPTURE_WIDTH,
                                                CameraNodeConfig::CAPTURE_HEIGHT,
                                                CameraNodeConfig::CAPTURE_ENCODING,
                                                CameraNodeConfig::FPS);

    if (!camera_.has_value()) {
        LOG_ERROR("camera_node", "Failed to initialize camera");
        return;
    }

    auto& camera{camera_.value()};

    publisher_ = create_publisher<msg::CameraImageFrame>(
            "/camera_image", rclcpp::QoS{CameraNodeConfig::QOS_HISTORY_LENGHT}.best_effort());
    raw_image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
            "/image", rclcpp::QoS{CameraNodeConfig::QOS_HISTORY_LENGHT}.best_effort());

    camera.set_frame_handler([&](const camera::CameraFrame& frame) {
        frame_count_++;

        std::vector<uint8_t> argb_buf{};
        argb_buf.resize(calculate_frame_size(4U, CameraNodeConfig::CAPTURE_WIDTH, CameraNodeConfig::CAPTURE_HEIGHT));
        int ret = libyuv::YUY2ToARGB(frame.buffer.data(),
                                     calculate_stride(frame.buffer.size(), CameraNodeConfig::CAPTURE_HEIGHT),
                                     argb_buf.data(),
                                     calculate_stride(argb_buf.size(), CameraNodeConfig::CAPTURE_HEIGHT),
                                     CameraNodeConfig::CAPTURE_WIDTH,
                                     CameraNodeConfig::CAPTURE_HEIGHT);
        LOG_DEBUG("camera_node", "YUV to ARGB convert result: {}, output buffer size: {}", ret, argb_buf.size());

        std::vector<uint8_t> raw_buf{};
        raw_buf.resize(calculate_frame_size(3U, CameraNodeConfig::CAPTURE_WIDTH, CameraNodeConfig::CAPTURE_HEIGHT));
        ret = libyuv::ARGBToRAW(argb_buf.data(),
                                calculate_stride(argb_buf.size(), CameraNodeConfig::CAPTURE_HEIGHT),
                                raw_buf.data(),
                                calculate_stride(raw_buf.size(), CameraNodeConfig::CAPTURE_HEIGHT),
                                CameraNodeConfig::CAPTURE_WIDTH,
                                CameraNodeConfig::CAPTURE_HEIGHT);

        LOG_DEBUG("camera_node", "ARGB to RAW convert result: {}, output buffer size: {}", ret, raw_buf.size());

        msg::CameraImageFrame msg{};
        msg.frame_number = frame_count_;
        msg.frame.width = CameraNodeConfig::CAPTURE_WIDTH;
        msg.frame.height = CameraNodeConfig::CAPTURE_HEIGHT;
        msg.frame.step = argb_buf.size() / CameraNodeConfig::CAPTURE_HEIGHT;
        msg.frame.encoding = to_ros_encoding(CameraNodeConfig::PUBLISH_ENCODING);
        msg.frame.header.stamp = this->now();
        msg.frame.is_bigendian = 0;
        msg.frame.data = argb_buf;

        publisher_->publish(msg);
        LOG_INFO("camera_node", "Publish image");

        sensor_msgs::msg::Image image_msg{};
        fill_image_msg(raw_buf, image_msg);
        image_msg.width = CameraNodeConfig::CAPTURE_WIDTH;
        image_msg.height = CameraNodeConfig::CAPTURE_HEIGHT;
        image_msg.step = raw_buf.size() / CameraNodeConfig::CAPTURE_HEIGHT;
        image_msg.encoding = to_ros_encoding(CameraNodeConfig::PUBLISH_ENCODING);
        image_msg.header.stamp = this->now();
        image_msg.is_bigendian = 0;
        image_msg.data = raw_buf;

        raw_image_publisher_->publish(image_msg);
        LOG_INFO("camera_node", "Publish raw image");
    });

    camera.start_capture();
}

CameraNode::~CameraNode() {
    if (camera_.has_value() && camera_->is_capturing()) {
        camera_->stop_capture();
    }
}

bool CameraNode::is_initialized() const {
    return camera_.has_value();
}

void CameraNode::fill_image_msg(const std::vector<uint8_t>& frame_buf, sensor_msgs::msg::Image& img) const {
    img.width = CameraNodeConfig::CAPTURE_WIDTH;
    img.height = CameraNodeConfig::CAPTURE_HEIGHT;
    img.step = frame_buf.size() / CameraNodeConfig::CAPTURE_HEIGHT;
    img.encoding = to_ros_encoding(CameraNodeConfig::PUBLISH_ENCODING);
    img.header.stamp = this->now();
    img.is_bigendian = 0;
    img.data = frame_buf;
}

} // namespace rpicar::nodes
