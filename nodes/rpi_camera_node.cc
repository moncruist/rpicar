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

#include "nodes/rpi_camera_node.h"

#include "camera/camera.h"
#include "camera/rpi_camera.h"
#include "nodes/camera_node_config.h"
#include "rpicar/msg/camera_image_frame.hpp"
#include "utils/logging.h"

#include <libyuv/convert_argb.h>
#include <libyuv/convert_from_argb.h>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <string>

namespace rpicar::nodes {

namespace {
constexpr std::string to_ros_encoding(const camera::ImageEncoding encoding) {
    switch (encoding) {
        case camera::ImageEncoding::RGB24:
            return std::string{sensor_msgs::image_encodings::RGB8}; // NOLINT(hicpp-no-array-decay)
        case camera::ImageEncoding::BGR24:
            return std::string{sensor_msgs::image_encodings::BGR8}; // NOLINT(hicpp-no-array-decay)
        case camera::ImageEncoding::YUV420:
            return std::string{sensor_msgs::image_encodings::NV21}; // NOLINT(hicpp-no-array-decay)
        case camera::ImageEncoding::YUV422:
            return std::string{sensor_msgs::image_encodings::YUYV}; // NOLINT(hicpp-no-array-decay)
        case camera::ImageEncoding::MJPEG:
        default:
            return std::string{"Unknown"};
    }
}
} // namespace

RpiCameraNode::RpiCameraNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("camera", options),
    camera_{CameraNodeConfig::CAPTURE_WIDTH,
            CameraNodeConfig::CAPTURE_HEIGHT,
            CameraNodeConfig::PUBLISH_ENCODING,
            CameraNodeConfig::FPS} {

    publisher_ = create_publisher<msg::CameraImageFrame>(
            "/camera_image", rclcpp::QoS{CameraNodeConfig::QOS_HISTORY_LENGHT}.best_effort());

    camera_.set_frame_handler([&](const camera::CameraFrame& frame) {
        frame_count_++;
        msg::CameraImageFrame msg{};
        fill_image_msg(frame.buffer, msg.frame);
        msg.frame_number = frame_count_;

        publisher_->publish(msg);
        LOG_DEBUG(LOGGER_NAME,
                  "Publish image. Frame number: {}, buffer size: {}, timestamp: {}.{:03}",
                  msg.frame_number,
                  msg.frame.data.size(),
                  msg.frame.header.stamp.sec,
                  msg.frame.header.stamp.nanosec / 1'000'000);
    });

    if (!camera_.start_capture()) {
        LOG_ERROR(LOGGER_NAME, "Failed to start camera capture");
        throw std::runtime_error{"Failed to start camera capture"};
    }
}

RpiCameraNode::~RpiCameraNode() {
    if (camera_.is_capturing()) {
        camera_.stop_capture();
    }
}

void RpiCameraNode::fill_image_msg(const std::span<uint8_t> frame_buf, sensor_msgs::msg::Image& img) const {
    img.width = CameraNodeConfig::CAPTURE_WIDTH;
    img.height = CameraNodeConfig::CAPTURE_HEIGHT;
    img.step = frame_buf.size() / CameraNodeConfig::CAPTURE_HEIGHT;
    img.encoding = to_ros_encoding(CameraNodeConfig::PUBLISH_ENCODING);
    img.header.stamp = this->now();
    img.is_bigendian = 0;
    img.data.assign(frame_buf.begin(), frame_buf.end());
}

} // namespace rpicar::nodes
