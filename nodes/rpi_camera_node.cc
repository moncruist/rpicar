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
#include "camera/transcoder.h"
#include "nodes/camera_node_config.h"
#include "rpicar/msg/camera_image_frame.hpp"
#include "utils/logging.h"

#include <libyuv/convert_argb.h>
#include <libyuv/convert_from_argb.h>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <string>
#include <utility>

namespace rpicar::nodes {

namespace {
constexpr std::string to_ros_encoding(camera::ImageEncoding encoding) {
    switch (encoding) {
        case rpicar::camera::ImageEncoding::RGB24:
            return std::string{sensor_msgs::image_encodings::RGB8};
        case rpicar::camera::ImageEncoding::BGR24:
            return std::string{sensor_msgs::image_encodings::BGR8};
        case rpicar::camera::ImageEncoding::YUV420:
            return std::string{sensor_msgs::image_encodings::NV21};
        case rpicar::camera::ImageEncoding::YUV422:
            return std::string{sensor_msgs::image_encodings::YUYV};
        case rpicar::camera::ImageEncoding::MJPEG:
        default:
            return std::string{"Unknown"};
    }
}
} // namespace

RpiCameraNode::RpiCameraNode(const rclcpp::NodeOptions& options) : rclcpp::Node("camera", options) {
    camera_ = camera::RpiCamera::create_camera(CameraNodeConfig::CAPTURE_WIDTH,
                                               CameraNodeConfig::CAPTURE_HEIGHT,
                                               CameraNodeConfig::PUBLISH_ENCODING,
                                               CameraNodeConfig::FPS);

    if (!camera_.has_value()) {
        LOG_ERROR(LOGGER_NAME, "Failed to initialize camera");
        throw std::runtime_error{"Failed to initialize camera"};
    }

    publisher_ = create_publisher<msg::CameraImageFrame>("/camera_image", rclcpp::QoS{10});
    raw_image_publisher_ = create_publisher<sensor_msgs::msg::Image>("/image", rclcpp::QoS{10});

    camera_->set_frame_handler([&](const camera::CameraFrame& frame) {
        frame_count_++;

        LOG_DEBUG(LOGGER_NAME, "Received a new frame from RPI camera. Buffer size: {}", frame.buffer.size());

        msg::CameraImageFrame msg{};
        fill_image_msg(frame.buffer, msg.frame);
        msg.frame_number = frame_count_;

        publisher_->publish(msg);
        LOG_INFO(LOGGER_NAME, "Publish image");

        sensor_msgs::msg::Image image_msg{};
        fill_image_msg(frame.buffer, image_msg);

        raw_image_publisher_->publish(image_msg);
        LOG_INFO(LOGGER_NAME, "Publish raw image");
    });

    if (!camera_->start_capture()) {
        LOG_ERROR(LOGGER_NAME, "Failed to start camera capture");
        throw std::runtime_error{"Failed to start camera capture"};
    }
}

RpiCameraNode::~RpiCameraNode() {
    if (camera_.has_value() && camera_->is_capturing()) {
        camera_->stop_capture();
    }
}

bool RpiCameraNode::is_initialized() const {
    return camera_.has_value();
}

void RpiCameraNode::fill_image_msg(const std::span<uint8_t> frame_buf, sensor_msgs::msg::Image& img) {
    img.width = CameraNodeConfig::CAPTURE_WIDTH;
    img.height = CameraNodeConfig::CAPTURE_HEIGHT;
    img.step = frame_buf.size() / CameraNodeConfig::CAPTURE_HEIGHT;
    img.encoding = to_ros_encoding(CameraNodeConfig::PUBLISH_ENCODING);
    img.header.stamp = this->now();
    img.is_bigendian = 0;
    img.data.assign(frame_buf.begin(), frame_buf.end());
}

} // namespace rpicar::nodes
