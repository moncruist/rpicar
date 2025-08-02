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

#include <libyuv/convert_argb.h>
#include <libyuv/convert_from_argb.h>
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

CameraNode::CameraNode(const rclcpp::NodeOptions& options) : rclcpp::Node("camera", options) {
    camera_ = camera::V4L2Camera::create_camera(CameraNodeConfig::CAPTURE_WIDTH,
                                                CameraNodeConfig::CAPTURE_HEIGHT,
                                                CameraNodeConfig::CAPTURE_ENCODING,
                                                CameraNodeConfig::FPS);

    if (!camera_.has_value()) {
        LOG_ERROR("camera_node", "Failed to initialize camera");
        return;
    }

    publisher_ = create_publisher<msg::CameraImageFrame>("/camera_image", rclcpp::QoS{10}.best_effort());
    raw_image_publisher_ = create_publisher<sensor_msgs::msg::Image>("/image", rclcpp::QoS{10}.best_effort());

    camera_->set_frame_handler([&](const camera::CameraFrame& frame) {
        frame_count_++;


        std::vector<uint8_t> argb_buf{};
        argb_buf.resize(4U * CameraNodeConfig::CAPTURE_WIDTH * CameraNodeConfig::CAPTURE_HEIGHT);
        int ret = libyuv::YUY2ToARGB(frame.buffer.data(),
                                     frame.buffer.size() / CameraNodeConfig::CAPTURE_HEIGHT,
                                     argb_buf.data(),
                                     argb_buf.size() / CameraNodeConfig::CAPTURE_HEIGHT,
                                     CameraNodeConfig::CAPTURE_WIDTH,
                                     CameraNodeConfig::CAPTURE_HEIGHT);
        LOG_DEBUG("camera_node", "YUV to ARGB convert result: {}, output buffer size: {}", ret, argb_buf.size());

        std::vector<uint8_t> raw_buf{};
        raw_buf.resize(3U * CameraNodeConfig::CAPTURE_WIDTH * CameraNodeConfig::CAPTURE_HEIGHT);
        ret = libyuv::ARGBToRAW(argb_buf.data(),
                                argb_buf.size() / CameraNodeConfig::CAPTURE_HEIGHT,
                                raw_buf.data(),
                                raw_buf.size() / CameraNodeConfig::CAPTURE_HEIGHT,
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

    camera_->start_capture();
}

CameraNode::~CameraNode() {
    if (camera_.has_value() && camera_->is_capturing()) {
        camera_->stop_capture();
    }
}

bool CameraNode::is_initialized() const {
    return camera_.has_value();
}

void CameraNode::fill_image_msg(const std::vector<uint8_t>& frame_buf, sensor_msgs::msg::Image& img) {
    img.width = CameraNodeConfig::CAPTURE_WIDTH;
    img.height = CameraNodeConfig::CAPTURE_HEIGHT;
    img.step = frame_buf.size() / CameraNodeConfig::CAPTURE_HEIGHT;
    img.encoding = to_ros_encoding(CameraNodeConfig::PUBLISH_ENCODING);
    img.header.stamp = this->now();
    img.is_bigendian = 0;
    img.data = frame_buf;
}

} // namespace rpicar::nodes
