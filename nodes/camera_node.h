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
#pragma once

#include "camera/v4l2_camera.h"
#include "camera/transcoder.h"
#include "rpicar/msg/camera_image_frame.hpp"

#include <cstddef>
#include <optional>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <image_transport/image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rpicar::nodes {

class CameraNode : public rclcpp::Node {
public:
    explicit CameraNode(const rclcpp::NodeOptions& options);
    ~CameraNode() override;

    CameraNode(const CameraNode&) = delete;
    CameraNode(CameraNode&&) = delete;
    CameraNode& operator=(const CameraNode&) = delete;
    CameraNode& operator=(CameraNode&&) = delete;

    [[nodiscard]]
    bool is_initialized() const;

private:
    void fill_image_msg(const std::vector<uint8_t>& frame_buf, sensor_msgs::msg::Image& img);

    std::optional<camera::V4L2Camera> camera_{};
    rclcpp::Publisher<msg::CameraImageFrame>::SharedPtr publisher_{};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_image_publisher_{};
    size_t frame_count_{0U};
};


} // namespace rpicar::nodes
