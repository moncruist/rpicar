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
#ifdef RPI_BUILD
#include "nodes/rpi_camera_node.h"
#else
#include "nodes/camera_node.h"
#endif


#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
#ifdef RPI_BUILD
    auto camera_node = std::make_shared<rpicar::nodes::RpiCameraNode>(rclcpp::NodeOptions{});
#else
    auto camera_node = std::make_shared<rpicar::nodes::CameraNode>(rclcpp::NodeOptions{});
#endif
    rclcpp::spin(camera_node);
    rclcpp::shutdown();
    return 0;
}