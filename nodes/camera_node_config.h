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

#ifndef NODES_CAMERA_NODE_CONFIG_H
#define NODES_CAMERA_NODE_CONFIG_H

#include "camera/camera.h"

#include <cstdint>

namespace rpicar::nodes {

struct CameraNodeConfig {
    static constexpr uint16_t CAPTURE_WIDTH{800U};
    static constexpr uint16_t CAPTURE_HEIGHT{600U};
    static constexpr camera::ImageEncoding CAPTURE_ENCODING{camera::ImageEncoding::YUV422};
    static constexpr camera::ImageEncoding PUBLISH_ENCODING{camera::ImageEncoding::RGB24};
    static constexpr uint16_t FPS{15U};
    static constexpr std::size_t QOS_HISTORY_LENGHT{10U};
};

} // namespace rpicar::nodes

#endif // NODES_CAMERA_NODE_CONFIG_H
