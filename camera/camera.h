// rpicar
// Copyright (C) 2021-2025 Konstantin Zhukov
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
#ifndef CAMERA_CAMERA_H
#define CAMERA_CAMERA_H

#include <chrono>
#include <cstdint>
#include <functional>
#include <span>
#include <string>

#include "utils/compat.h"

namespace rpicar::camera {

static constexpr std::string CAMERA_LOGGER{"camera"};

enum class ImageEncoding : uint8_t { RGB24, BGR24, YUV420, YUV422, MJPEG };

constexpr const char* to_string(const ImageEncoding& encoding) {
    switch (encoding) {
        case rpicar::camera::ImageEncoding::RGB24:
            return "RGB24";
        case rpicar::camera::ImageEncoding::BGR24:
            return "BGR24";
        case rpicar::camera::ImageEncoding::YUV420:
            return "YUV420";
        case rpicar::camera::ImageEncoding::YUV422:
            return "YUV422";
        case rpicar::camera::ImageEncoding::MJPEG:
            return "MJPEG";
        default:
            return "Unknown";
    }
}

struct ImageFormat {
    uint16_t width{};
    uint16_t height{};
    ImageEncoding encoding{};
};

struct CameraFrame {
    ImageFormat format{};
    std::chrono::time_point<std::chrono::steady_clock> timestamp{};
    std::span<uint8_t> buffer{};
};

// NOLINTNEXTLINE(hicpp-special-member-functions)
class ICamera {
public:
    virtual ~ICamera() = default;

    virtual bool start_capture() = 0;
    virtual bool stop_capture() = 0;
    [[nodiscard]]
    virtual bool is_capturing() const = 0;

    [[nodiscard]]
    virtual ImageFormat current_format() const = 0;

    virtual void set_frame_handler(std::function<void(const CameraFrame&)> handler) = 0;
};

} // namespace rpicar::camera

#endif // CAMERA_CAMERA_H
