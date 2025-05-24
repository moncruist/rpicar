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

#pragma once

#include "camera/camera.h"

#include <libcamera/libcamera.h>
#include <list>
#include <map>
#include <queue>
#include <span>
#include <vector>

namespace rpicar::camera {

class RpiCamera : public ICamera {
public:
    ~RpiCamera() override;

    RpiCamera(const RpiCamera&) = delete;
    RpiCamera(RpiCamera&& other) noexcept;

    RpiCamera& operator=(const RpiCamera&) = delete;
    RpiCamera& operator=(RpiCamera&& other) noexcept;

    [[nodiscard]]
    static std::optional<RpiCamera>
    create_camera(uint16_t width, uint16_t height, ImageEncoding encoding, uint16_t fps);

    bool start_capture() override;
    bool stop_capture() override;

    [[nodiscard]]
    ImageFormat current_format() const override;

    [[nodiscard]]
    bool is_capturing() const override;

    void set_frame_handler(std::function<void(const CameraFrame&)> handler) override;

private:
    static constexpr auto RPI_CAMERA_ID{"imx708"};

    RpiCamera() noexcept;

    bool configure_camera(uint16_t width, uint16_t height, ImageEncoding encoding, uint16_t fps);

    bool allocate_buffers();
    void deallocate_buffers();

    bool make_requests(uint16_t fps);

    void requested_completed_handler(libcamera::Request* request);

    std::unique_ptr<libcamera::CameraManager> manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> configuration_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::map<libcamera::FrameBuffer*, std::vector<std::span<uint8_t>>> mapped_buffers_;
    std::map<libcamera::Stream*, std::queue<libcamera::FrameBuffer*>> frame_buffers_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    bool camera_acquired_{false};
    bool camera_started_{false};
    std::function<void(const CameraFrame&)> frame_handler_;
};

} // namespace rpicar::camera
