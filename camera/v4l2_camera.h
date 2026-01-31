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
#ifndef CAMERA_V4L2_CAMERA_H
#define CAMERA_V4L2_CAMERA_H

#include "camera/camera.h"

#include <cstdint>
#include <functional>
#include <linux/videodev2.h>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace rpicar::camera {

class V4L2Camera : public ICamera {
public:
    ~V4L2Camera() override;

    V4L2Camera(const V4L2Camera&) = delete;
    V4L2Camera& operator=(const V4L2Camera&) = delete;

    V4L2Camera(V4L2Camera&& other) noexcept;
    V4L2Camera& operator=(V4L2Camera&& other) noexcept;

    static std::optional<V4L2Camera>
    create_camera(uint16_t width, uint16_t height, ImageEncoding encoding, uint16_t fps);

    bool start_capture() override;
    bool stop_capture() override;

    [[nodiscard]]
    bool is_capturing() const override;

    [[nodiscard]]
    ImageFormat current_format() const override;

    void set_frame_handler(std::function<void(const CameraFrame&)> handler) override;

private:
    static constexpr const char* DEFAULT_VIDEO = "/dev/video0";
    static constexpr size_t BUFFER_COUNT = 10;
    static constexpr int WAIT_TIMEOUT = 30;

    struct CaptureBuffer {
        void* addr{nullptr};
        size_t length{0};
    };

    struct ControlBlock {
        std::mutex lock;
        V4L2Camera* camera_instance{nullptr};
    };

    explicit V4L2Camera(std::string device_path) noexcept;

    void destroy() noexcept;
    bool v4l2_stop_capture() noexcept;

    [[nodiscard]]
    std::optional<v4l2_format> v4l2_get_video_format() const;

    [[nodiscard]]
    std::optional<v4l2_capability> v4l2_query_capabilities() const;

    bool v4l2_set_fps(uint16_t fps);


    bool open();
    void close();

    [[nodiscard]]
    bool is_open() const;

    [[nodiscard]]
    bool is_streaming_supported() const;

    int select_format(unsigned int width, unsigned int height, uint32_t format);

    void print_camera_info() const;

    void start_capture_thread();

    void enumerate_capture_formats();

    bool init_buffers();
    void cleanup_buffers();

    static void capture_thread_func(int handle, std::stop_token token, std::shared_ptr<ControlBlock> control_block);
    void read_frame(const v4l2_buffer& buffer);

    static uint32_t convert_image_encoding_to_v4l2(ImageEncoding encoding);
    static std::optional<ImageEncoding> v4l2_to_image_encoding(uint32_t encoding);
    static std::string v4l2_encoding_to_string(uint32_t encoding);


    int handle_{-1};
    v4l2_capability capabilities_{};
    std::vector<v4l2_fmtdesc> formats_;
    std::string device_path_;
    v4l2_format current_fmt_{};

    std::vector<CaptureBuffer> buffers_;

    bool running_{false};

    std::jthread capture_thread_;
    std::shared_ptr<ControlBlock> control_block_;
    std::function<void(const CameraFrame&)> frame_handler_;
};

} // namespace rpicar::camera

#endif // CAMERA_V4L2_CAMERA_H
