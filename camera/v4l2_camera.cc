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

#include "camera/v4l2_camera.h"

#include "camera/camera.h"
#include "utils/error_utils.h"
#include "utils/logging.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <linux/videodev2.h>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stop_token>
#include <string>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <utility>

namespace rpicar::camera {

namespace {

int xioctl(int fd, unsigned long int request, void* arg) {
    int r{};

    do {
        // NOLINTNEXTLINE(hicpp-vararg)
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);

    return r;
}

struct EncodingMappingEntry {
    ImageEncoding internal;
    uint32_t v4l2;
};

const std::array<EncodingMappingEntry, 5> kImageEncodingMappingTable{
        {{.internal = ImageEncoding::RGB24, .v4l2 = V4L2_PIX_FMT_RGB24},
         {.internal = ImageEncoding::BGR24, .v4l2 = V4L2_PIX_FMT_BGR24},
         {.internal = ImageEncoding::YUV420, .v4l2 = V4L2_PIX_FMT_YUV420},
         {.internal = ImageEncoding::YUV422, .v4l2 = V4L2_PIX_FMT_YUYV},
         {.internal = ImageEncoding::MJPEG, .v4l2 = V4L2_PIX_FMT_MJPEG}}};

constexpr std::chrono::time_point<std::chrono::steady_clock> from_timeval(const timeval& tv) {
    return std::chrono::time_point<std::chrono::steady_clock>{std::chrono::seconds{tv.tv_sec} +
                                                              std::chrono::microseconds{tv.tv_usec}};
}

} // namespace

V4L2Camera::V4L2Camera(std::string device_path) noexcept
  : device_path_(std::move(device_path)), control_block_(std::make_shared<ControlBlock>()) {
    control_block_->camera_instance = this; // Safe to do without locking since no thread is running
}

V4L2Camera::V4L2Camera(V4L2Camera&& other) noexcept {
    destroy();

    handle_ = other.handle_;
    capabilities_ = other.capabilities_;
    formats_ = std::move(other.formats_);
    device_path_ = std::move(other.device_path_);
    current_fmt_ = other.current_fmt_;
    buffers_ = std::move(other.buffers_);
    running_ = other.running_;
    {
        const std::unique_lock lock(other.control_block_->lock);
        other.control_block_->camera_instance = this;
    }

    control_block_ = std::move(other.control_block_);
    capture_thread_ = std::move(other.capture_thread_);

    other.handle_ = -1;
}

V4L2Camera& V4L2Camera::operator=(V4L2Camera&& other) noexcept {
    destroy();

    handle_ = other.handle_;
    capabilities_ = other.capabilities_;
    formats_ = std::move(other.formats_);
    device_path_ = std::move(other.device_path_);
    current_fmt_ = other.current_fmt_;
    buffers_ = std::move(other.buffers_);
    running_ = other.running_;
    {
        const std::unique_lock lock(other.control_block_->lock);
        other.control_block_->camera_instance = this;
    }

    control_block_ = std::move(other.control_block_);
    capture_thread_ = std::move(other.capture_thread_);

    other.handle_ = -1;

    return *this;
}

V4L2Camera::~V4L2Camera() {
    destroy();
}

// NOLINTNEXTLINE(bugprone-exception-escape)
void V4L2Camera::destroy() noexcept {
    v4l2_stop_capture();
    close();
    cleanup_buffers();
}

// NOLINTNEXTLINE(bugprone-exception-escape)
bool V4L2Camera::v4l2_stop_capture() noexcept {
    if (!running_) {
        return false;
    }

    capture_thread_.request_stop();
    capture_thread_.join();

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(handle_, VIDIOC_STREAMOFF, &type) == -1) {
        LOG_ERROR(CAMERA_LOGGER, CAMERA_LOGGER, "Failed to stop capture: {}", errno_to_str(errno));
    }

    running_ = false;
    return true;
}

std::optional<v4l2_format> V4L2Camera::v4l2_get_video_format() const {
    v4l2_format fmt{};

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    const int ret = xioctl(handle_, VIDIOC_G_FMT, &fmt);
    if (ret == -1) {
        LOG_ERROR(CAMERA_LOGGER, "Cannot get current device format {}: {}", device_path_, errno_to_str(errno));
        return std::nullopt;
    }

    return fmt;
}

std::optional<v4l2_capability> V4L2Camera::v4l2_query_capabilities() const {
    v4l2_capability cap{};

    const int ret = xioctl(handle_, VIDIOC_QUERYCAP, &cap);
    if (ret == -1) {
        LOG_ERROR(CAMERA_LOGGER, "Cannot open video device {}: {}", device_path_, errno_to_str(errno));
        return std::nullopt;
    }
    return cap;
}

bool V4L2Camera::v4l2_set_fps(uint16_t fps) {
    v4l2_streamparm stream_param{};

    stream_param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(handle_, VIDIOC_G_PARM, &stream_param) != 0) {
        LOG_ERROR(CAMERA_LOGGER, "Failed to get video capture params {}: {}", device_path_, errno_to_str(errno));
        return false;
    }

    stream_param.parm.capture.capturemode |= V4L2_CAP_TIMEPERFRAME;

    stream_param.parm.capture.timeperframe.numerator = 1000 / fps; // NOLINT(readability-magic-numbers)
    stream_param.parm.capture.timeperframe.denominator = 1000; // NOLINT(readability-magic-numbers)
    if (xioctl(handle_, VIDIOC_S_PARM, &stream_param) != 0) {
        LOG_ERROR(CAMERA_LOGGER, "Failed to set frame rate {}: {}", device_path_, errno_to_str(errno));
        return false;
    }

    auto time_per_frame = stream_param.parm.capture.timeperframe;

    unsigned effective_fps = time_per_frame.denominator * time_per_frame.numerator;
    if (effective_fps != fps) {
        LOG_ERROR(CAMERA_LOGGER,
                  "Effective FPS is different from request: effective={} requested={}",
                  effective_fps,
                  fps);
        return false;
    }

    return true;
}

bool V4L2Camera::open() {
    if (is_open()) {
        LOG_ERROR(CAMERA_LOGGER, "Device is already opened");
        return false;
    }

    LOG_INFO(CAMERA_LOGGER, "Opening video device: {}", device_path_);
    handle_ = ::open(device_path_.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC, 0);
    if (handle_ == -1) {
        LOG_ERROR(CAMERA_LOGGER, "Cannot open video device {}: {}", device_path_, errno_to_str(errno));
        return false;
    }

    LOG_INFO(CAMERA_LOGGER, "Requesting capabilities");
    auto cap = v4l2_query_capabilities();
    if (cap.has_value()) {
        capabilities_ = cap.value();
    } else {
        ::close(handle_);
        handle_ = -1;
        return false;
    }

    auto fmt = v4l2_get_video_format();
    if (fmt.has_value()) {
        current_fmt_ = fmt.value();
    } else {
        ::close(handle_);
        handle_ = -1;
        memset(&capabilities_, 0, sizeof(capabilities_));
        return false;
    }

    enumerate_capture_formats();

    return true;
}

void V4L2Camera::close() {
    if (!is_open()) {
        return;
    }

    LOG_INFO(CAMERA_LOGGER, "Closing device: {}", device_path_);
    ::close(handle_);
    handle_ = -1;
    memset(&capabilities_, 0, sizeof(capabilities_));
    device_path_.clear();
}

bool V4L2Camera::is_streaming_supported() const {
    if (!is_open()) {
        return false;
    }

    return (capabilities_.capabilities & V4L2_CAP_STREAMING) != 0;
}

int V4L2Camera::select_format(unsigned int width, unsigned int height, uint32_t format) {
    struct v4l2_cropcap cropcap{};

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (xioctl(handle_, VIDIOC_CROPCAP, &cropcap) == 0) {
        struct v4l2_crop crop{};
        crop.c = cropcap.defrect;
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        xioctl(handle_, VIDIOC_S_CROP, &crop); // Ignore errors
    } else {
        LOG_ERROR(CAMERA_LOGGER, "Can't query crop capabilities: {}", errno_to_str(errno));
    }

    struct v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = format;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    const int ret = xioctl(handle_, VIDIOC_S_FMT, &fmt);
    if (ret == 0) {
        current_fmt_ = fmt;
    } else {
        LOG_ERROR(CAMERA_LOGGER, "Can't set video format: {}", errno_to_str(errno));
        return ret;
    }

    if ((fmt.fmt.pix.width != width) || (fmt.fmt.pix.height != height) || (fmt.fmt.pix.pixelformat != format)) {
        LOG_ERROR(CAMERA_LOGGER, "Can't fully set video capture format parameters");
        LOG_ERROR(CAMERA_LOGGER, "Width: expected={}\tactual={}", width, fmt.fmt.pix.width);
        LOG_ERROR(CAMERA_LOGGER, "Height: expected={}\tactual={}", height, fmt.fmt.pix.height);
        LOG_ERROR(CAMERA_LOGGER,
                  "PixelFormat: expected={}\tactual={}",
                  v4l2_encoding_to_string(format),
                  v4l2_encoding_to_string(fmt.fmt.pix.pixelformat));
        return -1;
    }

    if (!buffers_.empty()) {
        cleanup_buffers();
    }

    if (init_buffers()) {
        return 0;
    }
    return -1;
}

bool V4L2Camera::is_open() const {
    return handle_ != -1;
}

void V4L2Camera::enumerate_capture_formats() {
    struct v4l2_fmtdesc fmt{};

    int status = 0;
    uint32_t i = 0;

    formats_.clear();

    do {
        memset(&fmt, 0, sizeof(fmt));
        fmt.index = i;
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        status = xioctl(handle_, VIDIOC_ENUM_FMT, &fmt);
        if (status == 0) {
            formats_.push_back(fmt);
            i++;
        }
    } while (status != -1);
}

void V4L2Camera::print_camera_info() const {
    LOG_DEBUG(CAMERA_LOGGER, "Capabilities for {}", device_path_);
    LOG_DEBUG(CAMERA_LOGGER, "Driver: {}", reinterpret_cast<const char*>(capabilities_.driver));
    LOG_DEBUG(CAMERA_LOGGER, "Card: {}", reinterpret_cast<const char*>(capabilities_.card));
    LOG_INFO(CAMERA_LOGGER, "Bus info: {}", reinterpret_cast<const char*>(capabilities_.bus_info));

    unsigned int major = 0;
    unsigned int minor = 0;
    unsigned int patch = 0;
    major = (capabilities_.version >> 16U) & 0xFFU; // NOLINT(readability-magic-numbers)
    minor = (capabilities_.version >> 8U) & 0xFFU; // NOLINT(readability-magic-numbers)
    patch = (capabilities_.version) & 0xFFU; // NOLINT(readability-magic-numbers)
    std::ostringstream version_stream;
    version_stream << major << "." << minor << "." << patch;

    LOG_DEBUG(CAMERA_LOGGER, "Version: {}", version_stream.str());
    auto raw_cap = capabilities_.capabilities;
    bool video_capture = false;
    bool audio_support = false;
    bool rw_support = false;
    bool async_io = false;
    bool streaming_support = false;
    bool metadata_output = false;

#ifdef V4L2_CAP_VIDEO_CAPTURE
    video_capture = (raw_cap & V4L2_CAP_VIDEO_CAPTURE) != 0;
#endif

#ifdef V4L2_CAP_AUDIO
    audio_support = (raw_cap & V4L2_CAP_AUDIO) != 0;
#endif

#ifdef V4L2_CAP_READWRITE
    rw_support = (raw_cap & V4L2_CAP_READWRITE) != 0;
#endif

#ifdef V4L2_CAP_ASYNCIO
    async_io = (raw_cap & V4L2_CAP_ASYNCIO) != 0;
#endif

#ifdef V4L2_CAP_STREAMING
    streaming_support = (raw_cap & V4L2_CAP_STREAMING) != 0;
#endif

#ifdef V4L2_CAP_META_OUTPUT
    metadata_output = (raw_cap & V4L2_CAP_META_OUTPUT) != 0;
#endif

    LOG_DEBUG(CAMERA_LOGGER, "Device capabilities (RAW): {:x}", raw_cap);
    LOG_DEBUG(CAMERA_LOGGER, "Video capture: {}", video_capture);
    LOG_DEBUG(CAMERA_LOGGER, "Audio support: {}", audio_support);
    LOG_DEBUG(CAMERA_LOGGER, "Read/Write API support: {}", rw_support);
    LOG_DEBUG(CAMERA_LOGGER, "AsyncIO API support: {}", async_io);
    LOG_DEBUG(CAMERA_LOGGER, "Streaming API support: {}", streaming_support);
    LOG_DEBUG(CAMERA_LOGGER, "Metadata output device: {}", metadata_output);

    LOG_DEBUG(CAMERA_LOGGER, "Number of supported pixel formats: {}", formats_.size());
    for (const auto& fmt : formats_) {
        LOG_DEBUG(CAMERA_LOGGER,
                  "{}: Format={} Description={} Compressed={} Emulated={}",
                  fmt.index,
                  v4l2_encoding_to_string(fmt.pixelformat),
                  reinterpret_cast<const char*>(fmt.description),
                  static_cast<bool>(fmt.flags & V4L2_FMT_FLAG_COMPRESSED),
                  static_cast<bool>(fmt.flags & V4L2_FMT_FLAG_EMULATED));
    }


    auto current_format_opt = v4l2_get_video_format();
    if (current_format_opt.has_value()) {
        struct v4l2_format current_format = current_format_opt.value();
        LOG_DEBUG(CAMERA_LOGGER,
                  "Current format: width={} height={} fmt={}",
                  current_format.fmt.pix.width,
                  current_format.fmt.pix.height,
                  v4l2_encoding_to_string(current_format.fmt.pix.pixelformat));
    } else {
        LOG_ERROR(CAMERA_LOGGER, "Failed to query current format: {}", errno_to_str(errno));
    }
}

void V4L2Camera::start_capture_thread() {
    auto ctrl_block = control_block_;
    const int handle = this->handle_;

    capture_thread_ = std::jthread(
            [handle, ctrl_block](std::stop_token token) { capture_thread_func(handle, std::move(token), ctrl_block); });
}

std::optional<V4L2Camera>
V4L2Camera::create_camera(uint16_t width, uint16_t height, ImageEncoding encoding, uint16_t fps) {
    const uint32_t v4l2_encoding = convert_image_encoding_to_v4l2(encoding);
    if (v4l2_encoding == 0) {
        LOG_ERROR(CAMERA_LOGGER, "Unsupported encoding: {}", to_string(encoding));
        return std::nullopt;
    }

    V4L2Camera camera(DEFAULT_VIDEO);

    if (!camera.open()) {
        LOG_ERROR(CAMERA_LOGGER, "Failed to open camera device");
        return std::nullopt;
    }

    if (!camera.is_streaming_supported()) {
        LOG_ERROR(CAMERA_LOGGER, "Streaming is not supported for {}", DEFAULT_VIDEO);
        return std::nullopt;
    }

    if (std::ranges::find(camera.formats_, v4l2_encoding, &v4l2_fmtdesc::pixelformat) == camera.formats_.end()) {
        LOG_ERROR(CAMERA_LOGGER, "Image encoding {} is not supported", to_string(encoding));
        return std::nullopt;
    }

    if (camera.select_format(width, height, v4l2_encoding) != 0) {
        LOG_ERROR(CAMERA_LOGGER, "Failed to select format for {}", DEFAULT_VIDEO);
        return std::nullopt;
    }

    if (!camera.v4l2_set_fps(fps)) {
        return std::nullopt;
    }

    camera.print_camera_info();

    return camera;
}

bool V4L2Camera::start_capture() {
    if (running_) {
        return false;
    }

    for (size_t i = 0; i < buffers_.size(); i++) {
        v4l2_buffer v4l_buf{};
        memset(&v4l_buf, 0, sizeof(v4l_buf));

        v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l_buf.memory = V4L2_MEMORY_MMAP;
        v4l_buf.index = i;

        if (-1 == xioctl(handle_, VIDIOC_QBUF, &v4l_buf)) {
            LOG_ERROR(CAMERA_LOGGER, "Failed to enqueue buffer={}", i);
        }
    }

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (xioctl(handle_, VIDIOC_STREAMON, &type) == -1) {
        LOG_ERROR(CAMERA_LOGGER, "Failed to start capture: {}", errno_to_str(errno));
        return false;
    }

    LOG_INFO(CAMERA_LOGGER, "Capture started");
    start_capture_thread();
    running_ = true;

    return true;
}

bool V4L2Camera::stop_capture() {
    return v4l2_stop_capture();
}

bool V4L2Camera::is_capturing() const {
    return running_;
}

ImageFormat V4L2Camera::current_format() const {
    ImageFormat fmt{};
    if (auto encoding = v4l2_to_image_encoding(current_fmt_.fmt.pix.pixelformat); encoding.has_value()) {
        fmt.encoding = encoding.value();
    } else {
        LOG_ERROR(CAMERA_LOGGER,
                  "Unsupported V4L2 encoding: {}",
                  v4l2_encoding_to_string(current_fmt_.fmt.pix.pixelformat));
    }
    fmt.width = current_fmt_.fmt.pix.width;
    fmt.height = current_fmt_.fmt.pix.height;
    return fmt;
}

void V4L2Camera::set_frame_handler(std::function<void(const CameraFrame&)> handler) {
    frame_handler_ = std::move(handler);
}

// NOLINTNEXTLINE(performance-unnecessary-value-param)
void V4L2Camera::capture_thread_func(int handle, std::stop_token token, std::shared_ptr<ControlBlock> control_block) {
    const int epoll_fd = epoll_create1(EPOLL_CLOEXEC);
    epoll_event ev{};
    ev.events = EPOLLIN;
    ev.data.fd = handle;

    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, handle, &ev) == -1) {
        LOG_ERROR(CAMERA_LOGGER, "Failed to add fd to epoll: {}", errno_to_str(errno));
        ::close(epoll_fd);
        return;
    }

    while (!token.stop_requested()) {
        epoll_event wait_ev{};
        const int nfds = epoll_wait(epoll_fd, &wait_ev, 1, WAIT_TIMEOUT);
        if (nfds == -1) {
            LOG_ERROR(CAMERA_LOGGER, "epoll_wait failed: {}", errno_to_str(errno));
            break;
        }

        if (nfds == 0) {
            continue;
        }

        v4l2_buffer buf{};
        memset(&buf, 0, sizeof(buf));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (xioctl(handle, VIDIOC_DQBUF, &buf) == -1) {
            LOG_ERROR(CAMERA_LOGGER, "Failed to dequeue buffer: {}", errno_to_str(errno));
            continue;
        }

        {
            const std::unique_lock lock(control_block->lock);
            if (control_block->camera_instance == nullptr) {
                break;
            }

            control_block->camera_instance->read_frame(buf);
        }

        if (xioctl(handle, VIDIOC_QBUF, &buf) == -1) {
            LOG_ERROR(CAMERA_LOGGER, "Failed to reenqueue the buffer: {}", errno_to_str(errno));
        }
    }

    ::close(epoll_fd);
}

bool V4L2Camera::init_buffers() {
    struct v4l2_requestbuffers req{};

    req.count = BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(handle_, VIDIOC_REQBUFS, &req) == -1) {
        if (errno == EINVAL) {
            LOG_ERROR(CAMERA_LOGGER, "Device doesn't support memory mapping");
        } else {
            LOG_ERROR(CAMERA_LOGGER, "Failed to request memory mapping buffers_: {}", errno_to_str(errno));
        }
        return false;
    }

    if (req.count < 2) {
        std::cerr << "Not enough memory\n";
        return false;
    }

    LOG_INFO(CAMERA_LOGGER, "Buffers count: {}", req.count);
    buffers_.reserve(req.count);

    for (unsigned int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf{};

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(handle_, VIDIOC_QUERYBUF, &buf) == -1) {
            LOG_ERROR(CAMERA_LOGGER, "Query buffer idx={} failed: {}", i, errno_to_str(errno));
            cleanup_buffers();
            return false;
        }

        void* addr = mmap(nullptr,
                          buf.length,
                          PROT_READ | PROT_WRITE /* required */,
                          MAP_SHARED /* recommended */,
                          handle_,
                          buf.m.offset);
        buffers_.push_back({addr, buf.length});
    }

    return true;
}

void V4L2Camera::cleanup_buffers() {
    for (const auto& buf : buffers_) {
        if (buf.addr != nullptr) {
            munmap(buf.addr, buf.length);
        }
    }
    buffers_.clear();
}

void V4L2Camera::read_frame(const v4l2_buffer& buffer) {
    if (buffer.index >= buffers_.size()) {
        LOG_ERROR(CAMERA_LOGGER, "Buffer index {} is out of range", buffer.index);
        return;
    }

    LOG_INFO(CAMERA_LOGGER,
             "New frame idx={} length={} used={}",
             buffer.index,
             buffers_[buffer.index].length,
             buffer.bytesused);

    if (frame_handler_) {
        const CameraFrame frame{.format = current_format(),
                                .timestamp = from_timeval(buffer.timestamp),
                                .buffer = {reinterpret_cast<uint8_t*>(buffers_[buffer.index].addr), buffer.bytesused}};
        frame_handler_(frame);
    }
}

uint32_t V4L2Camera::convert_image_encoding_to_v4l2(ImageEncoding encoding) {
    uint32_t result = 0;

    const auto* pos = std::ranges::find(kImageEncodingMappingTable, encoding, &EncodingMappingEntry::internal);
    if (pos != kImageEncodingMappingTable.end()) {
        result = pos->v4l2;
    }

    return result;
}

std::optional<ImageEncoding> V4L2Camera::v4l2_to_image_encoding(uint32_t encoding) {
    const auto* pos = std::ranges::find(kImageEncodingMappingTable, encoding, &EncodingMappingEntry::v4l2);
    if (pos != kImageEncodingMappingTable.end()) {
        return pos->internal;
    }

    return std::nullopt;
}

std::string V4L2Camera::v4l2_encoding_to_string(uint32_t encoding) {
    constexpr size_t kPixelFormatSize = 4;
    std::string decoded(kPixelFormatSize, ' ');

    for (unsigned int j = 0; j < kPixelFormatSize; j++) {
        // NOLINTNEXTLINE(readability-magic-numbers)
        decoded[j] = static_cast<char>((encoding >> (j * 8U)) & 0xFFU);
    }
    return decoded;
}

} // namespace rpicar::camera