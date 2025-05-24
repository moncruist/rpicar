// rpivision
// Copyright (C) 2021 Konstantin Zhukov
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

#include "camera/rpi_camera.h"

#include "utils/logging.h"

#include <initializer_list>
#include <libcamera/control_ids.h>
#include <ranges>
#include <sys/mman.h>
#include <utility>

using libcamera::CameraConfiguration;
using libcamera::FrameBuffer;
using libcamera::FrameBufferAllocator;
using libcamera::Request;
using libcamera::Stream;
using libcamera::StreamConfiguration;
using libcamera::StreamRole;
using libcamera::controls::FrameDurationLimits;

namespace rpicar::camera {

RpiCamera::RpiCamera() noexcept : manager_(std::make_unique<libcamera::CameraManager>()) {}

RpiCamera::~RpiCamera() {
    stop_capture();
}

RpiCamera::RpiCamera(RpiCamera&& other) noexcept
  : manager_(std::move(other.manager_)),
    camera_(std::move(other.camera_)),
    configuration_(std::move(other.configuration_)),
    allocator_(std::move(other.allocator_)),
    mapped_buffers_(std::move(other.mapped_buffers_)),
    frame_buffers_(std::move(other.frame_buffers_)),
    camera_acquired_(std::exchange(other.camera_acquired_, false)),
    camera_started_(std::exchange(other.camera_started_, false)) {}

RpiCamera& RpiCamera::operator=(RpiCamera&& other) noexcept {
    manager_ = std::move(other.manager_);
    camera_ = std::move(other.camera_);
    configuration_ = std::move(other.configuration_);
    allocator_ = std::move(other.allocator_);
    mapped_buffers_ = std::move(other.mapped_buffers_);
    frame_buffers_ = std::move(other.frame_buffers_);
    camera_acquired_ = std::exchange(other.camera_acquired_, false);
    camera_started_ = std::exchange(other.camera_started_, false);

    return *this;
}

std::optional<RpiCamera>
RpiCamera::create_camera(uint16_t width, uint16_t height, ImageEncoding encoding, uint16_t fps) {
    RpiCamera camera{};

    if (camera.configure_camera(width, height, encoding, fps)) {
        return camera;
    }

    return std::nullopt;
}

bool RpiCamera::start_capture() {
    camera_->start();
    camera_started_ = true;

    camera_->requestCompleted.connect(this, &RpiCamera::requested_completed_handler);

    for (std::unique_ptr<Request>& request : requests_) {
        if (camera_->queueRequest(request.get()) < 0) {
            stop_capture();
            return false;
        }
    }

    return true;
}


bool RpiCamera::stop_capture() {
    if (camera_) {
        camera_->requestCompleted.disconnect(this, &RpiCamera::requested_completed_handler);

        if (camera_started_) {
            camera_->stop();
            camera_started_ = false;
        }

        requests_.clear();

        deallocate_buffers();

        if (configuration_) {
            configuration_.reset();
        }

        if (camera_acquired_) {
            camera_->release();
            camera_acquired_ = false;
        }

        camera_.reset();
    }

    return true;
}

bool RpiCamera::configure_camera(uint16_t width, uint16_t height, ImageEncoding encoding, uint16_t fps) {
    int status = manager_->start();
    if (status) {
        LOG_ERROR(CAMERA_LOGGER, "Camera manager failed to start: {}", -status);
        return false;
    }

    const auto cameras = manager_->cameras();
    if (cameras.empty()) {
        LOG_ERROR(CAMERA_LOGGER, "No cameras were found");
        return false;
    }

    const auto rpi_camera_pos = std::ranges::find_if(cameras, [](std::shared_ptr<libcamera::Camera> camera) {
        return camera->id().find(RPI_CAMERA_ID) != std::string::npos;
    });

    if (rpi_camera_pos == cameras.end()) {
        LOG_ERROR(CAMERA_LOGGER, "Could not find RPi camera_");
        return false;
    }

    camera_ = *rpi_camera_pos;

    status = camera_->acquire();
    if (status < 0) {
        LOG_ERROR(CAMERA_LOGGER, "Failed to acquire camera: {}", -status);
        camera_.reset();
        return false;
    }

    camera_acquired_ = true;

    LOG_INFO(CAMERA_LOGGER, "Configuring video");
    configuration_ = camera_->generateConfiguration({StreamRole::VideoRecording});
    if (!configuration_) {
        LOG_ERROR(CAMERA_LOGGER, "Could not generate camare configuration");
        camera_->release();
        camera_.reset();
        return false;
    }

    configuration_->at(0).size.width = width;
    configuration_->at(0).size.height = height;

    const auto conf_status = configuration_->validate();
    if (conf_status == CameraConfiguration::Invalid) {
        LOG_ERROR(CAMERA_LOGGER, "Camera configuration is invalid");
        camera_->release();
        camera_.reset();
        configuration_.reset();
        return false;
    }

    camera_->configure(configuration_.get());

    auto fd_ctrl{camera_->controls().find(&FrameDurationLimits)};


    allocator_ = std::make_unique<FrameBufferAllocator>(camera_);

    if (!allocate_buffers()) {
        camera_->release();
        allocator_.reset();
        camera_.reset();
        configuration_.reset();
        return false;
    }

    if (!make_requests()) {
        deallocate_buffers();
        camera_->release();
        allocator_.reset();
        camera_.reset();
        configuration_.reset();
        return false;
    }
}

bool RpiCamera::allocate_buffers() {
    bool success{true};
    for (StreamConfiguration& config : *configuration_) {
        Stream* stream = config.stream();

        if (allocator_->allocate(stream) < 0) {
            LOG_ERROR(CAMERA_LOGGER, "Failed to allocate capture buffers");
            success = false;
            break;
        }

        for (const std::unique_ptr<FrameBuffer>& buffer : allocator_->buffers(stream)) {
            // "Single plane" buffers appear as multi-plane here, but we can spot them because then
            // planes all share the same fd. We accumulate them so as to mmap the buffer only once.
            size_t buffer_size = 0;
            const size_t planes_size = buffer->planes().size();
            for (size_t i = 0; i < planes_size; i++) {
                const FrameBuffer::Plane& plane = buffer->planes()[i];
                buffer_size += plane.length;
                if ((i == buffer->planes().size() - 1) || (plane.fd.get() != buffer->planes()[i + 1].fd.get())) {
                    void* memory = mmap(NULL, buffer_size, PROT_READ | PROT_WRITE, MAP_SHARED, plane.fd.get(), 0);
                    mapped_buffers_[buffer.get()].emplace_back(static_cast<uint8_t*>(memory), buffer_size);
                    buffer_size = 0;
                }
            }
            frame_buffers_[stream].push(buffer.get());
        }
    }

    LOG_INFO(
            CAMERA_LOGGER, "Allocated buffers {} for {} configurations", frame_buffers_.size(), configuration_->size());

    if (!success) {
        deallocate_buffers();
    }

    return success;
}

void RpiCamera::deallocate_buffers() {
    for (const auto& [buffer, mapped_regions] : mapped_buffers_) {
        for (const auto& region : mapped_regions) {
            munmap(region.data(), region.size());
        }
    }

    mapped_buffers_.clear();

    for (const auto& [buffer, _] : frame_buffers_) {
        allocator_->free(buffer);
    }
    frame_buffers_.clear();
}

bool RpiCamera::make_requests(uint16_t fps) {
    auto free_buffers(frame_buffers_);
    while (true) {
        for (StreamConfiguration& config : *configuration_) {
            Stream* stream = config.stream();
            if (stream == configuration_->at(0).stream()) {
                if (free_buffers[stream].empty()) {
                    LOG_DEBUG(CAMERA_LOGGER, "Requests created");
                    return true;
                }
                std::unique_ptr<Request> request = camera_->createRequest();
                if (!request) {
                    LOG_ERROR(CAMERA_LOGGER, "failed to make request");
                    return false;
                }

                std::int64_t value_pair[2] = {lowerMicroSeconds, higherMicroSeconds};
                request->controls().set(libcamera::controls::FrameDurationLimits,
                                        libcamera::Span<const std::int64_t, 2>(value_pair));

                requests_.push_back(std::move(request));
            } else if (free_buffers[stream].empty()) {
                LOG_ERROR(CAMERA_LOGGER, "concurrent streams need matching numbers of buffers");
                return false;
            }

            FrameBuffer* buffer = free_buffers[stream].front();
            free_buffers[stream].pop();
            if (requests_.back()->addBuffer(stream, buffer) < 0) {
                LOG_ERROR(CAMERA_LOGGER, "failed to add buffer to request");
                return false;
            }
        }
    }
}

void RpiCamera::requested_completed_handler(libcamera::Request* request) {
    if (request->status() == Request::RequestCancelled) {
        LOG_INFO(CAMERA_LOGGER, "Request cancelled");
        return;
    }

    uint64_t timestamp{};
    if (request->metadata().contains(libcamera::controls::SensorTimestamp.id())) {
        timestamp = request->metadata().get(libcamera::controls::SensorTimestamp).value();
    } else {
        timestamp = request->buffers().begin()->second->metadata().timestamp;
    }


    LOG_INFO(CAMERA_LOGGER,
             "Status: {}\tBuf size: {}\tTimestamp: {}",
             static_cast<int>(request->status()),
             request->buffers().begin()->second->planes().begin()->length,
             timestamp);

    request->reuse(Request::ReuseBuffers);
    camera_->queueRequest(request);
}

} // namespace rpicar::camera
