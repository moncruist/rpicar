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
#ifndef CAMERA_TRANSCODER_H
#define CAMERA_TRANSCODER_H

#include "camera/camera.h"

#include <list>
#include <optional>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
};

namespace rpicar::camera {

struct DecodedFrame {
    uint16_t width;
    uint16_t height;
    std::vector<uint8_t> data;
};

class Transcoder {
public:
    explicit Transcoder(ImageFormat from, ImageEncoding to);
    Transcoder(const Transcoder&) = delete;
    Transcoder(Transcoder&& other) noexcept;
    ~Transcoder();

    Transcoder& operator=(const Transcoder&) = delete;
    Transcoder& operator=(Transcoder&& other) noexcept;

    std::list<DecodedFrame> transcode(const CameraFrame& frame);

private:
    static AVCodecID get_codec_id(ImageEncoding encoding);
    static AVPixelFormat get_pixel_format(ImageEncoding encoding);
    static DecodedFrame passthrough_frame(const CameraFrame& frame);

    std::optional<DecodedFrame> encode_frame(const AVFrame* frame);
    std::optional<DecodedFrame> change_pixel_format(const AVFrame* frame);
    std::optional<DecodedFrame> change_pixel_format(const CameraFrame& frame);

    ImageFormat from_;
    ImageEncoding to_;

    bool transcoding_required_{false};
    const AVCodec* decoder_{nullptr};
    AVCodecContext* decoding_context_{nullptr};
    AVPixelFormat from_pix_fmt_{AV_PIX_FMT_NONE};
    AVPacket* decoding_packet_{nullptr};
    AVCodecParserContext* decoding_parser_{nullptr};

    const AVCodec* encoder_{nullptr};
    AVCodecContext* encoding_context_{nullptr};
    AVPixelFormat to_pix_fmt_{AV_PIX_FMT_NONE};

    SwsContext* sws_context_{nullptr};
};

} // namespace rpicar::camera

#endif // CAMERA_TRANSCODER_H