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

#include "camera/transcoder.h"

#include "camera/camera.h"
#include "utils/logging.h"

#include <array>
#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <list>
#include <optional>
#include <utility>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/codec.h>
#include <libavcodec/codec_id.h>
#include <libavcodec/packet.h>
#include <libavutil/avutil.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
#include <libavutil/imgutils.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
};

namespace rpicar::camera {

Transcoder::Transcoder(ImageFormat from, ImageEncoding to) : from_(from), to_(to) {
    const AVCodecID from_codec = get_codec_id(from.encoding);
    const AVCodecID to_codec = get_codec_id(to);

    transcoding_required_ = from_codec != to_codec;
    if (transcoding_required_) {
        if (from_codec != AV_CODEC_ID_NONE) {
            decoder_ = avcodec_find_decoder(from_codec);
            assert(decoder_ != nullptr);

            decoding_context_ = avcodec_alloc_context3(decoder_);
            assert(decoding_context_ != nullptr);

            decoding_context_->width = from.width;
            decoding_context_->height = from.height;

            if (avcodec_open2(decoding_context_, decoder_, nullptr) < 0) {
                LOG_ERROR(CAMERA_LOGGER, "Failed to open decoder");
            }

            decoding_packet_ = av_packet_alloc();
            assert(decoding_packet_ != nullptr);

            decoding_parser_ = av_parser_init(from_codec);
        }

        if (to_codec != AV_CODEC_ID_NONE) {
            encoder_ = avcodec_find_encoder(to_codec);
            assert(encoder_ != nullptr);

            encoding_context_ = avcodec_alloc_context3(encoder_);
            assert(encoding_context_ != nullptr);

            encoding_context_->width = from.width;
            encoding_context_->height = from.height;
            encoding_context_->pix_fmt = get_pixel_format(to);

            if (avcodec_open2(encoding_context_, encoder_, nullptr) < 0) {
                LOG_ERROR(CAMERA_LOGGER, "Failed to open encoder");
            }
        }
    }

    from_pix_fmt_ = get_pixel_format(from.encoding);
    to_pix_fmt_ = get_pixel_format(to);
    if (from_pix_fmt_ != to_pix_fmt_) {
        sws_context_ = sws_getContext(from.width,
                                      from.height,
                                      from_pix_fmt_,
                                      from.width,
                                      from.height,
                                      to_pix_fmt_,
                                      SWS_FAST_BILINEAR,
                                      nullptr,
                                      nullptr,
                                      nullptr);
        assert(sws_context_ != nullptr);
    }
}

Transcoder::Transcoder(Transcoder&& other) noexcept {
    from_ = other.from_;
    to_ = other.to_;
    transcoding_required_ = other.transcoding_required_;
    std::swap(decoder_, other.decoder_);
    std::swap(decoding_context_, other.decoding_context_);
    from_pix_fmt_ = other.from_pix_fmt_;
    std::swap(decoding_packet_, other.decoding_packet_);
    std::swap(decoding_parser_, other.decoding_parser_);
    std::swap(encoder_, other.encoder_);
    std::swap(encoding_context_, other.encoding_context_);
    to_pix_fmt_ = other.to_pix_fmt_;
    std::swap(sws_context_, other.sws_context_);
}

Transcoder::~Transcoder() {
    if (decoding_context_ != nullptr) {
        avcodec_free_context(&decoding_context_);
    }

    if (decoding_packet_ != nullptr) {
        av_packet_free(&decoding_packet_);
    }

    if (decoding_parser_ != nullptr) {
        av_parser_close(decoding_parser_);
    }

    if (encoding_context_ != nullptr) {
        avcodec_free_context(&encoding_context_);
    }

    if (sws_context_ != nullptr) {
        sws_freeContext(sws_context_);
    }
}

Transcoder& Transcoder::operator=(Transcoder&& other) noexcept {
    from_ = other.from_;
    to_ = other.to_;
    transcoding_required_ = other.transcoding_required_;
    std::swap(decoder_, other.decoder_);

    if (decoding_context_ != nullptr) {
        avcodec_free_context(&decoding_context_);
        decoding_context_ = nullptr;
    }
    std::swap(decoding_context_, other.decoding_context_);

    from_pix_fmt_ = other.from_pix_fmt_;

    if (decoding_packet_ != nullptr) {
        av_packet_free(&decoding_packet_);
        decoding_packet_ = nullptr;
    }
    std::swap(decoding_packet_, other.decoding_packet_);

    if (decoding_parser_ != nullptr) {
        av_parser_close(decoding_parser_);
        decoding_parser_ = nullptr;
    }
    std::swap(decoding_parser_, other.decoding_parser_);

    std::swap(encoder_, other.encoder_);

    if (encoding_context_ != nullptr) {
        avcodec_free_context(&encoding_context_);
        encoding_context_ = nullptr;
    }
    std::swap(encoding_context_, other.encoding_context_);

    to_pix_fmt_ = other.to_pix_fmt_;

    if (sws_context_ != nullptr) {
        sws_freeContext(sws_context_);
        sws_context_ = nullptr;
    }
    std::swap(sws_context_, other.sws_context_);

    return *this;
}

std::list<DecodedFrame> Transcoder::transcode(const CameraFrame& frame) {
    std::list<DecodedFrame> result;
    if (from_.encoding == to_) {
        if ((from_.width != frame.format.width) || (from_.height != frame.format.height)) {
            LOG_ERROR(CAMERA_LOGGER,
                      "Incompatible frame size: expected WxH={}x{}, current WxH={}x{}",
                      from_.width,
                      from_.height,
                      frame.format.width,
                      frame.format.height);
            return result;
        }

        DecodedFrame decoded = passthrough_frame(frame);
        result.push_back(std::move(decoded));
    } else {
        AVFrame* av_frame = av_frame_alloc();
        if (decoder_ != nullptr) {
            // Requires decoding
            size_t in_len = frame.buffer.size_bytes();
            size_t offset = 0;

            while (in_len > 0) {
                if (decoding_parser_ != nullptr) {
                    const int bytes_parsed = av_parser_parse2(decoding_parser_,
                                                              decoding_context_,
                                                              &decoding_packet_->data,
                                                              &decoding_packet_->size,
                                                              &frame.buffer[offset],
                                                              static_cast<int>(in_len),
                                                              AV_NOPTS_VALUE,
                                                              AV_NOPTS_VALUE,
                                                              0);
                    if (bytes_parsed < 0) {
                        LOG_ERROR(CAMERA_LOGGER, "Decoding error: {}", AVUNERROR(bytes_parsed));
                        av_frame_free(&av_frame);
                        return result;
                    }
                    if (bytes_parsed == 0) {
                        break;
                    }

                    const std::size_t bytes_parsed_unsigned = bytes_parsed;

                    offset += bytes_parsed_unsigned;
                    in_len = (bytes_parsed_unsigned > in_len) ? 0U : (in_len - bytes_parsed_unsigned);
                } else {
                    decoding_packet_->data = frame.buffer.data();
                    decoding_packet_->size = static_cast<int>(frame.buffer.size_bytes());
                    in_len = 0;
                }

                if (decoding_packet_->size > 0) {
                    int ret = avcodec_send_packet(decoding_context_, decoding_packet_);
                    if (ret < 0) {
                        LOG_ERROR(CAMERA_LOGGER, "avcodec_send_packet failed: {}", AVUNERROR(ret));
                        av_frame_free(&av_frame);
                        return result;
                    }

                    while (ret >= 0) {
                        ret = avcodec_receive_frame(decoding_context_, av_frame);
                        if (ret == AVERROR_EOF || ret == AVERROR(EAGAIN)) {
                            break;
                        }
                        if (ret < 0) {
                            LOG_ERROR(CAMERA_LOGGER, "avcodec_receive_frame failed: {}", AVUNERROR(ret));
                            av_frame_free(&av_frame);
                            return result;
                        }

                        LOG_INFO(CAMERA_LOGGER, "Frame size {}x{}", av_frame->width, av_frame->height);

                        std::optional<DecodedFrame> enc_frame;
                        if (encoder_ != nullptr) {
                            enc_frame = encode_frame(av_frame);
                        } else {
                            enc_frame = change_pixel_format(av_frame);
                        }

                        if (enc_frame.has_value()) {
                            result.push_back(std::move(enc_frame.value()));
                        }
                    }
                }
            }
        } else {
            std::optional<DecodedFrame> enc_frame;
            if (encoder_ != nullptr) {
                enc_frame = encode_frame(av_frame);
            } else {
                enc_frame = change_pixel_format(frame);
            }

            if (enc_frame.has_value()) {
                result.push_back(std::move(enc_frame.value()));
            }
        }

        av_frame_free(&av_frame);
    }
    return result;
}

AVCodecID Transcoder::get_codec_id(ImageEncoding encoding) {
    switch (encoding) {
        case ImageEncoding::MJPEG:
            return AV_CODEC_ID_MJPEG;

        default:
            return AV_CODEC_ID_NONE;
    }
}

AVPixelFormat Transcoder::get_pixel_format(ImageEncoding encoding) {
    switch (encoding) {
        case ImageEncoding::RGB24:
            return AV_PIX_FMT_RGB24;

        case ImageEncoding::BGR24:
            return AV_PIX_FMT_BGR24;

        case ImageEncoding::YUV420:
            return AV_PIX_FMT_YUV420P;

        case ImageEncoding::YUV422:
            return AV_PIX_FMT_YUYV422;

        case ImageEncoding::MJPEG:
            return AV_PIX_FMT_YUV420P;

        default:
            return AV_PIX_FMT_NONE;
    }
}

DecodedFrame Transcoder::passthrough_frame(const CameraFrame& frame) {
    DecodedFrame decoded{.width = frame.format.width, .height = frame.format.height, .data{}};
    decoded.data.assign(frame.buffer.begin(), frame.buffer.end());
    return decoded;
}

std::optional<DecodedFrame> Transcoder::encode_frame(const AVFrame* frame) {
    int ret = avcodec_send_frame(encoding_context_, frame);
    if (ret < 0) {
        LOG_ERROR(CAMERA_LOGGER, "Failed to encode frame: {}", AVUNERROR(ret));
        return std::nullopt;
    }

    AVPacket* enc_packet = av_packet_alloc();
    ret = avcodec_receive_packet(encoding_context_, enc_packet);
    if (ret < 0) {
        LOG_ERROR(CAMERA_LOGGER, "Failed to get encoded packet: {}", AVUNERROR(ret));
        av_packet_free(&enc_packet);
        return std::nullopt;
    }

    DecodedFrame decoded{.width = static_cast<uint16_t>(frame->width),
                         .height = static_cast<uint16_t>(frame->height),
                         .data = std::vector<uint8_t>(enc_packet->data, enc_packet->data + enc_packet->size)};
    av_packet_free(&enc_packet);

    return decoded;
}

std::optional<DecodedFrame> Transcoder::change_pixel_format(const AVFrame* frame) {
    AVFrame* enc_frame = av_frame_alloc();
    // NOLINTBEGIN(hicpp-no-array-decay)
    const int ret = sws_scale(
            sws_context_, frame->data, frame->linesize, 0, from_.height, enc_frame->data, enc_frame->linesize);
    // NOLINTEND(hicpp-no-array-decay)
    if (ret < 0) {
        LOG_ERROR(CAMERA_LOGGER, "AVFrame sws_scale failed: {}", AVUNERROR(ret));
        av_frame_free(&enc_frame);
        return std::nullopt;
    }

    DecodedFrame decoded{.width = static_cast<uint16_t>(frame->width),
                         .height = static_cast<uint16_t>(frame->height),
                         .data = std::vector<uint8_t>(enc_frame->data[0], enc_frame->data[0] + enc_frame->linesize[0])};
    av_frame_free(&enc_frame);

    return decoded;
}

std::optional<DecodedFrame> Transcoder::change_pixel_format(const CameraFrame& frame) {
    AVFrame* enc_frame = av_frame_alloc();
    const std::array<const uint8_t*, 1U> planes{frame.buffer.data()};
    const std::array<int, 1U> line_sizes{static_cast<int>(frame.buffer.size_bytes() / frame.format.height)};


    const int alloc_ret = av_image_alloc(
            &enc_frame->data[0], &enc_frame->linesize[0], frame.format.width, frame.format.height, to_pix_fmt_, 1);
    if (alloc_ret < 0) {
        LOG_ERROR(CAMERA_LOGGER, "CameraFrame av_image_alloc failed: {}", AVUNERROR(alloc_ret));
        av_frame_free(&enc_frame);
        return std::nullopt;
    }

    const int ret = sws_scale(sws_context_,
                              planes.data(),
                              line_sizes.data(),
                              0,
                              from_.height,
                              &enc_frame->data[0],
                              &enc_frame->linesize[0]);
    if (ret < 0) {
        LOG_ERROR(CAMERA_LOGGER, "CameraFrame sws_scale failed: {}", AVUNERROR(ret));
        av_freep(reinterpret_cast<void*>(enc_frame->data));
        av_frame_free(&enc_frame);
        return std::nullopt;
    }

    DecodedFrame decoded{.width = frame.format.width,
                         .height = frame.format.height,
                         .data = std::vector<uint8_t>(enc_frame->data[0], enc_frame->data[0] + enc_frame->linesize[0])};
    av_freep(reinterpret_cast<void*>(enc_frame->data));
    av_frame_free(&enc_frame);

    return decoded;
}

} // namespace rpicar::camera