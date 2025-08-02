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

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "compat.h"

#define LOG_DEBUG(logger, msg_fmt, ...)                                                   \
    do {                                                                                  \
        const std::string formatted_msg{cmp::format(msg_fmt __VA_OPT__(, ) __VA_ARGS__)}; \
        _Pragma("GCC diagnostic push");                                                   \
        _Pragma("GCC diagnostic ignored \"-Wformat-security\"");                          \
        RCLCPP_DEBUG(rclcpp::get_logger(logger), formatted_msg.c_str());                  \
        _Pragma("GCC diagnostic pop");                                                    \
    } while (0)

#define LOG_INFO(logger, msg_fmt, ...)                                                    \
    do {                                                                                  \
        const std::string formatted_msg{cmp::format(msg_fmt __VA_OPT__(, ) __VA_ARGS__)}; \
        _Pragma("GCC diagnostic push");                                                   \
        _Pragma("GCC diagnostic ignored \"-Wformat-security\"");                          \
        RCLCPP_INFO(rclcpp::get_logger(logger), formatted_msg.c_str());                   \
        _Pragma("GCC diagnostic pop");                                                    \
    } while (0)

#define LOG_WARN(logger, msg_fmt, ...)                                                    \
    do {                                                                                  \
        const std::string formatted_msg{cmp::format(msg_fmt __VA_OPT__(, ) __VA_ARGS__)}; \
        _Pragma("GCC diagnostic push");                                                   \
        _Pragma("GCC diagnostic ignored \"-Wformat-security\"");                          \
        RCLCPP_WARN(rclcpp::get_logger(logger), formatted_msg.c_str());                   \
        _Pragma("GCC diagnostic pop");                                                    \
    } while (0)

#define LOG_ERROR(logger, msg_fmt, ...)                                                   \
    do {                                                                                  \
        const std::string formatted_msg{cmp::format(msg_fmt __VA_OPT__(, ) __VA_ARGS__)}; \
        _Pragma("GCC diagnostic push");                                                   \
        _Pragma("GCC diagnostic ignored \"-Wformat-security\"");                          \
        RCLCPP_ERROR(rclcpp::get_logger(logger), formatted_msg.c_str());                  \
        _Pragma("GCC diagnostic pop");                                                    \
    } while (0)

#define LOG_FATAL(logger, msg_fmt, ...)                                                   \
    do {                                                                                  \
        const std::string formatted_msg{cmp::format(msg_fmt __VA_OPT__(, ) __VA_ARGS__)}; \
        _Pragma("GCC diagnostic push");                                                   \
        _Pragma("GCC diagnostic ignored \"-Wformat-security\"");                          \
        RCLCPP_FATAL(rclcpp::get_logger(logger), formatted_msg.c_str());                  \
        _Pragma("GCC diagnostic pop");                                                    \
    } while (0)

