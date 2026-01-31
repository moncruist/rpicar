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
#ifndef UTILS_ASSERT_H
#define UTILS_ASSERT_H

#include <stacktrace>

#define BOOST_ENABLE_ASSERT_HANDLER
#include "utils/logging.h"

#include <boost/assert.hpp>

namespace boost {

inline void
assertion_failed_msg(const char* expr, const char* msg, const char* function, const char* file, const long line) {
    LOG_FATAL("assert",
              "Expression '{0}' is false in function '{1}' ({2}:{3}): {4}.\nBacktrace:\n{5}",
              expr,
              function,
              file,
              line,
              (msg ? msg : "<...>"),
              std::stacktrace::current());

    std::abort();
}

inline void assertion_failed(const char* expr, const char* function, const char* file, const long line) {
    ::boost::assertion_failed_msg(expr, nullptr, function, file, line);
}
} // namespace boost

#ifdef DEBUG_ASSERTIONS
#define ASSERT_DBG(expr)          BOOST_ASSERT(expr)
#define ASSERT_DBG_MSG(expr, msg) BOOST_ASSERT_MSG(expr, msg)

#else
#define ASSERT_DBG(expr)          ((void) 0)
#define ASSERT_DBG_MSG(expr, msg) ((void) 0)

#endif

#define ASSERT_PRD(expr)          BOOST_ASSERT(expr)
#define ASSERT_PRD_MSG(expr, msg) BOOST_ASSERT_MSG(expr, msg)

#endif // UTILS_ASSERT_H
