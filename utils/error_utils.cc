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

#include "error_utils.h"

#include <array>
#include <cstddef>
#include <cstring>
#include <string>

namespace rpicar {

std::string errno_to_str(const int errnum) noexcept {
    static constexpr std::size_t BUF_LEN{1024U};
    std::array<char, BUF_LEN> buf{};

    // NOLINTNEXTLINE(misc-include-cleaner): false positive, strerror_r is included through <cstring>
    return std::string{strerror_r(errnum, buf.data(), buf.size())}; 
}

} // namespace rpicar
