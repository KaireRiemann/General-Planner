#include <general_core/utils/string_utils.hpp>

#include <algorithm>
#include <cctype>

namespace general_planner::core_utils {

std::string normalizeToken(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    std::replace(value.begin(), value.end(), '-', '_');
    return value;
}

} // namespace general_planner::core_utils

