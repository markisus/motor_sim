#include "global_debug.h"
#include <string>
#include <vector>

namespace gdebug {

std::vector<std::string> debug_ids;

void push_id(std::string id) { debug_ids.push_back(std::move(id)); }
void pop_id() { debug_ids.pop_back(); }
bool have_id(const std::string& id) {
    return std::find(debug_ids.begin(), debug_ids.end(), id) != debug_ids.end();
}

} // namespace gdebug
