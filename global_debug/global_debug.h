#include <string>

namespace gdebug {

void push_id(std::string id);
void pop_id();
bool have_id(const std::string& id);

} // namespace gdebug

#define PRINT_ID(ID, ...)                                                      \
    if (::gdebug::have_id(ID))                                                 \
        printf(__VA_ARGS__);
