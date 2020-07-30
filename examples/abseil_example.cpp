#include <absl/container/flat_hash_map.h>
#include <absl/strings/str_format.h>
#include <cstdio>
#include <string>

int main(int argc, char* argv[]) {
    absl::flat_hash_map<int, float> some_map;
    some_map[5] = 1.2345f;

    std::string s =
        absl::StrFormat("I am %s cool because I am %d years old\n", "very", 29);

    printf("Formatted string:\n%s\n", s.c_str());
    return 0;
}
