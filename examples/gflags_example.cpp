// This example sets up two flags. Try the following.
// bazel run //examples:gflags_example -- --bool_flag --int_flag 1337
// or
// bazel-bin/examples/gflags_example --bool_flag --int_flag 1337

#include <gflags/gflags.h>
#include <iostream>

DEFINE_bool(bool_flag, /*default=*/false, "the description of the bool flag");
DEFINE_int32(int_flag, /*default=*/0, "the description of the int flag");

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags*/ true);
    std::cout << "The bool flag was " << FLAGS_bool_flag << std::endl;
    std::cout << "The int flag was " << FLAGS_int_flag << std::endl;
}
