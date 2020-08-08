#include "global_debug.h"

void f(int v) {
    PRINT_ID("a", "a: f(%d) called\n", v);
}

int main(int argc, char *argv[])
{
    printf("Debugging...\n");
    // should trigger print
    gdebug::push_id("a");
    f(0);
    gdebug::pop_id();

    // should not trigger print
    gdebug::push_id("b");
    f(1);
    gdebug::pop_id();
    printf("Done...\n");

    return 0;
}
