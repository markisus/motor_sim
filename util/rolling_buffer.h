#pragma once

#include <array>

struct RollingBufferContext {
    RollingBufferContext(size_t capacity) : capacity(capacity) {}

    size_t capacity;
    int next_idx = 0;
    bool wrap_around = false;
};

inline void rolling_buffer_advance_idx(RollingBufferContext* context) {
    ++(context->next_idx);
    if ((context->next_idx) >= context->capacity) {
        (context->next_idx) = 0;
        (context->wrap_around) = true;
    }
}

inline int get_rolling_buffer_count(const RollingBufferContext& context) {
    if (context.wrap_around) {
        return context.capacity;
    }
    return context.next_idx;
}

inline int get_rolling_buffer_begin(const RollingBufferContext& context) {
    if (context.wrap_around) {
        // if we are have wrapped around, the element that we are about to
        // overwrite (at next_idx) is the oldest, aka the logical beginning
        return context.next_idx;
    }
    return 0;
}

inline int get_rolling_buffer_end(const RollingBufferContext& context) {
    if (context.wrap_around) {
        return (context.next_idx + context.capacity - 1) % context.capacity;
    }
    return context.next_idx;
}
