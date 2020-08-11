#pragma once

// increment the timer by dt
// if the timer is greater than period, reset the timer and return true
template <typename TScalar>
inline bool periodic_timer(const TScalar period, const TScalar dt,
                           TScalar* timer) {
    *timer += dt;
    if (*timer >= period) {
        *timer -= period;
        return true;
    }
    return false;
}
