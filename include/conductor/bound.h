#ifndef BOUND_H
#define BOUND_H

template <typename T>
constexpr T Bound(T const& a, T const& limit) {
    return std::max(std::min(a, limit), -limit);
}

#endif // BOUND_H