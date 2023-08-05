#pragma once

#include <concepts>

template <typename T>
concept Arithmetic = requires(T a, T b) {
    { a + b } -> std::same_as<T>;
    { a - b } -> std::same_as<T>;
    { a * b } -> std::same_as<T>;
    { a / b } -> std::same_as<T>;
};

template<typename F, typename T>
concept CompareFunction = std::regular_invocable<F,T,T> && std::predicate<F,T,T>;
