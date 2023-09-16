#pragma once
#ifndef NDEBUG

/* Debug utilities that work only when the project is compiled in debug mode */

#include <cstddef>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

// Print a message and variables to stderr with line number
#define db_line(msg, ...)                      \
    __debug::varsLn(__LINE__, "(Debug) LINE ", \
                    msg __VA_OPT__(, {#__VA_ARGS__}, ) __VA_ARGS__)

// Print a message and variables to stderr with this file's name and line number
#define db_file(msg, ...)                                   \
    __debug::varsLn(__LINE__, "(Debug) " __FILE__ " LINE ", \
                    msg __VA_OPT__(, {#__VA_ARGS__}, ) __VA_ARGS__)

// Don't use this namespace directly in your code
namespace __debug {
void varsLn_h(size_t i, std::vector<const char*> names) {
    std::cerr << std::endl;
}

template <typename T, typename... Var>
void varsLn_h(size_t i, std::vector<const char*> names, T v, Var... vars) {
    std::cerr << names[i] << '=' << v << ",, ";
    varsLn_h(i + 1, names, vars...);
}

template <typename... Var>
void varsLn(const size_t&& ln,
            const char*&& prefix,
            const char*&& msg,
            const std::string names,
            Var... vars) {
    std::cerr << prefix << ln << ' ' << std::quoted(msg) << ": ";

    // Parse variable names
    std::vector<const char*> argNames;
    auto start = 0U;
    auto end = names.find(", ");
    while (end != std::string::npos) {
        argNames.push_back(names.substr(start, end - start).c_str());
        start = end + 2;
        end = names.find(", ", start);
    }
    argNames.push_back(names.substr(start, end).c_str());

    varsLn_h(0, argNames, vars...);
}

void varsLn(const size_t&& ln, const char*&& prefix, const char*&& msg) {
    std::cerr << prefix << ln << ' ' << std::quoted(msg) << std::endl;
}

}  // namespace __debug

#else
#define db_line(msg, ...)
#define db_file
#endif
