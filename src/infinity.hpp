#pragma once

#include <limits>
#include <ostream>
#include <stdexcept>

/* Wrapper class to implement infinity to any type.
 * Doesn't support negative infinity.
 */
template <typename T>
class inf {
private:
    T value = {};
    bool isInf = true;

public:
    inf() = default;
    inf(T value) : value(value), isInf(false) {}

    bool isInfinity() const noexcept {
        return isInf;
    }

    explicit operator T() const {
        if (!isInf)
            return value;
        if constexpr (std::numeric_limits<T>::has_infinity)
            return std::numeric_limits<T>::infinity();

        throw std::logic_error("Cannot convert infinity to a value");
    }

    T getValue() const {
        return (T)(*this);
    }

    // Arithmetic operators

    friend inf operator+(const inf& lhs, const inf& rhs) {
        if (lhs.isInf || rhs.isInf)
            return inf();
        return inf(lhs.value + rhs.value);
    }

    friend inf operator-(const inf& lhs, const inf& rhs) {
        if (lhs.isInf && rhs.isInf)
            throw std::logic_error("infinity minus infinity");

        if (lhs.isInf || rhs.isInf)
            return inf();
        return inf(lhs.value - rhs.value);
    }

    friend inf operator*(const inf& lhs, const inf& rhs) {
        if (lhs.isInf || rhs.isInf)
            return inf();
        return inf(lhs.value * rhs.value);
    }

    friend inf operator/(const inf& lhs, const inf& rhs) {
        if (lhs.isInf && rhs.isInf)
            throw std::logic_error("infinity divided by infinity");

        if (!lhs.isInf && rhs.isInf)
            return 0;
        if (lhs.isInf)
            return inf();
        return inf(lhs.value / rhs.value);
    }

    inf& operator++() {
        if (!isInf)
            ++value;
        return *this;
    }

    inf operator++(int) {
        inf tmp(*this);
        operator++();
        return tmp;
    }

    inf& operator--() {
        if (!isInf)
            --value;
        return *this;
    }

    inf operator--(int) {
        inf tmp(*this);
        operator--();
        return tmp;
    }

    // Comparison operators

    friend bool operator==(const inf& lhs, const inf& rhs) {
        if (lhs.isInf && rhs.isInf)
            return true;
        if (lhs.isInf || rhs.isInf)
            return false;
        return lhs.value == rhs.value;
    }

    friend bool operator!=(const inf& lhs, const inf& rhs) {
        return !(lhs == rhs);
    }

    friend bool operator<(const inf& lhs, const inf& rhs) {
        if (lhs.isInf)
            return false;
        if (!lhs.isInf && rhs.isInf)
            return true;
        return lhs.value < rhs.value;
    }

    friend bool operator<=(const inf& lhs, const inf& rhs) {
        return (lhs < rhs) || (lhs == rhs);
    }

    friend bool operator>(const inf& lhs, const inf& rhs) {
        return !(lhs <= rhs);
    }

    friend bool operator>=(const inf& lhs, const inf& rhs) {
        return !(lhs < rhs);
    }

    // Other operators
    friend std::ostream& operator<<(std::ostream& os, const inf& obj) {
        if (obj.isInf)
            os << "inf";
        else
            os << obj.value;
        return os;
    }
};
