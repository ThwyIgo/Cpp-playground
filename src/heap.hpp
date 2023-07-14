#pragma once

#include <vector>
#include <algorithm>
#include <functional>
#include <stdexcept>

template<typename T, bool (*Compare)(const T&, const T&) = std::less<T>()>
class heap {
private:
    std::vector<T> data;

public:
    heap() = default;
    heap(std::vector<T> data) : data(data) {
        heapify();
    }

    void push(T value) {
        data.push_back(value);
        std::push_heap(data.begin(), data.end(), Compare);
    }

    T pop() {
        if (data.size() == 0) 
            throw std::runtime_error("Heap is empty");

        std::pop_heap(data.begin(), data.end(), Compare);
        T top = data.back();
        data.pop_back();
        return top;
    }

    T top() const {
        if (data.size() == 0) 
            throw std::runtime_error("Heap is empty");

        return data.front();
    }

    void heapify() {
        std::make_heap(data.begin(), data.end(), Compare);
    }

    int size() const noexcept {
        return data.size();
    }

    bool empty() const noexcept {
        return data.empty();
    }
};