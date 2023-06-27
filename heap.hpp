#include <vector>
#include <algorithm>
#include <functional>

template<typename T, bool(*Compare)(const T&, const T&) = std::less<>()>
class heap {
    std::vector<T> data;

public:
    heap(void) {}

    constexpr heap(std::vector<T> vec) : data(vec) {
        heapify();
    }
    
    void push(T x) {
        data.push_back(x);
        std::push_heap(data.begin(), data.end(), Compare);
    }

    T pop(void) {
        std::pop_heap(data.begin(), data.end(), Compare);
        T ret = data.back();
        data.pop_back();
        return ret;
    }

    void heapify(void) {
        std::make_heap(data.begin(), data.end(), Compare);
    }

    bool empty(void) const {
        return data.empty();
    }
};
