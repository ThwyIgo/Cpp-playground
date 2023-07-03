#if __cplusplus < 202002L
#error "C++20 is required to compile this code"
#endif

#ifndef GRAPH_NO_IO
#include <iostream>
#endif // GRAPH_NO_IO

#include <algorithm>
#include <memory>
#include <limits>

#include "heap.hpp"
#include <optional>
#include <unordered_map>
#include <forward_list>

//            Value, Weight
template<typename T, typename W>
class Graph {
    using heap_min = heap<std::pair<T,std::shared_ptr<W>>,
        [](auto a, auto b) {
            return (*a.second > *b.second);
        }>;

    typedef std::pair<std::shared_ptr<std::unordered_map<
                                          T, W>>,
                      std::shared_ptr<std::unordered_map<
                                          T,std::optional<T>>>
                      > dijkstra_t;
        
    // The only attribute of this class
    std::unordered_map<T, std::forward_list<std::pair<T, W>>> verts;

public:
    Graph(void) {}

    explicit Graph(size_t reserve) {
        verts.reserve(reserve);
    }

    // O(n) : n = edges
    // Doesn't check if edges are duplicated
    constexpr Graph(std::initializer_list<std::pair<T,std::initializer_list<std::pair<T,W>>>> il) {
        verts.reserve(il.size());
        for (auto [k,v] : il)
            for (auto p : v)
                verts[k].push_front(p);
    }

    // O(n) : n = edges
    bool addEdge(T origin, T destination, W weight) {
        auto &list = verts[origin];
        if (std::find_if(list.begin(), list.end(),
                         eqEdge(destination))
            != list.end())
            return false;
        
        list.push_front({destination, weight});
        return true;
    }

    // O(1)
    bool addEdge(T origin, T destination, W weight, bool iSwearThisIsntDuplicate) {
        if (iSwearThisIsntDuplicate == false)
            return addEdge(origin, destination, weight);
        
        verts[origin].push_front({destination, weight});
        return true;
    }

    // O(n)
    bool removeEdge(T origin, T destination) {
        return std::erase_if(verts[origin],
                             eqEdge(destination));
    }

    // O(n) : n = origin's adjacent vertices
    std::pair<T,W>* getEdgePtr(T origin, T destination) {
        return &*std::find_if(verts[origin].begin(), verts[origin].end(),
                              eqEdge(destination));
    }
    
    // O(v * e) : v = vertices : e = edges
    dijkstra_t dijkstra(T origin) const {
        /* (*pprev)[v] == previous vertex of v in the path.
           if (*pprev)[v].has_value() == false, then v is the origin */
        auto pprev = std::make_shared<std::unordered_map<T,std::optional<T>>>();
        auto &prev = *pprev;
        prev.reserve(verts.size());
        heap_min heapMin;
        // *dist[v] == distance from origin to v
        std::unordered_map<T, std::shared_ptr<W>> dist;
        dist.reserve(verts.size());

        for (const auto &[v,_] : verts) {
            if (v != origin) {
                dist[v] = std::make_shared<W>(infinity());
                heapMin.push({v, dist[v]});
            }
            prev[v] = std::nullopt;
        }
        dist[origin] = std::make_shared<W>(0);
        heapMin.push({origin, dist[origin]});

        while (!heapMin.empty()) {
            const auto [u,uw] = heapMin.pop();
            const auto &[_,adjVerts] = *verts.find(u);

            for (auto &[v,w] : adjVerts) {
                W tempDist = *uw + w;

                if (dist[v] == nullptr)
                    dist[v] = std::make_shared<W>(infinity());
                if (tempDist < *dist[v]) {
                    *dist[v] = tempDist;
                    prev[v] = std::optional<T>(u);
                    heapMin.heapify();
                }
            }
        }

        auto distNoPtr = std::make_shared<std::unordered_map<T, W>>();
        (*distNoPtr).reserve(dist.size());
        for (auto &[v,pw] : dist) {
            (*distNoPtr)[v] = *pw;
            pw = nullptr; // free memory
        }
        
        return {distNoPtr, pprev};
    }

#ifndef GRAPH_NO_IO
    static void printPrevPath(const dijkstra_t &shortestPathTree, T destination) {
        const auto &[_,prevopt] = *(*shortestPathTree.second).find(destination);

        if (prevopt.has_value()) {
            printPrevPath(shortestPathTree, prevopt.value());
            std::cout << " -> ";
        }

        std::cout << destination;
    }

    static void printShortestDistances(const dijkstra_t &shortestPathTree) {
        const auto dist = *shortestPathTree.first;
        
        for (auto &[v,w] : dist) {
            std::cout << v << '/' << w << '\n';
        }
    }

    void print(void) const {
        for (auto &[k,l] : verts) {
            std::cout << k << " -> ";
            for (auto &[v,w] : l)
                std::cout << ", " << v << '/' << w;
            std::cout << '\n';
        }
    }
#endif // GRAPH_NO_IO

    constexpr static W infinity(void) {
        if (std::numeric_limits<W>::has_infinity)
            return std::numeric_limits<W>::infinity();
        return std::numeric_limits<W>::max();
    }
    
private:    
    auto static eqEdge(const T &destination) {
        return [&](const std::pair<T,W> &pair)
            {return pair.first == destination;};
    }
};
