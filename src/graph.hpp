#pragma once

#ifndef GRAPH_NO_IO
#include <iostream>
#endif // GRAPH_NO_IO

#include <algorithm>
#include <limits>
#include "infinity.hpp"
#include "concepts.hpp"

#include "heap.hpp"
#include <optional>
#include <unordered_map>
#include <forward_list>

//                 Value               Weight
template<std::equality_comparable T, typename W>
class Graph {
    typedef inf<W> Winf;

    using heap_min = heap<std::pair<T,Winf*>,
        [](auto a, auto b) {
            return (*a.second > *b.second);
        }>;
        
    // The only attribute of this class
    std::unordered_map<T, std::forward_list<std::pair<T, W>>> verts;

public:
    typedef std::pair<std::unordered_map<
                          T, Winf>,
                      std::unordered_map<
                          T, std::optional<T>>
                      > dijkstra_t;

    Graph() = default;
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
    [[nodiscard]]
    std::pair<T,W>* getEdgePtr(T origin, T destination) {
        return &*std::find_if(verts[origin].begin(), verts[origin].end(),
                              eqEdge(destination));
    }
    
    // O(v * e) : v = vertices : e = edges
    [[nodiscard]]
    dijkstra_t dijkstra(T origin) const requires Arithmetic<W> && std::totally_ordered<W> {
        heap_min heapMin;
        /* (*pprev)[v] == previous vertex of v in the path.
           if (*pprev)[v].has_value() == false, then v is the origin */
        std::unordered_map<T,std::optional<T>> prev;
        // dist[v] == distance from origin to v
        std::unordered_map<T, Winf> dist;

        for (const auto &[v,_] : verts) {
            if (v != origin) {
                dist[v] = Winf();
                heapMin.push({v, &dist[v]});
            }
            prev[v] = std::nullopt;
        }
        dist[origin] = Winf(0);
        heapMin.push({origin, &dist[origin]});

        while (!heapMin.empty()) {
            const auto [u,uw] = heapMin.pop();
            const auto &[_,adjVerts] = *verts.find(u);

            for (auto &[v,w] : adjVerts) {
                Winf tempDist = *uw + w;

                if (tempDist < dist[v]) {
                    dist[v] = tempDist;
                    prev[v] = std::optional<T>(u);
                    heapMin.heapify();
                }
            }
        }

        return {dist, prev};
    }

#ifndef GRAPH_NO_IO
    static void printPrevPath(const dijkstra_t &shortestPathTree, T destination) {
        const auto &[_,prevopt] = *shortestPathTree.second.find(destination);

        if (prevopt.has_value()) {
            printPrevPath(shortestPathTree, prevopt.value());
            std::cout << " -> ";
        }

        std::cout << destination;
    }

    static void printShortestDistances(const dijkstra_t &shortestPathTree) {
        const auto dist = shortestPathTree.first;
        
        for (auto &[v,w] : dist) {
            std::cout << v << '/' << w << '\n';
        }
    }

    void print() const {
        for (auto &[k,l] : verts) {
            std::cout << k << " -> ";
            for (auto &[v,w] : l)
                std::cout << ", " << v << '/' << w;
            std::cout << '\n';
        }
    }
#endif // GRAPH_NO_IO
    
private:    
    auto static eqEdge(const T &destination) {
        return [&](const std::pair<T,W> &pair)
            {return pair.first == destination;};
    }
};
