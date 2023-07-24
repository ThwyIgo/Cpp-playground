#pragma once

#include <stdexcept>
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

    static bool heapCmp(std::pair<T,Winf*> a, std::pair<T,Winf*> b) {
        return (*a.second > *b.second);
    }

    using heap_min = heap<std::pair<T,Winf*>, heapCmp>;

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

    // O(e) : e = edges
    // Doesn't check if edges are duplicated
    constexpr Graph(std::initializer_list<std::pair<T,std::initializer_list<std::pair<T,W>>>> il) {
        for (auto [k,v] : il)
            for (auto [d,w] : v)
                addEdge(k, d, w, true);
    }

    // O(1)
    size_t size() const {
        return verts.size();
    }

    // O(1)
    void addVert(T vert) {
        verts[vert];
    }

    // O(e) : e = edges
    void removeVert(T vert) {
        verts.erase(vert);
        for (auto& [origin,_] : verts)
            removeEdge(origin, vert);
    }

    // Vertices are created automatically
    // O(e) : e = edges
    bool addEdge(T origin, T destination, W weight) {
        addVert(destination);
        auto &adjList = verts[origin];
        if (std::find_if(adjList.begin(), adjList.end(),
                         eqEdge(destination))
            != adjList.end())
            return false;

        adjList.push_front({destination, weight});
        return true;
    }

    // Vertices are created automatically
    // O(1)
    bool addEdge(T origin, T destination, W weight, bool iSwearThisIsntDuplicate) {
        if (iSwearThisIsntDuplicate == false)
            return addEdge(origin, destination, weight);

        addVert(destination);
        verts[origin].push_front({destination, weight});
        return true;
    }

    // O(n) : n = origin's adjacent vertices
    bool removeEdge(T origin, T destination) {
        return std::erase_if(verts[origin],
                             eqEdge(destination));
    }

    // O(n) : n = origin's adjacent vertices
    [[nodiscard]]
    std::pair<T,W>* getEdgePtr(T origin, T destination) {
        if (!contains(origin))
            throw std::runtime_error("Edge's origin not found");

        auto& [_,adjList] = *verts.find(origin);
        auto it = std::find_if(adjList.begin(), adjList.end(),
                               eqEdge(destination));
        if (it == adjList.end())
            throw std::runtime_error("Edge not found");

        return &*it;
    }

    bool contains(T vert) const {
        return verts.contains(vert);
    }

    bool contains(T origin, T destination) const {
        if (!contains(origin))
            return false;

        auto [_,adjList] = *verts.find(origin);
        if (std::find_if(adjList.begin(), adjList.end(),
                         eqEdge(destination)) == adjList.end())
            return false;

        return true;
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
