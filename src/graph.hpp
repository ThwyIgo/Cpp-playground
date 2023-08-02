#pragma once

#ifndef GRAPH_NO_IO
#include <iostream>
#endif  // GRAPH_NO_IO

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include "concepts.hpp"
#include "infinity.hpp"

#include <forward_list>
#include <unordered_map>
#include "heap.hpp"

// Weighted directed generic graph
template <std::equality_comparable T, typename W>  // Value, Weight
class Graph {
    using Winf = inf<W>;
    using Vertex = std::shared_ptr<T>;
    using vertsMap_t =
        std::unordered_map<Vertex, std::forward_list<std::pair<Vertex, W>>>;

    // easy way to get vertexPtr in const functions
#define vertexPtr(v) vertexPtr.find(v)->second

    // The only data attribute of this class
    vertsMap_t verts;

  public:
    typedef std::pair<std::unordered_map<T, Winf>,
                      std::unordered_map<T, std::optional<T>>>
        dijkstra_t;

    Graph() = default;
    explicit Graph(std::size_t reserve) {
        verts.reserve(reserve);
    }

    // O(e) : e = edges
    // Doesn't check if edges are duplicated
    constexpr Graph(std::initializer_list<
                    std::pair<T, std::initializer_list<std::pair<T, W>>>> il) {
        for (auto [k, v] : il)
            for (auto [d, w] : v)
                addEdge(k, d, w, true);
    }

    // Number of vertices in the graph
    // O(1)
    std::size_t size() const {
        return verts.size();
    }

    // O(v)
    bool addVert(T vert) {
        if (contains(vert))
            return false;

        verts[vertexPtr[vert] = std::make_shared<T>(vert)];
        return true;
    }

    // O(e) : e = edges
    bool removeVert(T vert) {
        if (!contains(vert))
            return false;

        verts.erase(vertexPtr[vert]);
        for (auto& [origin, _] : verts)
            removeEdge(*origin, vert);

        vertexPtr.erase(vert);
        return true;
    }

    void modifyVertex(T vert, T newVert) {
        if (!contains(vert))
            throw std::runtime_error("Vertex not found");
        if (contains(newVert))
            throw std::runtime_error("newVert already exists");

        *vertexPtr[vert] = newVert;
        vertexPtr[newVert] = vertexPtr[vert];

        vertexPtr.erase(vert);
    }

    // Vertices are created automatically
    // O(e) : e = edges
    bool addEdge(T origin, T destination, W weight) {
        addVert(origin);
        addVert(destination);

        auto& adjList = verts[vertexPtr[origin]];
        if (std::ranges::find_if(adjList, eqEdge(destination)) != adjList.end())
            return false;

        adjList.push_front({vertexPtr[destination], weight});
        return true;
    }

    // Vertices are created automatically
    // O(1)
    bool addEdge(T origin,
                 T destination,
                 W weight,
                 bool iSwearThisIsntDuplicate) {
        if (iSwearThisIsntDuplicate == false)
            return addEdge(origin, destination, weight);

        addVert(origin);
        addVert(destination);
        verts[vertexPtr[origin]].push_front({vertexPtr[destination], weight});
        return true;
    }

    // O(n) : n = origin's adjacent vertices
    bool removeEdge(T origin, T destination) {
        if (contains(origin) && contains(destination)) {
            std::erase_if(verts[vertexPtr[origin]], [&](auto edge) {
                return edge.first == vertexPtr[destination];
            });
            return true;
        }
        return false;
    }

    // O(n) : n = origin's adjacent vertices
    void modifyEdgeWeight(T origin, T destination, W newWeight) {
        if (!contains(origin) || !contains(destination))
            throw std::runtime_error("Edge not found");

        auto& [_, adjList] = *verts.find(vertexPtr[origin]);
        auto it = std::ranges::find_if(adjList, eqEdge(destination));
        if (it == adjList.end())
            throw std::runtime_error("Edge not found");

        it->second = newWeight;
    }

    // Check if vertex is in the graph
    // O(1)
    bool contains(T vert) const {
        return vertexPtr.contains(vert);
    }

    // Check if edge is in the graph
    // O(e) : e = origin's adjacent vertices
    bool contains(T origin, T destination) const {
        if (!contains(origin) || !contains(destination))
            return false;

        auto [_, adjList] = *verts.find(vertexPtr(origin));
        if (std::ranges::find_if(adjList, eqEdge(destination)) == adjList.end())
            return false;

        return true;
    }

    // O(v * e) : v = vertices : e = edges
    [[nodiscard]] dijkstra_t dijkstra(T origin) const requires
        Arithmetic<W> && std::totally_ordered<W> {
        if (!contains(origin))
            throw std::runtime_error("Origin not found");

        heap<std::pair<T, Winf*>,
             [](auto a, auto b) { return (*a.second > *b.second); }>
            heapMin;
        /* prev[v] == previous vertex of v in the path.
           if prev[v].has_value() == false, then v is the origin */
        std::unordered_map<T, std::optional<T>> prev;
        // dist[v] == distance from origin to v
        std::unordered_map<T, Winf> dist;

        for (const auto& [v, _] : verts) {
            if (*v != origin) {
                dist[*v] = Winf();
                heapMin.push({*v, &dist[*v]});
            }
            prev[*v] = std::nullopt;
        }
        dist[origin] = Winf(0);
        heapMin.push({origin, &dist[origin]});

        while (!heapMin.empty()) {
            const auto [u, uw] = heapMin.pop();
            const auto& [_, adjVerts] = *verts.find(vertexPtr(u));

            for (auto& [v, w] : adjVerts) {
                Winf tempDist = *uw + w;

                if (tempDist < dist[*v]) {
                    dist[*v] = tempDist;
                    prev[*v] = std::optional<T>(u);
                    heapMin.heapify();
                }
            }
        }

        return {dist, prev};
    }

#ifndef GRAPH_NO_IO
    static void printPrevPath(const dijkstra_t& shortestPathTree,
                              T destination) {
        const auto& [_, prevopt] = *shortestPathTree.second.find(destination);

        if (prevopt.has_value()) {
            printPrevPath(shortestPathTree, prevopt.value());
            std::cout << " -> ";
        }

        std::cout << destination;
    }

    static void printShortestDistances(const dijkstra_t& shortestPathTree) {
        const auto dist = shortestPathTree.first;

        for (auto& [v, w] : dist) {
            std::cout << v << '/' << w << '\n';
        }
    }

    void print() const {
        if (size() == 0) {
            std::cout << "Empty\n";
            return;
        }
        for (auto& [k, l] : verts) {
            std::cout << *k << " -> ";
            for (auto& [v, w] : l)
                std::cout << ", " << *v << '/' << w;
            std::cout << '\n';
        }
    }
#endif  // GRAPH_NO_IO

    ///// Iterators /////

    /* Simple for-loop iterator over vertices.
       Elements aren't ordered here */
    class iterator {
        typename vertsMap_t::iterator it;

        friend iterator Graph<T, W>::begin();
        friend iterator Graph<T, W>::end();

        iterator(auto it) : it(it) {}

      public:
        T& operator*() { return *it->first; }

        iterator& operator++() {
            ++it;
            return *this;
        }

        bool operator!=(const iterator& other) { return it != other.it; }
    };

    iterator begin() {
        return iterator(verts.begin());
    }

    iterator end() {
        return iterator(verts.end());
    }

  private:
    // Map to convert a T to a Vertex*
    std::unordered_map<T, Vertex> vertexPtr;

    auto static eqEdge(const T& destination) {
        return [&](const std::pair<Vertex, W>& pair) {
            return *pair.first == destination;
        };
    }
#undef vertexPtr
};
