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
#include "basic_graph.hpp"
#include "heap.hpp"

// Weighted directed generic graph
template <std::equality_comparable T, typename W>  // Value, Weight
class WeightedDigraph : Graph<T> {
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

    WeightedDigraph() = default;
    explicit WeightedDigraph(std::size_t reserve) {
        verts.reserve(reserve);
    }

    // O(e) : e = edges
    // Doesn't check if edges are duplicated
    constexpr WeightedDigraph(
        std::initializer_list<
            std::pair<T, std::initializer_list<std::pair<T, W>>>> il) {
        for (auto [k, v] : il)
            for (auto [d, w] : v)
                addEdge(k, d, w, true);
    }

    // Number of vertices in the graph
    // O(1)
    std::size_t size() const override {
        return verts.size();
    }

    // O(1)
    bool addVert(T vert) override {
        if (contains(vert))
            return false;

        verts[vertexPtr[vert] = std::make_shared<T>(vert)];
        return true;
    }

    // O(e) : e = edges
    bool removeVert(T vert) override {
        if (!contains(vert))
            return false;

        verts.erase(vertexPtr[vert]);
        for (auto& [origin, _] : verts)
            removeEdge(*origin, vert);

        vertexPtr.erase(vert);
        return true;
    }

    void modifyVertex(T vert, T newVert) override {
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

    bool addEdge(T origin, T destination) override {
        return addEdge(origin, destination, W());
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
    bool removeEdge(T origin, T destination) override {
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
    bool contains(T vert) const override {
        return vertexPtr.contains(vert);
    }

    // Check if edge is in the graph
    // O(e) : e = origin's adjacent vertices
    bool contains(T origin, T destination) const override {
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
       Elements aren't ordered here.
       It should satisfy std::output_iterator<iterator, T> */
    class iterator {
        typename vertsMap_t::iterator it;

      public:
        using difference_type = typename vertsMap_t::iterator::difference_type;

        iterator(const typename vertsMap_t::iterator& it) : it(std::move(it)) {}

        T& operator*() { return *it->first; }

        iterator& operator++() {
            ++it;
            return *this;
        }

        iterator operator++(int) {
            iterator ret = *this;
            operator++();
            return ret;
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

#include <queue>

template <std::equality_comparable T>
class undigraph : Graph<T> {
    using Vertex = std::shared_ptr<T>;
    using vertsMap_t = std::unordered_map<Vertex, std::forward_list<Vertex>>;

    // easy way to get vertexPtr in const functions
#define vertexPtr(v) vertexPtr.find(v)->second

    vertsMap_t verts;

  public:
    // Number of vertices in the graph
    // O(1)
    std::size_t size() const override {
        return verts.size();
    }

    // O(1)
    bool addVert(T vert) override {
        if (contains(vert))
            return false;

        verts[vertexPtr[vert] = std::make_shared<T>(vert)];
        return true;
    }

    bool removeVert(T vert) override {
        if (!contains(vert))
            return false;

        verts.erase(vertexPtr[vert]);
        for (auto& [origin, _] : verts)
            removeEdge(*origin, vert);

        vertexPtr.erase(vert);
        return true;
    }

    void modifyVertex(T vert, T newVert) override {
        if (!contains(vert))
            throw std::runtime_error("Vertex not found");
        if (contains(newVert))
            throw std::runtime_error("newVert already exists");

        *vertexPtr[vert] = newVert;
        vertexPtr[newVert] = vertexPtr[vert];

        vertexPtr.erase(vert);
    }

    bool addEdge(T origin, T destination) override {
        addVert(origin);
        addVert(destination);

        auto& adjList = verts[vertexPtr[origin]];
        if (std::ranges::find(adjList, vertexPtr[destination]) != adjList.end())
            return false;

        adjList.push_front(vertexPtr[destination]);
        verts[vertexPtr[destination]].push_front(vertexPtr[origin]);
        return true;
    }

    bool addEdge(T origin, T destination, bool iSwearThisIsntDuplicate) {
        addVert(origin);
        addVert(destination);

        verts[vertexPtr[origin]].push_front(vertexPtr[destination]);
        verts[vertexPtr[destination]].push_front(vertexPtr[origin]);
        return true;
    }

    bool removeEdge(T origin, T destination) override {
        if (contains(origin) && contains(destination)) {
            std::erase(verts[vertexPtr[origin]], vertexPtr[destination]);
            std::erase(verts[vertexPtr[destination]], vertexPtr[origin]);
            return true;
        }
        return false;
    }

    // Check if vertex is in the graph
    // O(1)
    bool contains(T vert) const override {
        return vertexPtr.contains(vert);
    }

    bool contains(T origin, T destination) const override {
        if (!contains(origin) || !contains(destination))
            return false;

        auto [_, adjList] = *verts.find(vertexPtr(origin));
        if (std::ranges::find(adjList, vertexPtr(destination)) == adjList.end())
            return false;

        return true;
    }

    // Check if a graph is bipartite using BFS
    bool isBipartite() {
        /* If the colorof[vert] is nil, vert hasn't been visited yet */
        enum class Color { nil, black, white };
        Color current = Color::black;
        std::unordered_map<Vertex, Color> colorof;
        std::queue<Vertex> q;

        for (auto [v, _] : verts)
            colorof[v] = Color::nil;

        for (auto [origin, _] : verts) {
            if (colorof[origin] != Color::nil)
                continue;

            q.push(origin);
            while (!q.empty()) {
                Vertex v = q.front();
                q.pop();
                if (colorof[v] == Color::nil)
                    colorof[v] = current;

                current = colorof[v];

                for (Vertex adj : verts[v]) {
                    if (colorof[adj] == current)
                        return false;

                    if (colorof[adj] == Color::nil) {
                        q.push(adj);
                        // Invert color
                        colorof[adj] = current == Color::black ? Color::white
                                                               : Color::black;
                    }
                }
            }
        }

        return true;
    }

  private:
    // Map to convert a T to a Vertex*
    std::unordered_map<T, Vertex> vertexPtr;

#undef vertexPtr
};
