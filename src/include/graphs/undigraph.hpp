#pragma once

#include <algorithm>
#include <memory>
#include <queue>

#include <forward_list>
#include <unordered_map>
#include "graphs/basic_graph.hpp"

// Undirected graph
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
    bool isBipartite() const {
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

                for (Vertex adj : verts.find(v)->second) {
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
