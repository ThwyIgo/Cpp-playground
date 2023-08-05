#pragma once

#include <concepts>

// Basic graph interface
template <std::equality_comparable T>
class Graph {
  public:
    // Number of vertices in the graph
    virtual std::size_t size() const = 0;

    // Returns true if successful, false otherwise
    virtual bool addVert(T vert) = 0;

    /* Removing a vertex should remove all it's appearences in the graph.
       Returns true if successful, false otherwise */
    virtual bool removeVert(T vert) = 0;

    // Modifying a vertex should change all it's appearences in the graph
    virtual void modifyVertex(T vert, T newVert) = 0;

    /* Add edge with a default weight.
       Vertices should be created automatically.
       Returns true if successful, false otherwise.*/
    virtual bool addEdge(T origin, T destination) = 0;

    // Returns true if successful, false otherwise
    virtual bool removeEdge(T origin, T destination) = 0;

    // Check if vertex is in the graph
    virtual bool contains(T vert) const = 0;

    // Check if edge is in the graph
    virtual bool contains(T origin, T destination) const = 0;
};
