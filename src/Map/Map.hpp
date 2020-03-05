#ifndef MAP_HPP
#define MAP_HPP

#include <bits/stdc++.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include "Edge.hpp"

using namespace std;
class VertexCmp {
  public:
    bool operator()(Vertex* lhs, Vertex* rhs) const {
        return lhs->dist > rhs->dist;
    }
};
class EdgeCmp {
  public:
    bool operator()(Edge* lhs, Edge* rhs) const {
        return lhs->weight > rhs->weight;
    }
};
class Map {
  private:
    // vector storing vertices in the map: id of each vertex = index in vector
    vector<Vertex*> vertices;

    // Map: name of vertex -> id of vertex = index storing vertex ptr
    unordered_map<string, unsigned int> vertexId;

    // Directed edge in vector represents an undirected edge used in MST
    vector<Edge*> undirectedEdges;

    /*
     * Add a vertex with name and x, y coordinates to the map graph. Returns
     * false if the name already existed in the map graph, and true otherwise
     */
    bool addVertex(const string& name, float x, float y);

    /*
     * Add an undirected edge between vertices with names "name1" and "name2".
     * Returns false if either name is not in the map graph.
     */
    bool addEdge(const string& name1, const string& name2);

  public:
    /* TODO */
    Map();

    /* Build the map graph from vertex and edge files */
    bool buildMapFromFile(const string& vertexFileName,
                          const string& edgeFileName);

    /* TODO */
    void Dijkstra(const string& from, const string& to,
                  vector<Vertex*>& shortestPath);

    void buildPath(Vertex* curr, vector<Vertex*>& shortestPath);
    /* TODO */
    void findMST(vector<Edge*>& MST);
    Vertex* find(Vertex* vertex);
    void Union(Vertex* parent, Vertex* branch);

    /* TODO */
    void crucialRoads(vector<Edge*>& roads);

    /* Destructor of Map graph */
    ~Map();
};

#endif  // Map_HPP