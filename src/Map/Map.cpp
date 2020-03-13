
#include "Map.hpp"

/* Map Constructor */
Map::Map() {}

/* Build the map graph from vertex and edge files */
bool Map::buildMapFromFile(const string& vertexFileName,
                           const string& edgeFileName) {
    // add vertices first
    ifstream vertexFile(vertexFileName);
    while (vertexFile) {
        string s;
        if (!getline(vertexFile, s)) break;

        // process data at each line
        istringstream ss(s);
        vector<string> data;
        while (ss) {
            string str;
            if (!getline(ss, str, ' ')) break;
            data.push_back(str);
        }
        if (data.size() != 3) continue;

        // add vertex defined in this line to the graph
        string name(data[0]);
        float x = stoi(data[1]);
        float y = stoi(data[2]);

        addVertex(name, x, y);
    }

    // then add edges
    ifstream edgeFile(edgeFileName);
    while (edgeFile) {
        string s;
        if (!getline(edgeFile, s)) break;

        // process data at each line
        istringstream ss(s);
        vector<string> data;
        while (ss) {
            string str;
            if (!getline(ss, str, ' ')) break;
            data.push_back(str);
        }
        if (data.size() != 2) continue;

        // add edge defined in this line to the graph
        string name1(data[0]);
        string name2(data[1]);

        addEdge(name1, name2);
    }

    return true;
}

/*
 * Add a vertex with name and x, y coordinates to the map graph. Returns
 * false if the name already existed in the map graph, and true otherwise
 */
bool Map::addVertex(const string& name, float x, float y) {
    if (vertexId.count(name) > 0) return false;
    vertexId[name] = vertices.size();
    vertices.push_back(new Vertex(name, x, y));
    return true;
}

/*
 * Add an undirected edge between vertices with names "name1" and "name2".
 * Returns false if either name is not in the map graph.
 */
bool Map::addEdge(const string& name1, const string& name2) {
    if (vertexId.count(name1) == 0 || vertexId.count(name2) == 0) {
        return false;
    }
    unsigned int id1 = vertexId[name1];
    unsigned int id2 = vertexId[name2];
    Vertex* v1 = vertices[id1];
    Vertex* v2 = vertices[id2];
    float weight = sqrt(pow(v1->x - v2->x, 2) + pow(v1->y - v2->y, 2));
    v1->outEdges.push_back(new Edge(v1, v2, weight));
    v2->outEdges.push_back(new Edge(v2, v1, weight));

    undirectedEdges.push_back(new Edge(v1, v2, weight));
    return true;
}

/* Finds the shortest weighted path from one destination to another
   Based on a Breadth First Search
   Stores the shortest Path in the shortestPath string */
void Map::Dijkstra(const string& from, const string& to,
                   vector<Vertex*>& shortestPath) {
    // reset vertices data
    for (unsigned int i = 0; i < vertices.size(); i++) {
        vertices[i]->visited = false;
        vertices[i]->prev = 0;
        vertices[i]->path = 0;
        vertices[i]->dist = INT_MAX;
    }
    priority_queue<Vertex*, vector<Vertex*>, VertexCmp>
        traversed;  // priority queue to store vertices that have been traversed
    Vertex* src = vertices[vertexId[from]];  // sets src vertex if it exists
    Vertex* dest =
        vertices[vertexId[to]];  // sets destination vertex if it exists
    if (src == 0 || dest == 0) {
        return;  //  if vertices dont exist
    }
    Vertex* curr;           // vertex to store the current vertex
    Vertex* neighbor;       // vertex storing curr's neighbor vertex
    unsigned int currDist;  // to store the distance from src

    src->dist = 0;        // set src distance to 0
    traversed.push(src);  // add src to PQ

    while (!traversed.empty()) {  // while PQ not empty
        curr = traversed.top();   // curr is vertex of shortest distance
        traversed.pop();          // remove from PQ
        if (!curr->visited) {
            curr->visited = true;  // set visited
        }

        if (curr == dest) {
            break;  // we are done
        }
        for (unsigned int i = 0; i < curr->outEdges.size();
             i++) {  // for all of curr's neighbors
            Edge* currEdge = curr->outEdges[i];
            currDist =
                curr->dist + currEdge->weight;  // increment distance from src
            neighbor = currEdge->target;        // increment neighbor
            if (curr->path ==
                currEdge) {  // check if current edge has already been traversed
                break;
            }
            if (!neighbor->visited &&
                currDist <
                    neighbor->dist) {  // if neighbor has not yet been traversed
                                       // and it is the shortest path
                neighbor->dist = currDist;  // set neighbor dist
                neighbor->prev =
                    curr;  // set neighbor's previous vertex for path building
                neighbor->path =
                    currEdge;  // set the path traversed for path building
                traversed.push(
                    neighbor);  // add new vertex traversed to traversed
            }
        }
    }
    if (curr != dest) {  // no path exists
        return;
    }
    if (dest->prev) {  // path exists, build it
        buildPath(dest, shortestPath);
    }
}

void Map::buildPath(Vertex* curr, vector<Vertex*>& shortestPath) {
    if (curr->prev == 0) {  // base case curr is src
        shortestPath.push_back(curr);
        return;
    } else {  // recurse through previous actors to build shortestPath
        buildPath(curr->prev, shortestPath);
        shortestPath.push_back(curr);
    }
}

/* finds the Minimum Weight Spanning Tree of the given Map
 * That is the vector MST will contain all edges required to
 * reach every vertex with the minimum total weight */
void Map::findMST(vector<Edge*>& MST) {
    priority_queue<Edge*, vector<Edge*>, EdgeCmp>
        edgesTraversed;  // PQ to store edges being traversed
    Vertex* curr;        // Vertex to store the current vertex being traversed
    Vertex* neighbor;    // Vertex to store the current vertex's neighbor
    Edge* currEdge;      // Edge to store the current edge being traversed
    unsigned int totalWeight = 0;  // stores the totalWeight of the tree
    unsigned int edgeCount =
        0;  // stores the number of edges that have been traversed
    unsigned int vertexCount =
        0;  // stores the number of vertices that have traversed
    // reset vertices data
    for (unsigned int i = 0; i < vertices.size(); i++) {
        vertices[i]->visited = false;
        vertices[i]->prev = 0;
        vertices[i]->path = 0;
        vertices[i]->dist = INT_MAX;
    }
    // fill PQ
    for (unsigned int i = 0; i < vertices.size(); i++) {  // for all vertices
        curr = vertices[i];
        for (unsigned int j = 0; j < curr->outEdges.size();
             j++) {  // for all current vertex's out edges
            currEdge = curr->outEdges[j];
            neighbor = currEdge->target;
            if (curr != neighbor) {  // if not self loop
                Edge* newEdge =
                    new Edge(curr, neighbor,
                             currEdge->weight);  // these edges are never
                                                 // deleted, may cause mem leak
                undirectedEdges.push_back(newEdge);
                edgesTraversed.push(newEdge);
            }
        }
    }

    Vertex* parent1;
    Vertex* parent2;

    while (!edgesTraversed.empty()) {  // while PQ not empty
        currEdge = edgesTraversed.top();
        edgesTraversed.pop();
        parent1 = find(
            currEdge
                ->source);  // finds the parent of source vertex of current edge
        parent2 = find(currEdge->target);  // finds the parent of destination
                                           // vertex of current edge

        if (parent1 != parent2) {             // if different parents
            totalWeight += currEdge->weight;  // increment weight
            ++edgeCount;                      // increment edge count
            if (!currEdge->source->visited) {
                ++vertexCount;  // increment vertex count
            }
            if (!currEdge->target->visited) {
                ++vertexCount;  // increment vertex count
            }
            currEdge->source->visited =
                true;  // set both source and destination vertices to visited
            currEdge->target->visited = true;

            if (parent1->vertexCount < parent2->vertexCount) {
                Union(
                    parent1,
                    currEdge->target);  // sets the parent vertex of the target
            } else {
                Union(
                    parent2,
                    currEdge->source);  // sets the parent vertex of the source
            }

            MST.push_back(currEdge);  // add the current edge to the Min Weight
                                      // Spanning Tree
        }
    }
}
/* Helper method to find the parent of the current vertex */
Vertex* Map::find(Vertex* vertex) {
    Vertex* curr = vertex;
    unsigned int count = 0;
    while (curr->parent != curr) {
        curr = curr->parent;
        ++count;
    }
    curr->vertexCount = count;
    return curr;
}
/* Helper method to set the parent of the branch */
void Map::Union(Vertex* parent, Vertex* root) {
    Vertex* curr = root;
    Vertex* previous = curr->parent;
    while (curr->parent != parent) {
        previous = curr->parent;
        curr->parent = parent;
        curr = previous;
    }
}

/* Finds the bridges, or edges that are not contained in any cycles. Relies on
 * Depth First Search for cycle detection */
void Map::crucialRoads(vector<Edge*>& roads) {
    unsigned int count = 0;  // to store the number of edges traversed
    vector<int> pre(
        vertices.size(),
        0);  // to store the preorder of the vertex traversed at each index
    vector<int> low(vertices.size(),
                    0);  // to store the lowest vertex reachable from each index
    dfs(pre, low, count, roads, 0,
        0);  // perform a depth first search starting at the first vertex
}
/* Depth First Search: helper method for cycle detection */
void Map::dfs(vector<int>& pre, vector<int>& low, unsigned int count,
              vector<Edge*>& roads, unsigned int preIdx, unsigned int lowIdx) {
    count++;  // increment count to store order in which vertices are traversed
    pre[preIdx] = low[preIdx] = count;  // set arrays at current index to count
    Vertex* curr =
        vertices[preIdx];  // curr stores current Vertex being traversed
    for (unsigned int i = 0; i < curr->outEdges.size();
         i++) {  // for all of curr's neighbors
        Edge* currEdge = curr->outEdges[i];
        unsigned int nextIdx =
            vertexId[currEdge->target->name];  // next index is neighbor's index
                                               // in vertexId
        if (pre[nextIdx] == 0) {  // if pre[nextIdx] has not been traversed
            dfs(pre, low, count, roads, nextIdx, preIdx);  // continue with dfs
            if (low[nextIdx] < low[preIdx]) {
                low[preIdx] = low[nextIdx];  // update low if we are still
                                             // connected to vertex at preIdx
            }
            if (low[nextIdx] > pre[preIdx]) {  // if vertex at nextIdx is not
                                               // connected to vertex at preIdx
                roads.push_back(currEdge);     // then this edge is a bridge
            }
        } else if (nextIdx != lowIdx) {        // otherwise the cycle continues
            if (pre[nextIdx] < low[preIdx]) {  // update low
                low[preIdx] = pre[nextIdx];
            }
        }
    }
}

/* Destructor of Map graph */
Map::~Map() {
    for (Vertex* v : vertices) {
        for (Edge* e : v->outEdges) {
            delete e;
        }
        delete v;
    }
    for (Edge* e : undirectedEdges) {
        delete e;
    }
}
