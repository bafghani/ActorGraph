
#include "Map.hpp"

/* TODO */
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

/* TODO */
void Map::Dijkstra(const string& from, const string& to,
                   vector<Vertex*>& shortestPath) {
    priority_queue<Vertex*, vector<Vertex*>, VertexCmp> explored;
    Vertex* src = vertices[vertexId[from]];
    Vertex* dest = vertices[vertexId[to]];
    if (src == 0 || dest == 0) {
        return;  // vertices dont exist
    }
    Vertex* curr;
    Vertex* neighbor;
    unsigned int currDist;

    src->dist = 0;
    explored.push(src);

    while (!explored.empty()) {
        curr = explored.top();
        explored.pop();
        if (!curr->visited) {
            curr->visited = true;
        }

        if (curr == dest) {
            break;  // we are done
        }
        for (unsigned int i = 0; i < curr->outEdges.size(); i++) {
            Edge* currEdge = curr->outEdges[i];
            currDist = curr->dist + currEdge->weight;
            neighbor = currEdge->target;
            if (curr->path ==
                currEdge) {  // check if current edge has already been traversed
                break;
            }
            if (!neighbor->visited && currDist < neighbor->dist) {
                neighbor->dist = currDist;
                neighbor->prev = curr;
                neighbor->path = currEdge;
                explored.push(neighbor);
            }
        }
    }
    if (curr != dest) {  // no path exists
        return;
    }
    if (dest->prev) {  // path exists, build it
        buildPath(dest, shortestPath);
    }
    // reset vertices data
    for (auto i = 0; i < vertices.size(); i++) {
        vertices[i]->visited = false;
        vertices[i]->prev = 0;
        vertices[i]->path = 0;
        vertices[i]->dist = INT_MAX;
    }
}

void Map::buildPath(Vertex* curr, vector<Vertex*>& shortestPath) {
    if (curr->prev == 0) {
        shortestPath.push_back(curr);
        return;
    } else {
        buildPath(curr->prev, shortestPath);
        shortestPath.push_back(curr);
    }
}

/* TODO */
void Map::findMST(vector<Edge*>& MST) {
    priority_queue<Edge*, vector<Edge*>, EdgeCmp> edges;
    Vertex* curr;
    Vertex* neighbor;
    Edge* currEdge;
    unsigned int totalWeight = 0;
    unsigned int edgeCount = 0;
    unsigned int vertexCount = 0;
    // fill PQ
    for (unsigned int i = 0; i < vertices.size(); i++) {
        curr = vertices[i];
        for (unsigned int j = 0; j < curr->outEdges.size(); j++) {
            currEdge = curr->outEdges[j];
            neighbor = currEdge->target;
            if (curr != neighbor) {
                Edge* newEdge =
                    new Edge(curr, neighbor,
                             currEdge->weight);  // these edges are never
                                                 // deleted, may cause mem leak
                edges.push(newEdge);
            }
        }
    }

    Vertex* parent1;
    Vertex* parent2;

    while (!edges.empty()) {
        currEdge = edges.top();
        edges.pop();
        parent1 = find(currEdge->source);
        parent2 = find(currEdge->target);

        if (parent1 != parent2) {
            totalWeight += currEdge->weight;
            ++edgeCount;
            if (!currEdge->source->visited) {
                ++vertexCount;
            }
            if (!currEdge->target->visited) {
                ++vertexCount;
            }
            currEdge->source->visited = true;
            currEdge->target->visited = true;

            if (parent1->vertexCount < parent2->vertexCount) {
                Union(parent1, currEdge->target);
            } else {
                Union(parent2, currEdge->source);
            }

            MST.push_back(currEdge);
        }
    }
}

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
void Map::Union(Vertex* parent, Vertex* branch) {
    Vertex* curr = branch;
    Vertex* previous = curr->parent;
    while (curr->parent != parent) {
        previous = curr->parent;
        curr->parent = parent;
        curr = previous;
    }
}

/* TODO */
void Map::crucialRoads(vector<Edge*>& roads) {}

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
