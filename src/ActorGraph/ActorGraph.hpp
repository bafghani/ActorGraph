/**
 * TODO: add file header
 */

#ifndef ACTORGRAPH_HPP
#define ACTORGRAPH_HPP

#include <iostream>
#include <list>
#include <unordered_map>
#include <vector>
#include "ActorNode.hpp"
#include "MovieNode.hpp"

using namespace std;

/**
 * TODO: add class header
 */
class ActorGraph {
  protected:
    unordered_map<string, MovieNode*> moviesMap;
    unordered_map<string, ActorNode*> actorsMap;

  public:
    /* TODO */
    ActorGraph();

    /* TODO */
    bool buildGraphFromFile(const char* filename);

    /* TODO */
    void BFS(const string& fromActor, const string& toActor,
             string& shortestPath);
    void buildPath(ActorNode* curr, string& shortestPath);

    /* TODO */
    void predictLink(const string& queryActor, vector<string>& predictionNames,
                     unsigned int numPrediction);

    /* TODO */
    ~ActorGraph();
};

#endif  // ACTORGRAPH_HPP
