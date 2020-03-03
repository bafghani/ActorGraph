#ifndef ACTORNODE_HPP
#define ACTORNODE_HPP

#include <iostream>
#include <string>
#include <vector>
#include "MovieNode.hpp"

using namespace std;

class MovieNode;
/**
 * Actor nodes act as vertices
 */
class ActorNode {
  public:
    string actorName;
    vector<MovieNode*> movieList;
    vector<ActorNode*> neighbors;
    bool visited = false;
    ActorNode* prev = 0;
    MovieNode* path = 0;
    ActorNode(string name) : actorName(name) {}
};

#endif  // ACTORNODE_HPP