#ifndef ACTORNODE_HPP
#define ACTORNODE_HPP

#include <iostream>
#include <string>
#include <vector>
#include "MovieNode.hpp"

using namespace std;

/**
 * Actor nodes act as vertices
 */
class ActorNode {
  public:
    string actorName;
    vector<MovieNode*> movies;
};

#endif  // ACTORNODE_HPP