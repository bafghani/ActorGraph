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

    ActorNode(string name) : actorName(name)
};
fd
#endif  // ACTORNODE_HPP