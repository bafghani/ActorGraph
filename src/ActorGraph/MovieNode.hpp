#ifndef MOVIENODE_HPP
#define MOVIENODE_HPP

#include <iostream>
#include <string>
#include <vector>

#include "ActorNode.hpp"

using namespace std;

class ActorNode;

class MovieNode {
  public:
    string title;

    vector<ActorNode*> actorsList;

    MovieNode(string movieTitle) : title(movieTitle) {}
};

#endif  // MOVIENODE_HPP