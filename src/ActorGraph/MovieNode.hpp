#ifndef MOVIENODE_HPP
#define MOVIENODE_HPP

#include <iostream>
#include <string>
#include <vector>

#include "ActorNode.hpp"

using namespace std;

class MovieNode {
  public:
    string title;
    unsigned int year;

    vector<ActorNode*> actors;
};

#endif  // MOVIENODE_HPP