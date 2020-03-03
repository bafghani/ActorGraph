#include <gtest/gtest.h>
#include "ActorGraph.cpp"
#include "ActorGraph.hpp"

using namespace std;
using namespace testing;

// TODO: add tests for actor graph
/*
class SmallGraphFixture : public ::testing::Test {
  protected:
    ActorGraph ag;

  public:
    SmallGraphFixture() {
        const char* filename = "imdb.txt";
        ag.buildGraphFromFile(filename);
        string output = "(Bijan)--[Avengers#@2016]-->(Joey)";
        string shortestPath = "";
        ag.BFS("Bijan", "Joey", shortestPath);
        ASSERT_EQ(shortestPath, output);
    }
};
*/