/**
 * TODO: add file header
 */

#include "ActorGraph.hpp"
#include <math.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include "ActorNode.hpp"
#include "MovieNode.hpp"

using namespace std;

/* TODO */
ActorGraph::ActorGraph() {}

/* Build the actor graph from dataset file.
 * Each line of the dataset file must be formatted as:
 * ActorName <tab> MovieName <tab> Year
 * Two actors are connected by an undirected edge if they have worked in a movie
 * before.
 */
bool ActorGraph::buildGraphFromFile(const char* filename) {
    ifstream infile(filename);
    bool readHeader = false;

    while (infile) {
        string s;
        if (!getline(infile, s)) break;

        // skip the header of the file
        if (!readHeader) {
            readHeader = true;
            continue;
        }

        // read each line of the dataset to get the movie actor relation
        istringstream ss(s);
        vector<string> record;
        while (ss) {
            string str;
            if (!getline(ss, str, '\t')) break;
            record.push_back(str);
        }

        // if format is wrong, skip current line
        if (record.size() != 3) {
            continue;
        }

        // extract the information
        string actor(record[0]);
        string title(record[1]);
        int year = stoi(record[2]);

        // TODO: we have an actor/movie relationship to build the graph

        ActorNode* currActor = actorsMap[actor];  // checks for actor in map
        if (currActor == nullptr) {
            currActor = new ActorNode(actor);  // creates actor node
            actorsMap[actor] = currActor;      // sets key for actors map
        }

        // concatenate movie title and year into a single string
        string movieTitle = title + "#@" + to_string(year);

        MovieNode* currMovie = moviesMap[movieTitle];  // checks for movie in
                                                       // map
        if (currMovie == nullptr) {
            currMovie = new MovieNode(movieTitle);  // creates movie node
            moviesMap[movieTitle] = currMovie;      // sets key for movies map
        }

        currActor->movieList.push_back(
            currMovie);  // add current movie to list of actor's movies
        currMovie->actorsList.push_back(
            currActor);  // add current actor to the movie's actor list
    }

    // if failed to read the file, clear the graph and return
    if (!infile.eof()) {
        cerr << "Failed to read " << filename << endl;
        return false;
    }
    infile.close();

    return true;
}

/* TODO */
void ActorGraph::BFS(const string& fromActor, const string& toActor,
                     string& shortestPath) {
    queue<ActorNode*> explored;  // list to store nodes we have traversed
    queue<ActorNode*> prev;      // map to store all nodes that were traversed

    // if actors do not exist, return, don't change shortestPath, should be ""
    ActorNode* actor1 = actorsMap[fromActor];
    ActorNode* actor2 = actorsMap[toActor];
    ActorNode* curr;
    if (actor1 == nullptr || actor2 == nullptr) {
        return;
    }
    // if we are here, both actors exist
    explored.push(actor1);
    while (!explored.empty()) {
        ActorNode* curr = explored.front();
        if (curr == actor2) {
            break;  // we are done
        }
        curr->visited = true;
        explored.pop();

        // for all neighbors of curr
        for (int i = 0; i < curr->movieList.size(); i++) {
            MovieNode* currMovie = curr->movieList[i];
            for (int j = 0; j < currMovie->actorsList.size(); j++) {
                if (currMovie->actorsList[j] != curr) {  // avoid self loops
                    ActorNode* currActor = currMovie->actorsList[j];
                    if (!currActor->visited) {
                        currActor->prev = curr;
                        explored.push(currActor);
                        currActor->path = currMovie;
                    }
                }
            }
        }
    }
    if (curr != actor2) {  // no path exists
        return;
    }
    if (actor2->prev) {
        buildPath(actor2, shortestPath);
    }
    // if we are here then path does not exist
    // set all nodes back to unvisited
    for (auto i = actorsMap.begin(); i != actorsMap.end(); ++i) {
        i->second->visited = false;
        i->second->prev = 0;
        i->second->path = 0;
    }
}
void ActorGraph::buildPath(ActorNode* curr, string& shortestPath) {
    if (curr->prev == 0) {
        shortestPath += "(" + curr->actorName + ")";
        return;
    } else {
        buildPath(curr->prev, shortestPath);
        shortestPath +=
            "--[" + curr->path->title + "]-->(" + curr->actorName + ")";
    }
}

/* TODO */
void ActorGraph::predictLink(const string& queryActor,
                             vector<string>& predictionNames,
                             unsigned int numPrediction) {}

/* TODO */
ActorGraph::~ActorGraph() {
    for (auto it = actorsMap.begin(); it != actorsMap.end(); ++it) {
        delete it->second;
    }
    for (auto it = moviesMap.begin(); it != moviesMap.end(); ++it) {
        delete it->second;
    }
}
