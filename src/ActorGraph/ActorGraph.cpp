/**
 * This file creates a graph where actors are vertices
 * and movies are edges. Actors are connected by an edge
 * if they both played in the same movie
 *
 * Authors: Bijan Afghani
 *          Joseph Mattingly
 */

#include "ActorGraph.hpp"
#include <bits/stdc++.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include "ActorNode.hpp"
#include "MovieNode.hpp"

using namespace std;

/* ActorGraph constructor */
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

        // we have an actor/movie relationship to build the graph

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
    // set all nodes back to unvisited
    for (auto i = actorsMap.begin(); i != actorsMap.end(); ++i) {
        i->second->visited = false;
        i->second->prev = 0;
        i->second->path = 0;
        i->second->dist = INT_MAX;
    }
    // if actors do not exist, return, don't change shortestPath, should be ""
    ActorNode* actor1 = actorsMap[fromActor];
    ActorNode* actor2 = actorsMap[toActor];

    if (actor1 == nullptr || actor2 == nullptr) {
        return;
    }
    ActorNode* currActor;
    ActorNode* neighbor;

    actor1->dist = 0;

    explored.push(actor1);

    while (!explored.empty()) {
        currActor = explored.front();

        if (currActor == actor2) {
            break;  // we are done
        }
        explored.pop();

        // for all neighbors of curr

        for (unsigned int i = 0; i < currActor->movieList.size(); i++) {
            MovieNode* currMovie =
                currActor->movieList[i];  // each of currActors Movies will be
                                          // stored here

            for (unsigned int j = 0; j < currMovie->actorsList.size(); j++) {
                if (currMovie->actorsList[j] !=
                    currActor) {  // avoid self loops

                    neighbor =
                        currMovie->actorsList[j];  // neighbor = an actor who
                                                   // played in the currentMovie
                    if (neighbor->dist ==
                        INT_MAX) {  // if neighbor hasnt been visited
                        neighbor->dist =
                            currActor->dist + 1;     // increment its distance
                        neighbor->prev = currActor;  // set its parent actor
                        neighbor->path =
                            currMovie;  // set last movie edge traversed
                        explored.push(neighbor);  // add to queue
                    }
                }
            }
        }
    }
    if (currActor != actor2) {  // no path exists
        return;
    }
    if (actor2->prev) {  // path exists
        buildPath(actor2, shortestPath);
    }
    // if we are here then path does not exist
}
void ActorGraph::buildPath(ActorNode* curr, string& shortestPath) {
    if (curr->prev == 0) {  // base case is we are at beginning of path, in this
                            // case no recursion necessary
        shortestPath += "(" + curr->actorName + ")";
        return;
    } else {
        // recursively builds the path in reverse by traversing each actors prev
        // actor
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
