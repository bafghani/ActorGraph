/**
 * TODO: add file header
 */

#include "ActorGraph.hpp"
#include <fstream>
#include <iostream>
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
        string movieTitle = title + to_string(year);

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
                     string& shortestPath) {}

/* TODO */
void ActorGraph::predictLink(const string& queryActor,
                             vector<string>& predictionNames,
                             unsigned int numPrediction) {}

/* TODO */
ActorGraph::~ActorGraph() {}
