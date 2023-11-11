#pragma once

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <fstream>
#include <unordered_map>

using namespace std;

// Data structure to represent an edge
struct Edge {
    int target;
    int weight;
};

unordered_map<int, vector<Edge>> readGraphFromFile(const string& filename) {
    unordered_map<int, vector<Edge>> graph;

    ifstream file(filename);

    if (file.is_open()) {
        int u, v, w;
        while (file >> u >> v >> w) {
            graph[u].push_back({v, w});
        }

        file.close();
    } else {
        cerr << "Unable to open file: " << filename << endl;
        exit(EXIT_FAILURE);
    }

    return graph;
}

int main() {
    unordered_map<int, vector<Edge>> graphFromFile = readGraphFromFile("USA-road-d.FLA");

    int source, target;

    cout << "Enter source node: ";
    cin >> source;

    cout << "Enter target node: ";
    cin >> target;

    int shortestPathLengthFromFile = dijkstra(graphFromFile, source, target);

    cout << "Shortest Path Length (Different Approach): " << shortestPathLengthFromFile << endl;

    return 0;
}


