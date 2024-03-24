#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <random>
#include <chrono> // For measuring runtime

using namespace std;

// Structure to represent an edge in the graph
struct Edge {
    int u, v, w; // Source, destination, weight
};

// Function to load the graph from a DIMACS file
vector<Edge> loadGraph(const string& filename) {
    vector<Edge> edges;
    ifstream file(filename);
    string line;

    if (file.is_open()) {
        while (getline(file, line)) {
            // Skip comments
            if (line[0] == 'c') continue;

            // Parse problem line to get number of nodes and edges
            if (line[0] == 'p') {
                int n, m;
                char type[10];
                sscanf(line.c_str(), "%s %*s %d %d", type, &n, &m);
                // Assuming the graph is already known to have 'm' edges
                edges.reserve(m);
            }

            // Parse edge descriptors
            if (line[0] == 'a') {
                int u, v, w;
                sscanf(line.c_str(), "%*s %d %d %d", &u, &v, &w);
                // Adjust node labels to match custom binary format
                u--; v--; 
                edges.push_back({u, v, w});
            }
        }
        file.close();
    } else {
        cerr << "Unable to open file: " << filename << endl;
    }

    return edges;
}

// Function to select 200 random source-destination pairs
vector<pair<int, int>> selectRandomPairs(const vector<Edge>& graph, int numPairs) {
    vector<pair<int, int>> randomPairs;
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<int> distribution(0, graph.size() - 1);

    for (int i = 0; i < numPairs; ++i) {
        int sourceIndex = distribution(gen);
        int destIndex = distribution(gen);
        randomPairs.push_back({graph[sourceIndex].u, graph[destIndex].v});
    }

    return randomPairs;
}

// Function to run Dijkstra's algorithm (assuming it's implemented elsewhere)
// This function should return the shortest path length
int dijkstra(const vector<Edge>& graph, int source, int destination) {
    // Assume Dijkstra's algorithm is implemented elsewhere in the repository
    // and can be called with appropriate parameters.
    // Replace this placeholder code with the actual function call.
    // Example: int shortestPathLength = runDijkstra(graph, source, destination);
    return 0; // Placeholder return value
}

int main() {
    // File path for the DIMACS graph
    string filename = "USA-road-d.FLA.co";

    // Load the graph
    vector<Edge> graph = loadGraph(filename);

    // Check if the graph loaded successfully
    if (graph.empty()) {
        cerr << "Failed to load graph." << endl;
        return 1;
    }

    // Select 200 random source-destination pairs
    int numPairs = 200;
    vector<pair<int, int>> randomPairs = selectRandomPairs(graph, numPairs);

    // Measure total runtime for all 200 runs
    auto start = chrono::steady_clock::now();
    for (const auto& pair : randomPairs) {
        int source = pair.first;
        int destination = pair.second;
        int shortestPathLength = dijkstra(graph, source, destination);
        // Do something with the shortest path length if needed
    }
    auto end = chrono::steady_clock::now();
    chrono::duration<double> runtime = end - start;

    // Print the total runtime in seconds
    cout << "Total runtime in seconds for 200 random Dijkstra: " << runtime.count() << endl;

    return 0;
}
