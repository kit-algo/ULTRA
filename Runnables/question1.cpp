#include <bits/stdc++.h>
using namespace std;

#include "../Algorithms/Dijkstra/Dijkstra.h"

// Function to load the graph from the florida graph file after attaching it in the repository.
GRAPH loadGraph(const string& filename) {
   ifstream file(filename);

    GRAPH graph;

    string line;
    while (getline(file, line)) {
        istringstream iss(line);

        // As mentioned in DIMACS, ignore comments, i.e., lines starting with 'c'
        if (line[0] == 'c') {
            continue;
        }

        // Parsing problem line, i.e., line starting with 'p'
        if (line[0] == 'p') {
            string problemType;
            int numVertices, numEdges;

            //for example: p sp 100 120
            iss >> problemType >> numVertices >> numEdges;

            //initialing the graph object with the  no. of vertices
            graph = GRAPH(numVertices);
            continue;
        }

        // Parse edge descriptors (lines starting with 'a')
        if (line[0] == 'a') {
            int u, v, weight;
            iss >> u >> v >> weight;

            graph.addEdge(u, v, weight);

            continue;
        }
    }

    file.close();

    return graph;
}

int main() {
    // Load the graph from the DIMACS file by calling the function
    GRAPH floridaGraph = loadGraph("USA-road-d.FLA.gr");  

    // Creating an instance of the Dijkstra class with the loaded graph
    Dijkstra<GRAPH> dijkstraInstance(floridaGraph);

    // Measuring the total runtime for 200 random Dijkstra runs
    clock_t start_time = clock();

    for (int i = 0; i < 200; ++i) {
        // Generate random source and destination nodes
        int source = rand() % floridaGraph.numVertices();
        int target = rand() % floridaGraph.numVertices();

        // Run Dijkstra's algorithm for the current source-destination pair
        dijkstraInstance.run(source, target);

        // Example: Print the shortest path and length for validation
        cout << "Shortest Path: ";
        auto path = dijkstraInstance.getPath(target);
        for (const auto& vertex : path) {
            cout << vertex << " ";
        }
        cout << "\nShortest path length: " << dijkstraInstance.getDistance(target) << "\n";
    }

    clock_t end_time = clock();

    // Calculate and print the total runtime in seconds
    double total_runtime = static_cast<double>(end_time - start_time) / CLOCKS_PER_SEC;
    cout << "Total runtime in seconds for 200 random Dijkstra: " << total_runtime << endl;

    return 0;
}
