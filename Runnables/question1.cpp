#include <iostream>
#include <cstdlib>
#include <ctime>

#include "../../Helpers/Meta.h"
#include "../../Helpers/Types.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Vector/Vector.h"

#include "../../DataStructures/Container/ExternalKHeap.h"
#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/Attributes/AttributeNames.h"

#include "../Algorithms/Dijkstra/Dijkstra.h"

using namespace std;

// the graph function will get the florida graph from the respective file which with thereafter help in selecting 200 random source-destination pairs
GRAPH inputfile(const string& abc) {
   ifstream file(abc);// to get the file

    GRAPH allocation;
    // now we dive into the line to get the simplified version about which statement needs to be cared with and or can be carried in the way
    string line;
    while (getline(file, line)) {
        istringstream instr(line);

        // lines staring with "c" are comments which will not affect the code
        if (line[0] == 'c') {
            continue;
        }

        // lines with "p" as their start are the problem statements which needs to be solved and get the shortest path
        if (line[0] == 'p') {
            string question_asked;
            int vert, edge;

            //for example: p sp 1070376 2712798 - given in file
            instr >> question_asked >> vert >> edge;

            //for number of vertices
            allocation = GRAPH(vert);
            continue;
        }

        // lines with "a" as their initial character we go to solve edge descriptors
        if (line[0] == 'a') {
            int u, v, w;
            instr >> u >> v >> w;
            allocation.addEdge(u, v, w);
            continue;
        }
    }

    file.close();

    return allocation;
}

int main() {
    // getting graph query by given DIMACS file
    GRAPH floridaGraph = inputfile("USA-road-d.FLA.gr");

    // Creating an instance of the Dijkstra class with the loaded graph
    Dijkstra<GRAPH> dijkstraInstance(floridaGraph);

    // Initializing the clock to measure the time
    clock_t start = clock();

    // Number of random source-destination pairs
    for (int i = 0; i < 200; ++i) {
        int source = rand() % floridaGraph.vert();
        int destination = rand() % floridaGraph.vert();

        // Running Dijkstra's algorithm for the current source-destination
        dijkstraInstance.run(source, destination);
        cout << "Shortest Path: "; // validation
        auto path = dijkstraInstance.getPath(destination);
        for (const auto& vertex : path) {
            cout << vertex ;
        }
        cout << "sp: " << dijkstraInstance.getDistance(destination);
    }

    clock_t endt = clock();

    // to get the total runtime in secs
    double runtime = static_cast<double>(endt - start) / CLOCKS_PER_SEC;
    cout << "Total runtime in seconds for 200 random Dijkstra: " << runtime << endl;


    return 0;
}
