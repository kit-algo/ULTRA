// Submission by Ishika Gupta for Q2

#include <iostream>
#include <fstream>
#include <string>
#include <ModifiedDikstra.h>
#include <ctime>
#include <cstdlib>
using namespace std;

struct Edge {
    int u, v, w;
    Edge(int u, int v, int w)
    {
        u=this.u;
        v=this.v;
        w=this.w;
    } 
};

int main() {

    ifstream file("USA-road-d.FLA.gr");

    if (!file.is_open()) {
        cerr << "Error opening file." << endl;
        return 1;
    }

    string line;
    int numNodes, numEdges;
    vector<Edge> edges;

    while (getline(file, line)) 
    {
        if (line.empty() || line[0] == 'c') {
            continue;
        } else if (line[0] == 'p') {
            char type[3];
            sscanf(line.c_str(), "%2s %d %d", type, &numNodes, &numEdges);
        } else if (line[0] == 'a') {
            int u, v, w;
            sscanf(line.c_str(), "a %d %d %d", &u, &v, &w);
            edges.emplace_back(u, v, w);
        }
    }


    for (int i = 0; i < 200; ++i) {
        int source = rand() % numNodes + 1; 
        int destination = rand() % numNodes + 1;
        clock_t startTime = clock();

        DikstraModified.run(source, destination);

        double elapsedTime = static_cast<double>(clock() - startTime) / CLOCKS_PER_SEC;
        totalRuntime += elapsedTime;
    }

    cout << "Total runtime in seconds for 200 random for Modified Dijkstra Algo is : " << totalRuntime << endl;

    return 0;
}