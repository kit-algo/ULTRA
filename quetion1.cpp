#include <iostream>
#include <fstream>
#include <ctime>
#include "dijkstra.h"
using namespace std;
int main() {
    ifstream file("florida.dimacs");
    int n, m;
    while (file >> ws && file.peek() == 'c') file.ignore(4096, '\n');
    file.ignore(2); file.ignore(3); file >> n >> m;
    clock_t start = clock();
    for (int i = 0; i < 200; ++i) {
        int source = rand() % n;
        int target = rand() % n;
        dijkstra(source, target);
    }
    clock_t end = clock();
    double total_runtime = double(end - start) / CLOCKS_PER_SEC;
    cout << "Total runtime in seconds for 200 random Dijkstra: " << total_runtime << endl;
    return 0;
}

