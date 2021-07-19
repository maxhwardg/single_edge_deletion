#include "faster.hpp"

#include <iostream>

using namespace std;

int main() {
    cout << "Enter the number of vertices in the graph." << endl;
    int V, E;
    cin >> V;
    cout << "Enter the number of edges in the graph." << endl;
    cin >> E;
    cout << "Enter the edges of the graph. Enter each edge on its own "
            "line as a triple of space separated integers representing the source vertex,"
            "destination vertex, and edge weight. "
            "Source and destination should be indexes in the inclusive range from 0 to V-1. "
            "Weights should be positive. "
            "Technically, negative weights are possible to handle, it just isn't implemented yet. "
            "An example input containing 3 edges: " << endl <<
            "0 1 2" << endl <<
            "1 2 3" << endl <<
            "2 0 1" << endl <<
            "This graph is a directed cycle of 3 vertices 0 -> 1 -> 2 -> 0. "
            "Where the weights (around the cycle) are 2, 3, and 1."
         << endl;
    AdjList al(V);
    for (int i = 0; i < E; i++) {
        int s, d, w;
        cin >> s >> d >> w;
        al[s].emplace_back(d, w);
    }
    auto edges = FindWorstEdgesFaster(al);
    cout << "The worst edges to delete from the graph are: " << endl;
    for (auto& edge : edges) {
        cout << edge.first << " " << edge.second << endl;
    }
}