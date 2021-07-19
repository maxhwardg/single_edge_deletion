#include "faster.hpp"
#include "graph_utils.hpp"

#include <utility>
#include <vector>
#include <limits>
#include <queue>
#include <functional>
#include <map>
#include <iostream>
#include <set>

using namespace std;

typedef tuple<vector<double>, vector<int>, vector<int>> DistancesParentsVisited;

DistancesParentsVisited Dijkstra(const AdjList& adj, int source) {
    const int N = adj.size();
    vector<int> par(N, -1), visited;
    vector<double> dist(N, numeric_limits<double>::infinity());
    dist[source] = 0;
    priority_queue<pair<double,int>, vector<pair<double,int>>, greater<pair<double,int>>> pq;
    pq.emplace(0, source);
    while (pq.size()) {
        double d;
        int at;
        tie(d, at) = pq.top();
        pq.pop();
        if (d > dist[at]) continue;
        visited.push_back(at);
        for (auto& c : adj[at]) {
            double c_d = d+c.second;
            int c_node = c.first;
            if (dist[c_node] > c_d) {
                dist[c_node] = c_d;
                par[c_node] = at;
                pq.emplace(c_d, c_node);
            }
        }
    }
    return {dist, par, visited};
}

Edge FindWorstEdgeFaster(const AdjList& adj) {
    const int N = adj.size();
    auto tmp_adj = adj;
    vector<map<int, int>> edge_paths(N);
    vector<map<int, double>> edge_dists(N);
    vector<set<int>> tree_edges(N);
    for (int src = 0; src < N; ++src) {
        vector<double> dist;
        vector<int> par, vis;
        tie(dist, par, vis) = Dijkstra(adj, src);
        tree_edges.assign(N, set<int>());
        for (int node : vis) {
            if (par[node] == -1) {
                continue;
            }
            tree_edges[par[node]].insert(node);
            auto edges = tmp_adj[par[node]];
            int idx = -1;
            for (int i = 0; i < edges.size(); ++i) {
                if (edges[i].first == node) {
                    idx = i;
                }
            }
            tmp_adj[par[node]].erase(tmp_adj[par[node]].begin()+idx);
            vector<double> e_dist;
            vector<int> e_par, e_vis;
            tie(e_dist, e_par, e_vis) = Dijkstra(tmp_adj, src);
            tmp_adj[par[node]] = edges;

            int paths = e_vis.size()-1;
            double sum_dists = 0;

            for (int e_node : e_vis) {
                sum_dists += e_dist[e_node];
            }
            edge_paths[par[node]][node] += paths;
            edge_dists[par[node]][node] += sum_dists;
            
        }
        double sum_dists = 0;
        for (int v = 0; v < N; ++v) {
            if (dist[v] != numeric_limits<double>::infinity())
                sum_dists += dist[v];
        }
        for (int u = 0; u < N; ++u) {
            for (const auto& e : adj[u]) {
                if (!tree_edges[u].count(e.first)) {
                    edge_paths[u][e.first] += vis.size()-1;
                    edge_dists[u][e.first] += sum_dists;
                }
            }
        }
    }
    double worst_dists = -numeric_limits<double>::infinity();
    int worst_paths = N*N+5;
    Edge worst_edge{-1,-1};
    for (int node = 0; node < N; ++node) {
        for (const auto& node_dist : edge_dists[node]) {
            int paths = edge_paths[node][node_dist.first];
            if (paths < worst_paths || worst_paths == paths && worst_dists < node_dist.second) {
                worst_paths = paths;
                worst_dists = node_dist.second;
                worst_edge = {node, node_dist.first};
            }
        }
    }
    return worst_edge;
}

vector<Edge> FindWorstEdgesFaster(const AdjList& adj) {
    const int N = adj.size();
    auto tmp_adj = adj;
    vector<map<int, int>> edge_paths(N);
    vector<map<int, double>> edge_dists(N);
    vector<set<int>> tree_edges(N);
    for (int src = 0; src < N; ++src) {
        vector<double> dist;
        vector<int> par, vis;
        tie(dist, par, vis) = Dijkstra(adj, src);
        tree_edges.assign(N, set<int>());
        for (int node : vis) {
            if (par[node] == -1) {
                continue;
            }
            tree_edges[par[node]].insert(node);
            auto edges = tmp_adj[par[node]];
            int idx = -1;
            for (int i = 0; i < edges.size(); ++i) {
                if (edges[i].first == node) {
                    idx = i;
                }
            }
            tmp_adj[par[node]].erase(tmp_adj[par[node]].begin()+idx);
            vector<double> e_dist;
            vector<int> e_par, e_vis;
            tie(e_dist, e_par, e_vis) = Dijkstra(tmp_adj, src);
            tmp_adj[par[node]] = edges;

            int paths = e_vis.size()-1;
            double sum_dists = 0;

            for (int e_node : e_vis) {
                sum_dists += e_dist[e_node];
            }
            edge_paths[par[node]][node] += paths;
            edge_dists[par[node]][node] += sum_dists;
            
        }
        double sum_dists = 0;
        for (int v = 0; v < N; ++v) {
            if (dist[v] != numeric_limits<double>::infinity())
                sum_dists += dist[v];
        }
        for (int u = 0; u < N; ++u) {
            for (const auto& e : adj[u]) {
                if (!tree_edges[u].count(e.first)) {
                    edge_paths[u][e.first] += vis.size()-1;
                    edge_dists[u][e.first] += sum_dists;
                }
            }
        }
    }
    double worst_dists = -numeric_limits<double>::infinity();
    int worst_paths = N*N+5;
    vector<Edge> worst_edges;
    for (int node = 0; node < N; ++node) {
        for (const auto& node_dist : edge_dists[node]) {
            int paths = edge_paths[node][node_dist.first];
            if (paths < worst_paths || worst_paths == paths && worst_dists < node_dist.second) {
                worst_paths = paths;
                worst_dists = node_dist.second;
                worst_edges = {{node, node_dist.first}};
            } else if (paths == worst_paths && node_dist.second == worst_dists) {
                worst_edges.emplace_back(node, node_dist.first);
            }
        }
    }
    return worst_edges;
}