#include "graph_utils.hpp"
#include "naive.hpp"

#include <limits>

using namespace std;

AdjList AdjListFromMat(const AdjMat& adj_mat) {
    const int N = adj_mat.size();
    AdjList al(N);
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (adj_mat[i][j] == numeric_limits<double>::infinity())
                continue;
            al[i].emplace_back(j, adj_mat[i][j]);
        }
    }
    return al;
}

AdjMat AdjMatFromList(const AdjList& adj_list) {
    AdjMat mat(adj_list.size(), std::vector<double>(adj_list.size(), numeric_limits<double>::infinity()));
    for (int i = 0; i < mat.size(); ++i) mat[i][i] = 0.0;
    for (int u = 0; u < mat.size(); ++u) {
        for (const auto& e : adj_list[u]) {
            mat[u][e.first] = e.second;
        }
    }
    return mat;

}

pair<int, double> ConnectedAndSumDistances(AdjMat am) {
    FloydWarshall(am);
    int paths = 0;
    double sum_dists = 0;
    for (int i = 0; i < am.size(); ++i) {
        for (int j = 0; j < am.size(); ++j) {
            if (am[i][j] == numeric_limits<double>::infinity() || i == j)
                continue;
            ++paths;
            sum_dists += am[i][j];
        }
    }
    return {paths, sum_dists};
}

pair<int, double> ConnectedAndSumDistances(Edge e, AdjMat am) {
    am[e.first][e.second] = numeric_limits<double>::infinity();
    return ConnectedAndSumDistances(am);
}