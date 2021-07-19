#include "naive.hpp"
#include <limits>
#include <iostream>


using namespace std;

void FloydWarshall(AdjMat& M) {
    const int N = M.size();
    for (int k = 0; k < N; ++k)
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
                M[i][j] = min(M[i][j], M[i][k]+M[k][j]);
}

Edge FindWorstEdgeNaive(const AdjMat& adj_mat) {
    const int N = adj_mat.size();
    auto tmp_mat = adj_mat;
    int worst_connected = adj_mat.size()*adj_mat.size()+1;
    double worst_sum_shortest_paths = -numeric_limits<double>::infinity();
    Edge worst_edge{-1,-1};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) if (adj_mat[i][j] != numeric_limits<double>::infinity() && i != j) {
            tmp_mat = adj_mat;
            tmp_mat[i][j] = numeric_limits<double>::infinity();
            FloydWarshall(tmp_mat);
            int connceted_pairs = 0;
            double sum_shortest_paths = 0;
            for (int k = 0; k < N; ++k) {
                for (int l = 0; l < N; ++l) {
                    if (tmp_mat[k][l] != numeric_limits<double>::infinity() && k != l) {
                        ++connceted_pairs;
                        sum_shortest_paths += tmp_mat[k][l];
                    }
                }
            }
            if (connceted_pairs < worst_connected 
                    || connceted_pairs == worst_connected && sum_shortest_paths > worst_sum_shortest_paths) {
                worst_connected = connceted_pairs;
                worst_sum_shortest_paths = sum_shortest_paths;
                worst_edge = {i,j};
            }
        }
    }
    return worst_edge;
}