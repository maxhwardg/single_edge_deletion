#include <vector>
#include <utility>

typedef std::vector<std::vector<double>> AdjMat;
typedef std::vector<std::vector<std::pair<int, double>>> AdjList;

typedef std::pair<int, int> Edge;

AdjList AdjListFromMat(const AdjMat& adj_mat);

AdjMat AdjMatFromList(const AdjList& adj_list);

std::pair<int, double> ConnectedAndSumDistances(AdjMat am);

std::pair<int, double> ConnectedAndSumDistances(Edge e, AdjMat am);