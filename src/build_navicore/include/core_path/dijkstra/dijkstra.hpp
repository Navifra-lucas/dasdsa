#ifndef NAVIFRA_DIJKSTRA_ALGORITHM_HPP_
#define NAVIFRA_DIJKSTRA_ALGORITHM_HPP_

#include <bits/stdc++.h>

#include <functional>
#include <queue>

#define INF 0x3f3f3f3f
using namespace std;

typedef pair<int, float> weight_pair;
namespace NaviFra {
class Dijkstra {
    int n_vertex_num_;
    unique_ptr<list<weight_pair>[]> adj_;
    map<int, string> map_graph_;

    unique_ptr<vector<float>> vec_dist_;
    unique_ptr<vector<int>> vec_path_;

public:
    Dijkstra();
    /**
     * @brief Set the Node object (nate)
     *
     * @param num
     */
    void SetNode(int num);

    /**
     * @brief Add Edge(path) (nate)
     *
     * @param u start
     * @param v end
     * @param w weight
     */
    void AddEdge(int u, int v, float w);

    /**
     * @brief Set the Node Name object (nate)
     *
     * @param u number
     * @param v name
     */
    void SetName(int u, string v);

    /**
     * @brief Dijkstra algorithm start (nate)
     *
     * @param src start node
     */
    void FindShortestPath(int src);

    /**
     * @brief shortest path cost return (nate)
     *
     * @param s target node number
     * @return float totalcost(dist)
     */
    float CheckDist(int s);

    /**
     * @brief shortest path sqeunce make (nate)
     *
     * @param s
     * @param e
     * @param vec_path_num
     * @param cnt
     */
    void TracePath(int s, int e, vector<int>& vec_path_num, int& cnt);

    /**
     * @brief Get the shortest Path squence return (nate)
     *
     * @param s
     * @param e
     * @return vector<int>
     */
    vector<int> GetPath(int s, int e);
};
};  // namespace NaviFra
#endif