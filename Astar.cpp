#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <cmath>
#include <utility>
#include <iomanip>
using namespace std;

struct Node {
    int x, y;
    double g, f;
    vector<pair<int,int>> path;
    Node() : x(0), y(0), g(0), f(0) {}
    Node(int X, int Y, double G, double F, const vector<pair<int,int>>& P)
        : x(X), y(Y), g(G), f(F), path(P) {}
};

struct CompareNode {
    bool operator()(const Node& a, const Node& b) const {
        return a.f > b.f; // min-heap by f
    }
};

static inline double heuristic(const pair<int,int>& a, const pair<int,int>& b) {
    double dx = double(a.first  - b.first);
    double dy = double(a.second - b.second);
    return std::sqrt(dx*dx + dy*dy); // Euclidean
}

pair<vector<pair<int,int> >, double>
a_star(vector<vector<int> >& grid, pair<int,int> start, pair<int,int> goal) {
    int rows = (int)grid.size(), cols = (int)grid[0].size();

    // 8-directional moves: 4 straight (cost 1), 4 diagonal (cost sqrt(2))
    const int dirX[8] = {-1, 1, 0, 0, -1, -1,  1,  1};
    const int dirY[8] = { 0, 0,-1, 1, -1,  1, -1,  1};
    const double dirC[8] = {1, 1, 1, 1, std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)};

    priority_queue<Node, vector<Node>, CompareNode> open;
    set<pair<int,int> > visited;

    vector<pair<int,int> > initPath(1, start);
    open.push(Node(start.first, start.second, 0.0,
                   heuristic(start, goal), initPath));

    while (!open.empty()) {
        Node cur = open.top(); open.pop();
        pair<int,int> curPos(cur.x, cur.y);

        if (visited.count(curPos)) continue;
        visited.insert(curPos);

        if (curPos == goal) {
            return make_pair(cur.path, cur.g);
        }

        for (int k = 0; k < 8; ++k) {
            int nx = cur.x + dirX[k];
            int ny = cur.y + dirY[k];
            if (nx < 0 || ny < 0 || nx >= rows || ny >= cols) continue;
            if (grid[nx][ny] != 1) continue; // 1 = traversable, 0 = blocked

            pair<int,int> nxtPos(nx, ny);
            if (visited.count(nxtPos)) continue;

            double newG = cur.g + dirC[k];
            double newF = newG + heuristic(nxtPos, goal);

            vector<pair<int,int> > newPath = cur.path;
            newPath.push_back(nxtPos);

            open.push(Node(nx, ny, newG, newF, newPath));
        }
    }
    return make_pair(vector<pair<int,int> >(), -1.0); // no path
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    int n, m;
    cout << "Enter grid size (rows cols): ";
    if (!(cin >> n >> m)) return 0;

    vector<vector<int> > grid(n, vector<int>(m));
    cout << "Enter grid row by row (0 = blocked, 1 = free):\n";
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            cin >> grid[i][j];

    int sx, sy, gx, gy;
    cout << "Enter start position (row col): ";
    cin >> sx >> sy;
    cout << "Enter goal position (row col): ";
    cin >> gx >> gy;

    pair<int,int> start(sx, sy), goal(gx, gy);
    pair<vector<pair<int,int> >, double> res = a_star(grid, start, goal);

    if (res.second >= 0.0) {
        cout << "Path found:\n";
        for (size_t i = 0; i < res.first.size(); ++i) {
            cout << "(" << res.first[i].first << "," << res.first[i].second << ")"
                 << (i + 1 == res.first.size() ? "\n" : " ");
        }
        cout << "Minimum cost: " << fixed << setprecision(3) << res.second << "\n";
    } else {
        cout << "No path found.\n";
    }
    return 0;
}

























































































































































































































































































































































































































































































































