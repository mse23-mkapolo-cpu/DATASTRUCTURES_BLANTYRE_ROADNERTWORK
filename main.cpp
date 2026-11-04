
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <iostream>
#include <map>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
using namespace std;

const long long INF = 1000000000000000000LL;

// ---------------------------------------------------------------------
// 1. THE ROAD DATA: "Place A,Place B,distance in metres"
// ---------------------------------------------------------------------
const string ROAD_DATA =
    "Limbe Market,Limbe Health Centre,450\n"
    "Limbe Market,Liquid Lounge Roundabout,700\n"
    "Limbe Market,Yanakis Roundabout,1200\n"
    "Limbe Health Centre,Liquid Lounge Roundabout,350\n"
    "Limbe Health Centre,St Columba,3200\n"
    "Liquid Lounge Roundabout,Puma Energy Kanjedza,1000\n"
    "Liquid Lounge Roundabout,MASM,550\n"
    "MASM,Yanakis Roundabout,1000\n"
    "Yanakis Roundabout,Reunion Insurance Roundabout,1100\n"
    "Reunion Insurance Roundabout,Independence Arch,1000\n"
    "Puma Energy Kanjedza,Automotive Products Limited,1600\n"
    "Puma Energy Kanjedza,Independence Arch,2900\n"
    "Automotive Products Limited,St Columba,850\n"
    "Automotive Products Limited,Kamba Pork Market,2500\n"
    "Automotive Products Limited,Chichiri Shopping Mall,1500\n"
    "Independence Arch,Chichiri Shopping Mall,2900\n"
    "St Columba,Stereo Night Club,3100\n"
    "Kamba Pork Market,Stereo Night Club,300\n"
    "Kamba Pork Market,Red Cross Malawi,1200\n"
    "Stereo Night Club,Rosewood Mall,700\n"
    "Rosewood Mall,CI Primary School,1600\n"
    "Rosewood Mall,Red Cross Malawi,1400\n"
    "Red Cross Malawi,Mandala,550\n"
    "Red Cross Malawi,Chichiri Shopping Mall,2300\n"
    "Red Cross Malawi,Blantyre Clock Tower,3400\n"
    "Mandala,Blantyre Market,210\n"
    "CI Primary School,Blantyre Market,300\n"
    "Blantyre Market,PA Nandos,1200\n"
    "PA Nandos,Blantyre Clock Tower,1200\n"
    "Chichiri Shopping Mall,Castel,1900\n"
    "Castel,HHI,1400\n"
    "HHI,Blantyre Clock Tower,2500\n";

// ---------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------
string toStr(long long n) {
    ostringstream out;
    out << n;
    return out.str();
}

string km(long long metres) {
    char buf[32];
    sprintf(buf, "%.2f km", metres / 1000.0);
    return string(buf);
}

// ---------------------------------------------------------------------
// 2. THE GRAPH (adjacency list)
// ---------------------------------------------------------------------
struct Edge {
    int u, v;          // the two places joined by the road
    long long w;       // length in metres
};

struct Neighbour {
    int place;
    long long w;
};

struct Graph {
    map<string, int> id;                   // place name -> number
    vector<string> name;                   // number -> place name
    vector< vector<Neighbour> > adj;       // adj[u] = roads leaving u
    vector<Edge> edges;                    // every road once

    int addPlace(const string& p) {
        if (p.empty()) throw invalid_argument("empty place name");
        if (id.count(p)) return id[p];
        int n = (int)name.size();
        id[p] = n;
        name.push_back(p);
        adj.push_back(vector<Neighbour>());
        return n;
    }

    void addRoad(const string& a, const string& b, long long w) {
        if (w < 0)  throw invalid_argument("negative road length: " + a + " - " + b);
        if (a == b) throw invalid_argument("self-loop not allowed: " + a);
        int u = addPlace(a);
        int v = addPlace(b);
        Neighbour toV = { v, w };
        Neighbour toU = { u, w };
        adj[u].push_back(toV);             // undirected: store both directions
        adj[v].push_back(toU);
        Edge e = { u, v, w };
        edges.push_back(e);
    }

    int getId(const string& p) const {
        map<string, int>::const_iterator it = id.find(p);
        if (it == id.end()) throw invalid_argument("unknown place: " + p);
        return it->second;
    }

    int V() const { return (int)name.size(); }
    int E() const { return (int)edges.size(); }
};

// Read roads from text, one per line: "Place A,Place B,metres"
Graph loadGraph(const string& text) {
    Graph g;
    stringstream in(text);
    string line;
    int lineNo = 0;
    while (getline(in, line)) {
        lineNo++;
        if (line.empty()) continue;
        stringstream ss(line);
        string a, b, w;
        if (!getline(ss, a, ',') || !getline(ss, b, ',') || !getline(ss, w))
            throw invalid_argument("bad format on line " + toStr(lineNo));

        // convert the distance text to a number, checking every character
        long long metres = 0;
        bool negative = false;
        for (size_t i = 0; i < w.size(); i++) {
            if (i == 0 && w[i] == '-') { negative = true; continue; }
            if (!isdigit((unsigned char)w[i]))
                throw invalid_argument("bad distance on line " + toStr(lineNo));
            metres = metres * 10 + (w[i] - '0');
        }
        if (w.empty() || w == "-") throw invalid_argument("bad distance on line " + toStr(lineNo));
        if (negative) metres = -metres;

        g.addRoad(a, b, metres);
    }
    return g;
}

// ---------------------------------------------------------------------
// 3. DIJKSTRA'S ALGORITHM (binary heap)
// ---------------------------------------------------------------------
struct QueueItem {
    long long dist;
    int place;
};
// makes priority_queue give the SMALLEST distance first
struct SmallerFirst {
    bool operator()(const QueueItem& a, const QueueItem& b) const {
        if (a.dist != b.dist) return a.dist > b.dist;
        return a.place > b.place;
    }
};

// Returns the shortest distance from src to every place; prev[] stores
// the previous place on each shortest route.
vector<long long> dijkstra(const Graph& g, int src, vector<int>& prev, bool showSteps) {
    vector<long long> dist(g.V(), INF);
    prev.assign(g.V(), -1);
    priority_queue<QueueItem, vector<QueueItem>, SmallerFirst> pq;

    dist[src] = 0;
    QueueItem start = { 0, src };
    pq.push(start);
    int step = 0;

    while (!pq.empty()) {
        QueueItem top = pq.top();
        pq.pop();
        int u = top.place;
        if (top.dist > dist[u]) continue;              // stale entry: ignore

        if (showSteps)
            cout << "Step " << ++step << ": settle " << g.name[u] << " (" << km(dist[u]) << ")\n";

        for (size_t i = 0; i < g.adj[u].size(); i++) {
            int v = g.adj[u][i].place;
            long long w = g.adj[u][i].w;
            if (dist[u] + w < dist[v]) {               // relaxation
                if (showSteps)
                    cout << "     update " << g.name[v] << ": "
                         << (dist[v] == INF ? string("inf") : km(dist[v]))
                         << " -> " << km(dist[u] + w) << "\n";
                dist[v] = dist[u] + w;
                prev[v] = u;
                QueueItem item = { dist[v], v };
                pq.push(item);
            }
        }
    }
    return dist;
}

// Shortest route between two named places. Returns the distance (INF if none)
// and fills 'route' with the places in order.
long long shortestRoute(const Graph& g, const string& from, const string& to,
                        vector<string>& route, bool showSteps = false) {
    int s = g.getId(from);
    int t = g.getId(to);
    vector<int> prev;
    vector<long long> dist = dijkstra(g, s, prev, showSteps);
    route.clear();
    if (dist[t] == INF) return INF;
    for (int x = t; x != -1; x = prev[x]) route.push_back(g.name[x]);
    reverse(route.begin(), route.end());
    return dist[t];
}

// ---------------------------------------------------------------------
// 4. KRUSKAL'S ALGORITHM (sorted edges + union-find)
// ---------------------------------------------------------------------
vector<int> parent;

int findSet(int x) {
    while (parent[x] != x) {
        parent[x] = parent[parent[x]];                 // path compression
        x = parent[x];
    }
    return x;
}

bool shorterRoad(const Edge& a, const Edge& b) { return a.w < b.w; }

vector<Edge> kruskal(const Graph& g) {
    vector<Edge> sorted = g.edges;
    stable_sort(sorted.begin(), sorted.end(), shorterRoad);
    parent.resize(g.V());
    for (int i = 0; i < g.V(); i++) parent[i] = i;

    vector<Edge> tree;
    for (size_t i = 0; i < sorted.size(); i++) {
        int a = findSet(sorted[i].u);
        int b = findSet(sorted[i].v);
        if (a == b) continue;                          // would make a cycle
        parent[a] = b;
        tree.push_back(sorted[i]);
        if ((int)tree.size() == g.V() - 1) break;
    }
    return tree;
}

// ---------------------------------------------------------------------
// 5. PRIM'S ALGORITHM (binary heap)
// ---------------------------------------------------------------------
struct PrimItem {
    long long w;
    int from, to;
};
struct LighterFirst {
    bool operator()(const PrimItem& a, const PrimItem& b) const {
        if (a.w != b.w) return a.w > b.w;
        return a.to > b.to;
    }
};

vector<Edge> prim(const Graph& g, int start) {
    vector<bool> inTree(g.V(), false);
    vector<Edge> tree;
    priority_queue<PrimItem, vector<PrimItem>, LighterFirst> pq;

    inTree[start] = true;
    for (size_t i = 0; i < g.adj[start].size(); i++) {
        PrimItem it = { g.adj[start][i].w, start, g.adj[start][i].place };
        pq.push(it);
    }

    while (!pq.empty() && (int)tree.size() < g.V() - 1) {
        PrimItem top = pq.top();
        pq.pop();
        if (inTree[top.to]) continue;
        inTree[top.to] = true;
        Edge e = { top.from, top.to, top.w };
        tree.push_back(e);
        for (size_t i = 0; i < g.adj[top.to].size(); i++) {
            int x = g.adj[top.to][i].place;
            if (!inTree[x]) {
                PrimItem it = { g.adj[top.to][i].w, top.to, x };
                pq.push(it);
            }
        }
    }
    return tree;
}

long long totalLength(const vector<Edge>& es) {
    long long sum = 0;
    for (size_t i = 0; i < es.size(); i++) sum += es[i].w;
    return sum;
}

// Graph made of only some roads (used to measure a route inside the MST)
Graph subgraph(const Graph& g, const vector<Edge>& es) {
    Graph h;
    for (int i = 0; i < g.V(); i++) h.addPlace(g.name[i]);
    for (size_t i = 0; i < es.size(); i++)
        h.addRoad(g.name[es[i].u], g.name[es[i].v], es[i].w);
    return h;
}

// ---------------------------------------------------------------------
// 6. AUTOMATED TESTS
// ---------------------------------------------------------------------
int passed = 0, failed = 0;

void check(int n, const string& test, const string& expected, const string& actual) {
    bool ok = (expected == actual);
    if (ok) passed++; else failed++;
    cout << "T" << n << " | " << test << " | expected: " << expected
         << " | actual: " << actual << " | " << (ok ? "PASS" : "FAIL") << "\n";
}

// shortest distance as text, e.g. "7.76 km" or "no route"
string distText(const Graph& g, const string& a, const string& b) {
    vector<string> r;
    long long d = shortestRoute(g, a, b, r);
    return d == INF ? string("no route") : km(d);
}

// load bad road data and return the error message it causes
string errorFrom(const string& roads) {
    try { loadGraph(roads); return "no error"; }
    catch (exception& e) { return string("error: ") + e.what(); }
}

string join(const vector<string>& route) {
    string s;
    for (size_t i = 0; i < route.size(); i++) s += (i ? " > " : "") + route[i];
    return s;
}

void runTests() {
    passed = failed = 0;
    Graph g = loadGraph(ROAD_DATA);
    vector<string> r;

    check(1, "Number of places loaded", "22", toStr(g.V()));
    check(2, "Number of roads loaded", "32", toStr(g.E()));
    check(3, "Total road length", "46.06 km", km(totalLength(g.edges)));
    check(4, "Limbe Market to Blantyre Market", "7.76 km", distText(g, "Limbe Market", "Blantyre Market"));
    shortestRoute(g, "Limbe Market", "Blantyre Market", r);
    check(5, "Route Limbe Market to Blantyre Market",
          "Limbe Market > Liquid Lounge Roundabout > Puma Energy Kanjedza > Automotive Products Limited > "
          "Kamba Pork Market > Red Cross Malawi > Mandala > Blantyre Market", join(r));
    check(6, "Same start and destination (Mandala to Mandala)", "0.00 km", distText(g, "Mandala", "Mandala"));
    check(7, "Direct road Mandala to Blantyre Market", "0.21 km", distText(g, "Mandala", "Blantyre Market"));
    check(8, "Reverse journey Blantyre Market to Limbe Market", "7.76 km", distText(g, "Blantyre Market", "Limbe Market"));
    check(9, "Red Cross Malawi to Blantyre Clock Tower (beats 3.40 km direct road)", "3.16 km",
          distText(g, "Red Cross Malawi", "Blantyre Clock Tower"));
    check(10, "Limbe Market to Blantyre Clock Tower (farthest place)", "10.16 km",
          distText(g, "Limbe Market", "Blantyre Clock Tower"));
    check(11, "Limbe Market to St Columba (via Limbe Health Centre)", "3.65 km", distText(g, "Limbe Market", "St Columba"));
    check(12, "Limbe Market to CI Primary School", "8.06 km", distText(g, "Limbe Market", "CI Primary School"));

    string err;
    try { distText(g, "Limbe Market", "Zomba"); err = "no error"; }
    catch (exception& e) { err = string("error: ") + e.what(); }
    check(13, "Unknown place name 'Zomba'", "error: unknown place: Zomba", err);
    check(14, "Negative road length", "error: negative road length: A - B", errorFrom("A,B,-5\n"));
    check(15, "Line with missing distance", "error: bad format on line 1", errorFrom("A,B\n"));
    check(16, "Non-numeric distance", "error: bad distance on line 1", errorFrom("A,B,12km\n"));
    check(17, "Blank place name", "error: empty place name", errorFrom(",B,100\n"));
    check(18, "Self-loop road", "error: self-loop not allowed: A", errorFrom("A,A,100\n"));

    Graph h1 = loadGraph("A,B,100\nC,D,200\n");
    check(19, "Unreachable place (A to D in two separate parts)", "no route", distText(h1, "A", "D"));
    Graph h2 = loadGraph("A,B,0\nB,C,300\n");
    check(20, "Zero-length road is accepted", "0.30 km", distText(h2, "A", "C"));
    Graph h3 = loadGraph("A,B,900\nA,B,400\n");
    check(21, "Parallel roads: shorter one is used", "0.40 km", distText(h3, "A", "B"));

    vector<Edge> mst = kruskal(g);
    check(22, "Kruskal MST number of edges", "21", toStr((long long)mst.size()));
    check(23, "Kruskal MST total length", "20.66 km", km(totalLength(mst)));
    bool same = true;
    for (int s = 0; s < g.V(); s++) {
        vector<Edge> p = prim(g, s);
        if (p.size() != 21 || totalLength(p) != totalLength(mst)) same = false;
    }
    check(24, "Prim from all 22 start places equals Kruskal", "true", same ? "true" : "false");
    Graph t = subgraph(g, mst);
    check(25, "Limbe Market to Blantyre Market inside the MST", "7.96 km",
          distText(t, "Limbe Market", "Blantyre Market"));

    cout << "\n" << passed << " passed, " << failed << " failed\n";
}

// ---------------------------------------------------------------------
// 7. MAIN PROGRAM (menu)
// ---------------------------------------------------------------------
int main(int argc, char* argv[]) {
    Graph g = loadGraph(ROAD_DATA);

    if (argc > 1 && string(argv[1]) == "test") {
        runTests();
        return failed == 0 ? 0 : 1;
    }

    cout << "Blantyre road network: " << g.V() << " places, " << g.E()
         << " roads, total " << km(totalLength(g.edges)) << "\n";

    while (true) {
        cout << "\n1. Shortest route between two places\n"
             << "2. Shortest route with Dijkstra steps shown\n"
             << "3. Minimum spanning tree (Kruskal)\n"
             << "4. Run the 25 automated tests\n"
             << "5. List all places\n"
             << "0. Exit\n"
             << "Choice: ";
        string choice;
        if (!getline(cin, choice) || choice == "0") break;

        try {
            if (choice == "1" || choice == "2") {
                string from, to;
                cout << "From: "; getline(cin, from);
                cout << "To:   "; getline(cin, to);
                vector<string> route;
                long long d = shortestRoute(g, from, to, route, choice == "2");
                if (d == INF) cout << "No route exists.\n";
                else {
                    cout << "Route: ";
                    for (size_t i = 0; i < route.size(); i++) cout << (i ? " -> " : "") << route[i];
                    cout << "\nDistance: " << km(d) << "\n";
                }
            } else if (choice == "3") {
                vector<Edge> mst = kruskal(g);
                for (size_t i = 0; i < mst.size(); i++)
                    cout << "  " << g.name[mst[i].u] << " - " << g.name[mst[i].v]
                         << "  " << km(mst[i].w) << "\n";
                cout << "Roads: " << mst.size() << "   Total: " << km(totalLength(mst)) << "\n";
            } else if (choice == "4") {
                runTests();
            } else if (choice == "5") {
                for (int i = 0; i < g.V(); i++) cout << "  " << i + 1 << ". " << g.name[i] << "\n";
            } else {
                cout << "Please choose 0-5.\n";
            }
        } catch (exception& e) {
            cout << "Error: " << e.what() << "\n";
        }
    }
    return 0;
}
