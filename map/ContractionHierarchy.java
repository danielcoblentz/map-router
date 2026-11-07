/*************************************************************************
 *  Contraction hierarchies.
 *
 *  Preprocessing contracts the vertices one at a time in order of
 *  importance. Contracting v removes it from the remaining graph and, for
 *  every pair of neighbours u, w whose shortest path ran through v, adds a
 *  shortcut edge u-w of length d(u,v) + d(v,w). A bounded local Dijkstra,
 *  the witness search, decides whether some other path is at least as
 *  short, in which case no shortcut is needed.
 *
 *  Every edge, original or shortcut, is kept only at its lower-ranked
 *  endpoint, so each vertex stores just its upward edges. A query runs
 *  Dijkstra upward from s and upward from d at the same time and the
 *  answer is the best vertex both searches reach. Because the graph is
 *  undirected the same upward graph serves both directions.
 *
 *  Each shortcut remembers the vertex it bypasses, so a path made of
 *  shortcuts is unpacked recursively into original road segments.
 *
 *************************************************************************/

import java.awt.Color;

public class ContractionHierarchy {
    private static final double INFINITY = Double.MAX_VALUE;

    // A witness search settles at most this many vertices before giving up.
    // Giving up early only adds a shortcut that was not strictly needed, so
    // the limits trade preprocessing time against query time, never
    // correctness. Estimating a vertex's priority only needs a rough count,
    // so it uses a tighter limit than the contraction that commits shortcuts.
    // On usa.txt these settings took preprocessing from 22 s (500 for both,
    // edge difference weighted 1) to about 5 s with no loss in query speed.
    private static final int WITNESS_SETTLE_LIMIT = 200;
    private static final int SIMULATE_SETTLE_LIMIT = 20;
    private static final int EDGE_DIFFERENCE_WEIGHT = 2;

    private final EuclideanGraph G;
    private final int V;
    private final int[] rank;          // contraction order, 0 = first contracted

    // upward graph in compressed sparse row form: the upward edges of v are
    // upTo/upW/upMid[upFirst[v] .. upFirst[v+1] - 1]
    private final int[] upFirst;
    private final int[] upTo;
    private final double[] upW;
    private final int[] upMid;         // bypassed vertex, or -1 for a road
    private final int shortcutCount;

    // per-query state, lazily reset with a query stamp as in Dijkstra
    private final double[] distF, distB;
    private final int[] predEdgeF, predEdgeB;   // upward edge used to reach v
    private final int[] seenF, seenB;
    private final IndexPQ pqF, pqB;
    private int queryId = 0;
    private int lastS = -1, lastD = -1;
    private double best;
    private int meet;
    private int lastVisitedCount;

    public ContractionHierarchy(EuclideanGraph G) {
        this.G = G;
        this.V = G.V();
        rank = new int[V];

        Preprocessor pre = new Preprocessor(G);
        Adj[] up = pre.run(rank);
        shortcutCount = pre.shortcuts;

        upFirst = new int[V + 1];
        for (int v = 0; v < V; v++) upFirst[v + 1] = upFirst[v] + up[v].n;
        int m = upFirst[V];
        upTo = new int[m];
        upW = new double[m];
        upMid = new int[m];
        for (int v = 0; v < V; v++) {
            int base = upFirst[v];
            for (int i = 0; i < up[v].n; i++) {
                upTo[base + i]  = up[v].to[i];
                upW[base + i]   = up[v].w[i];
                upMid[base + i] = up[v].mid[i];
            }
        }

        distF = new double[V];
        distB = new double[V];
        predEdgeF = new int[V];
        predEdgeB = new int[V];
        seenF = new int[V];
        seenB = new int[V];
        pqF = new IndexPQ(V);
        pqB = new IndexPQ(V);

        // preprocessing ran its own queues; don't let that leak into the
        // per-query counters that Paths reports
        IndexPQ.insertCount = 0;
        IndexPQ.changeCount = 0;
        IndexPQ.delMinCount = 0;
        IndexPQ.maxPQSize = 0;
    }

    // number of shortcut edges added by preprocessing
    public int shortcutCount() { return shortcutCount; }

    // number of edges in the upward graph, roads and shortcuts together
    public int upwardEdgeCount() { return upFirst[V]; }

    // position of v in the contraction order
    public int rank(int v) { return rank[v]; }

    // vertices settled by both searches in the last query
    public int visitedCount() { return lastVisitedCount; }

    // run one query and cache the result for showPath, drawPath and path
    public void compute(int s, int d) { query(s, d); }

    // shortest path distance from s to d, or Double.MAX_VALUE if unreachable
    public double distance(int s, int d) {
        query(s, d);
        return best;
    }

    // vertices of the last computed path from s to d, or null if unreachable
    public int[] path() {
        if (meet < 0) return null;
        IntList out = new IntList();
        out.add(lastS);

        // s up to the meeting vertex: collect the forward chain, then replay
        // it from the s end
        IntList chain = new IntList();
        for (int v = meet; v != lastS; v = owner(predEdgeF[v])) chain.add(v);
        for (int i = chain.n - 1; i >= 0; i--) {
            int v = chain.a[i];
            int e = predEdgeF[v];
            expand(owner(e), v, upMid[e], out);
        }

        // meeting vertex down to d: the backward chain is already in order
        for (int v = meet; v != lastD; ) {
            int e = predEdgeB[v];
            int next = owner(e);
            expand(v, next, upMid[e], out);
            v = next;
        }
        return out.toArray();
    }

    // print the last path in the same d-...-s format Dijkstra uses
    public void showPath(int d, int s) {
        query(s, d);
        int[] p = path();
        if (p == null) {
            System.out.println("No path from " + s + " to " + d);
            return;
        }
        StringBuilder sb = new StringBuilder();
        for (int i = p.length - 1; i > 0; i--) sb.append(p[i]).append('-');
        sb.append(p[0]);
        System.out.println(sb);
    }

    // draw the last path using Turtle graphics
    public void drawPath(int s, int d) {
        if (s != lastS || d != lastD) query(s, d);
        int[] p = path();
        if (p == null) return;
        Turtle.setColor(Color.red);
        for (int i = 1; i < p.length; i++)
            G.point(p[i - 1]).drawTo(G.point(p[i]));
        Turtle.render();
    }

    /*********************************************************************
     *  Query: bidirectional Dijkstra on the upward graph.
     *********************************************************************/

    private void query(int s, int d) {
        queryId++;
        lastS = s;
        lastD = d;
        best = INFINITY;
        meet = -1;
        pqF.clear();
        pqB.clear();

        distF[s] = 0.0; seenF[s] = queryId; predEdgeF[s] = -1; pqF.insert(s, 0.0);
        distB[d] = 0.0; seenB[d] = queryId; predEdgeB[d] = -1; pqB.insert(d, 0.0);

        int visited = 0;
        boolean forwardDone = false, backwardDone = false;
        while (!forwardDone || !backwardDone) {
            if (!forwardDone) {
                int v = step(pqF, distF, predEdgeF, seenF, distB, seenB);
                if (v < 0) forwardDone = true; else visited++;
            }
            if (!backwardDone) {
                int v = step(pqB, distB, predEdgeB, seenB, distF, seenF);
                if (v < 0) backwardDone = true; else visited++;
            }
        }
        lastVisitedCount = visited;
    }

    // settle one vertex on one side; returns it, or -1 once this side can
    // no longer improve on the best meeting point found so far
    private int step(IndexPQ pq, double[] dist, int[] predEdge, int[] seen,
                     double[] otherDist, int[] otherSeen) {
        if (pq.isEmpty()) return -1;
        int v = pq.delMin();
        // every vertex still queued is at least this far away, so nothing on
        // this side can produce a shorter s-d path
        if (dist[v] >= best) return -1;

        if (otherSeen[v] == queryId) {
            double total = dist[v] + otherDist[v];
            if (total < best) { best = total; meet = v; }
        }

        for (int e = upFirst[v]; e < upFirst[v + 1]; e++) {
            int w = upTo[e];
            double nd = dist[v] + upW[e];
            if (seen[w] != queryId) {
                seen[w] = queryId;
                dist[w] = nd;
                predEdge[w] = e;
                pq.insert(w, nd);
            } else if (nd < dist[w]) {
                dist[w] = nd;
                predEdge[w] = e;
                pq.change(w, nd);
            }
        }
        return v;
    }

    /*********************************************************************
     *  Path unpacking.
     *********************************************************************/

    // the lower-ranked endpoint of upward edge e, i.e. the vertex storing it
    private int owner(int e) {
        // binary search for the v with upFirst[v] <= e < upFirst[v+1]
        int lo = 0, hi = V - 1;
        while (lo < hi) {
            int mid = (lo + hi + 1) >>> 1;
            if (upFirst[mid] <= e) lo = mid; else hi = mid - 1;
        }
        return lo;
    }

    // index of the upward edge from low to high; low must be the lower rank
    private int edge(int low, int high) {
        for (int e = upFirst[low]; e < upFirst[low + 1]; e++)
            if (upTo[e] == high) return e;
        throw new IllegalStateException("missing edge " + low + "-" + high);
    }

    // append the road vertices from u to w, excluding u, for an edge u-w
    // that bypasses mid (or is a plain road if mid is -1). mid was contracted
    // before both u and w, so both halves are stored at mid.
    private void expand(int u, int w, int mid, IntList out) {
        if (mid < 0) { out.add(w); return; }
        expand(u, mid, upMid[edge(mid, u)], out);
        expand(mid, w, upMid[edge(mid, w)], out);
    }

    /*********************************************************************
     *  Preprocessing.
     *********************************************************************/

    // growable adjacency list holding one edge per neighbour
    private static final class Adj {
        int[] to = new int[4];
        double[] w = new double[4];
        int[] mid = new int[4];
        int n;

        int indexOf(int x) {
            for (int i = 0; i < n; i++) if (to[i] == x) return i;
            return -1;
        }

        // add edge to x, or shorten the existing one; parallel edges collapse
        void relax(int x, double weight, int via) {
            int i = indexOf(x);
            if (i >= 0) {
                if (weight < w[i]) { w[i] = weight; mid[i] = via; }
                return;
            }
            if (n == to.length) {
                to  = java.util.Arrays.copyOf(to,  2 * n);
                w   = java.util.Arrays.copyOf(w,   2 * n);
                mid = java.util.Arrays.copyOf(mid, 2 * n);
            }
            to[n] = x; w[n] = weight; mid[n] = via; n++;
        }

        void remove(int x) {
            int i = indexOf(x);
            if (i < 0) return;
            n--;
            to[i] = to[n]; w[i] = w[n]; mid[i] = mid[n];
        }
    }

    private static final class Preprocessor {
        final int V;
        final Adj[] adj;              // edges among not-yet-contracted vertices
        final boolean[] contracted;
        final int[] deletedNeighbours;
        int shortcuts = 0;

        // witness search state
        final double[] wDist;
        final int[] wSeen;
        final IndexPQ wpq;
        int wStamp = 0;

        Preprocessor(EuclideanGraph G) {
            V = G.V();
            adj = new Adj[V];
            for (int v = 0; v < V; v++) adj[v] = new Adj();
            for (int v = 0; v < V; v++) {
                for (IntIterator it = G.neighbors(v); it.hasNext(); ) {
                    int w = it.next();
                    if (w != v) adj[v].relax(w, G.distance(v, w), -1);
                }
            }
            contracted = new boolean[V];
            deletedNeighbours = new int[V];
            wDist = new double[V];
            wSeen = new int[V];
            wpq = new IndexPQ(V);
        }

        // contract every vertex; fills rank[] and returns each vertex's
        // upward edges
        Adj[] run(int[] rank) {
            IndexPQ order = new IndexPQ(V);
            for (int v = 0; v < V; v++) order.insert(v, priority(v));

            Adj[] up = new Adj[V];
            int next = 0;
            while (!order.isEmpty()) {
                int v = order.delMin();
                rank[v] = next++;
                contract(v, false);
                contracted[v] = true;

                // v's remaining edges all lead to higher-ranked vertices;
                // detach them from the neighbours and freeze them as v's
                // upward edges
                Adj a = adj[v];
                up[v] = a;
                for (int i = 0; i < a.n; i++) {
                    int u = a.to[i];
                    adj[u].remove(v);
                    deletedNeighbours[u]++;
                }
                // neighbours' priorities depend on v, so refresh them
                for (int i = 0; i < a.n; i++) {
                    int u = a.to[i];
                    order.change(u, priority(u));
                }
            }
            return up;
        }

        // shortcuts added minus edges removed, plus a term that spreads
        // contraction evenly across the map instead of eating one region
        double priority(int v) {
            int added = contract(v, true);
            return EDGE_DIFFERENCE_WEIGHT * (added - adj[v].n) + deletedNeighbours[v];
        }

        // count the shortcuts contracting v needs; add them unless simulating
        int contract(int v, boolean simulate) {
            Adj a = adj[v];
            int needed = 0;
            for (int i = 0; i < a.n; i++) {
                int u = a.to[i];
                double wu = a.w[i];

                double maxVia = 0.0;
                for (int j = i + 1; j < a.n; j++) maxVia = Math.max(maxVia, wu + a.w[j]);
                if (i + 1 >= a.n) continue;

                witnessSearch(u, v, maxVia, simulate ? SIMULATE_SETTLE_LIMIT : WITNESS_SETTLE_LIMIT);

                for (int j = i + 1; j < a.n; j++) {
                    int w = a.to[j];
                    double via = wu + a.w[j];
                    // another path at least as short exists without v
                    if (wSeen[w] == wStamp && wDist[w] <= via) continue;
                    needed++;
                    if (!simulate) {
                        adj[u].relax(w, via, v);
                        adj[w].relax(u, via, v);
                        shortcuts++;
                    }
                }
            }
            return needed;
        }

        // bounded Dijkstra from u in the remaining graph, avoiding v
        void witnessSearch(int u, int avoid, double maxDist, int settleLimit) {
            wStamp++;
            wpq.clear();
            wDist[u] = 0.0;
            wSeen[u] = wStamp;
            wpq.insert(u, 0.0);
            int settled = 0;
            while (!wpq.isEmpty()) {
                int x = wpq.delMin();
                if (wDist[x] > maxDist || ++settled > settleLimit) break;
                Adj ax = adj[x];
                for (int i = 0; i < ax.n; i++) {
                    int y = ax.to[i];
                    if (y == avoid) continue;
                    double nd = wDist[x] + ax.w[i];
                    if (wSeen[y] != wStamp) {
                        wSeen[y] = wStamp;
                        wDist[y] = nd;
                        wpq.insert(y, nd);
                    } else if (nd < wDist[y]) {
                        wDist[y] = nd;
                        wpq.change(y, nd);
                    }
                }
            }
        }
    }

    // minimal growable int array, to keep the hot paths free of boxing
    private static final class IntList {
        int[] a = new int[16];
        int n;
        void add(int x) {
            if (n == a.length) a = java.util.Arrays.copyOf(a, 2 * n);
            a[n++] = x;
        }
        int[] toArray() { return java.util.Arrays.copyOf(a, n); }
    }
}
