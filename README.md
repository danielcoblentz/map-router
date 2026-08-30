# Map Routing

Repeated shortest-path queries on the continental US road network, using A* search, a lazy per-query reset, and a 4-ary indexed priority queue.

## Provenance

This is coursework, not an independent build.
The assignment shipped a template: the graph representation, the file and stdin readers, the turtle graphics, the map data, a C reference implementation under `map/c/`, and a working plain Dijkstra.
What I wrote is the optimisation layer on top of that template, the instrumentation, the tests, and the fixes described below.
The writeup I submitted with the assignment, including the hours logged and the limitations I reported at the time, is kept verbatim in `map/readme_class.txt`.

Commit `dd873a0` is the untouched starter code, so `git diff dd873a0 -- map` shows exactly what I changed.

### Provided

`EuclideanGraph.java`, `Point.java`, `In.java`, `StdIn.java`, `Turtle.java`, `IntIterator.java`, `Distances.java`, `ShortestPath.java` and `Paths.java`.
The original `Dijkstra.java`, a plain Dijkstra with no early exit that rebuilt `dist[]`, `pred[]` and the queue on every query.
The original `IndexPQ.java`, a binary heap taken from Sedgewick, Algorithms in Java, 3rd edition.
Everything under `map/c/`, and every `usa-*.txt`, `grid*.txt` and `input6.txt` data or query file.

### Mine

- The lazy per-query reset in `Dijkstra`.
- The A* heuristic and the early exit when the destination is dequeued.
- The 4-ary heap in `IndexPQ`, replacing the provided binary heap.
- The counters in `IndexPQ` and the per-query and summary reporting in `Paths`.
- The bug fixes below and the JUnit tests under `test/`.

## Known limitations

**`usa.txt` is not connected, and three of the shipped queries have no answer.**
The graph has seven connected components.
Vertices 20901, 19295 and 35957 each sit with a single neighbour in a two-vertex component, and `usa-1000long.txt` and `usa-50000short.txt` between them contain three queries that start in those components.
The version I submitted hung on all three.
`drawPath` guarded on `pred[d] == -1`, but under the lazy reset `pred[]` is deliberately never cleared, so for a destination the current query never touched it followed a stale predecessor chain and never terminated.
At the time I deleted those three query lines from the input files to get a clean run.
The lines are restored, `drawPath` now uses the same reachability test as `showPath`, and the three queries print `No path from ... to ...` and carry on.

**`Distances` does not use A*.**
Only `Paths` and `ShortestPath` call `enableAStar`, so `java Distances` runs plain Dijkstra with the lazy reset and the 4-ary heap.

**The heuristic assumes Euclidean edge weights.**
Edge cost is the straight-line distance between endpoints, so straight-line distance to the goal never overestimates and the search stays optimal.
A cost model such as travel time would need a different heuristic.

## Key algorithms

### A* heuristic

Plain Dijkstra expands the frontier evenly in every direction.
A* orders the queue by `f(n) = g(n) + h(n)`, where `g(n)` is the cost from the source and `h(n)` is the straight-line distance from `n` to the destination, which pulls the search towards the goal.

`dist[]` holds `g` alone.
The heuristic is applied when computing the queue priority rather than folded into the edge weight, so `distance(s, d)` still returns a real path length when A* is on.

### 4-ary heap in IndexPQ

The provided `IndexPQ` was a binary heap.
Giving each node four children instead of two cuts the height from `log2 N` to `log4 N`, which halves the levels a `fixUp` or `fixDown` walks, at the cost of comparing four children instead of two on the way down.

Index arithmetic, 1-based:

- Parent of `i`: `(i + 2) / 4`
- Children of `i`: `4 * (i - 1) + 2` through `4 * (i - 1) + 5`

### Lazy reset

The provided Dijkstra reinitialised `dist[]` and `pred[]` and filled the queue with all `V` vertices on every query.
On a graph of 87575 vertices that is the dominant cost when the answer only touches a few hundred of them.

Instead, a `seen[]` array records the query id that last touched each vertex.
A vertex whose `seen[]` entry is stale counts as unvisited, so no array ever has to be cleared.

```java
// seen[w] != queryId means w has not been touched in this query yet
if (seen[w] != queryId || baseCost < dist[w] - EPSILON) {
    dist[w] = baseCost;
    pred[w] = v;
    ...
    seen[w] = queryId;
}
```

The priority queue is allocated once per `Dijkstra` instance and emptied in O(1) between queries, for the same reason.
Allocating a fresh `IndexPQ` per query would have cost three O(V) arrays each time and undone most of the benefit.

### Early stopping

For a single source and destination pair there is no reason to settle the whole graph, so the loop exits as soon as the destination is dequeued.

```java
int v = pq.delMin();
if (v == d) break;
```

## Results & Observations


### 1. Complexity analysis
| Implementation  | Worst-Case Time per Query                       | Additional Space Overhead |
| --------------- | ----------------------------------------------- | ------------------------- |
| Plain Dijkstra  | O((V + E) log V)                                | O(V + E)                  |
| + Lazy Reset    | O((V′ + E′) log V′), where V′, E′ ≪ V, E        | + O(V) for `seen[]` array |
| + A\* Heuristic | O((V″ + E″) log V″), where V″, E″ ≤ V′, E′      | same                      |
| + 4-ary Heap    | O((V″ + E″) log₄ V″) ≈ O((V″ + E″) · ½ log₂ V″) | same                      |


Variable definitions:

- V: total number of vertices in the graph
- E: total number of edges in the graph
- V′, E′: number of vertices and edges actually visited during a query using lazy reset
- V″, E″: number of vertices and edges actually visited during a query using A* heuristic

`log₄ V` is approximately half of `log₂ V`, which means the 4-ary heap performs fewer comparisons per priority queue operation.


## 2. Empirical Metrics

### Base Dijkstra algorithm (no improvements)

| Input File         | Total Time (s) | Program Time (s) | Avg Vertices Visited | PQ Inserts | PQ Changes | PQ delMins | Max PQ Size | Seen\[] Memory (bytes) | Output Agrees? |
| ------------------ | -------------- | ---------------- | -------------------- | ---------- | ---------- | ---------- | ----------- | ---------------------- | -------------- |
| usa-1000long.txt   | 17.399         | 58.980           | 87475.4              | 87475      | 150000     | 87475      | 22000       | 350300                 | Yes            |
| usa-5000short.txt  | 84.220         | 282.408          | 87563.0              | 87563      | 152000     | 87563      | 23000       | 350300                 | Yes            |
| usa-50000short.txt | 858.453        | 2863.474         | 1660.2               | 1660       | 4300       | 1660       | 1800        | 350300                 | Yes            |

### Improved Dijkstra Algorithm” (with methods mentioned above)

| Input File         | Total Time (s) | Program Time (s) | Avg Vertices Visited | PQ Inserts | PQ Changes | PQ delMins | Max PQ Size | Seen\[] Memory (bytes) | Output Agrees? |
| ------------------ | -------------- | ---------------- | -------------------- | ---------- | ---------- | ---------- | ----------- | ---------------------- | -------------- |
| usa-1000long.txt   | 1.837          | 8.944            | 9599.0               | 0.0        | 1881.5     | 9599       | 296.3       | 350300                 | Yes            |
| usa-5000short.txt  | 1.031          | 32.684           | 405.2                | 0.0        | 75.8       | 405        | 33.9        | 350300                 | Yes            |
| usa-50000short.txt | 12.059         | 325.033          | 426.4                | 0.0        | 79.2       | 426        | 34.3        | 350300                 | Yes            |




| Column                   | Meaning                                                                                                          |
| ------------------------ | ---------------------------------------------------------------------------------------------------------------- |
| **Total Time (s)**       | Cumulative time spent solving all shortest-path queries, excluding visualization and delays (pure compute time). |
| **Program Time (s)**     | Wall-clock time for the entire program, including graphics rendering and `Thread.sleep(5)` pauses.               |
| **Avg Vertices Visited** | Mean number of nodes actually relaxed (or “touched”) during each shortest-path query. Lower is better.           |
| **PQ Inserts**           | Number of times a node was added to the priority queue. Should be non-zero per query (bug noted).                |
| **PQ Changes**           | Number of times a node’s priority was decreased in the queue (due to a better path found).                       |
| **PQ delMins**           | Number of `delMin()` operations — roughly matches nodes visited, since each visit pops from the PQ.              |
| **Max PQ Size**          | Largest size the priority queue reached during a query — gives a sense of the frontier width.                    |
| **Seen\[] Memory**       | Space in bytes used by the `seen[]` array to track which nodes were touched — constant per run.                  |


### Performance imporvements
- Compared to the baseline version, the optimized algorithm with A* heuristic, lazy reset, and a 4-ary heap achieves dramatic speedups:
- Over 90% reduction in nodes visited per query for long-range paths.
- Execution time dropped from nearly 2900s to 325s for 50,000 queries.
- PQ operations reduced by an order of magnitude, and memory usage stayed constant.

## Input format

A map file looks like:

```
<V> <E>
0    x0    y0
1    x1    y1
...
V-1  xV-1  yV-1
u0   v0
u1   v1
...
uE-1 vE-1
```

`V` is the number of intersections and `E` the number of two-way roads.
The next `V` lines give a vertex index and its integer coordinates, and the following `E` lines give unordered pairs of vertex indices.
Queries are pairs of source and destination indices read from stdin.

## Build and run

```
cd map
javac *.java
java Paths usa.txt < usa-5000short.txt
```

`Paths` opens a turtle graphics window and draws each path, so the JVM keeps running after the last query until the window is closed.
`Distances` prints path lengths only and needs no display.

## Tests

The tests use JUnit 5 through the standalone console launcher, which is downloaded on demand rather than committed.

```
JUNIT=junit-platform-console-standalone-1.10.2.jar
curl -L -o $JUNIT https://repo1.maven.org/maven2/org/junit/platform/junit-platform-console-standalone/1.10.2/$JUNIT
javac -d out map/*.java
javac -cp "out;$JUNIT" -d out test/*.java
java -jar $JUNIT execute -cp out --select-class=IndexPQTest --select-class=DijkstraTest
```

`out/` and that jar are both in `.gitignore`.

Use `:` instead of `;` in the classpath on macOS and Linux.

`IndexPQTest` covers `delMin` ordering, raising and lowering a priority with `change`, the 4-ary parent and child invariant read back by reflection, `clear`, and the operation counters.
`DijkstraTest` builds an eight-vertex graph whose shortest path is obvious by inspection, then asserts that plain Dijkstra and A* return the same path and the same length, that repeated queries on one instance stay correct despite the lazy reset, and that an unreachable destination is reported instead of hanging.

## Acknowledgments

- [Sedgewick and Wayne, Algorithms, 4th edition](https://algs4.cs.princeton.edu/home/)
- [A* Search Algorithm, Red Blob Games](https://www.redblobgames.com/pathfinding/a-star/introduction.html)
- [K-ary heap, GeeksforGeeks](https://www.geeksforgeeks.org/k-ary-heap/)
