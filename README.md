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

## Complexity

| Implementation | Worst-case time per query | Extra space |
| --- | --- | --- |
| Plain Dijkstra | O((V + E) log V) | O(V + E) |
| Plus lazy reset | O((Vt + Et) log Vt), over the touched subset Vt, Et | plus O(V) for `seen[]` |
| Plus A* | O((Va + Ea) log Va), where Va, Ea are no larger than Vt, Et | same |
| Plus 4-ary heap | O((Va + Ea) log4 Va) | same |

`log4 V` is half of `log2 V`, so the 4-ary heap walks half as many levels per operation.

## Measured results

Everything below was measured on an Intel Core i9-9900K at 3.6 GHz with 32 GB of RAM, Windows 10 and JDK 21.0.2.
Numbers from one machine and one JDK do not transfer, so treat the ratios as the point rather than the absolute times.

Current code, one run of `java Paths usa.txt < <query file>` per row, reading the summary the program prints.
"Query time" is the `Total time` line, which sums the per-query search and excludes graph loading and drawing.
"Wall clock" is the whole process, including the window and the 5 ms pause after each query.

| Query file | Queries | Query time | Wall clock | Avg vertices visited | Avg PQ inserts | Avg PQ changes | Avg max PQ size |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `usa-1000long.txt` | 1000 | 1.726 s | 9.4 s | 9589.4 | 9841.9 | 1879.6 | 296.0 |
| `usa-5000short.txt` | 5000 | 0.300 s | 31.7 s | 405.3 | 435.5 | 75.8 | 33.9 |
| `usa-50000short.txt` | 50000 | 3.039 s | 309.8 s | 426.4 | 456.6 | 79.2 | 34.3 |

Against the starter template, through `Distances` so that both sides run headless and neither side uses A*.
This isolates the lazy reset, the early exit and the 4-ary heap.
Two runs each, wall clock for the whole process, of which roughly 0.9 s is reading and building the graph.

| Query file | Starter template (`dd873a0`) | Current |
| --- | --- | --- |
| `usa-100long.txt` | 3.02 s, 3.00 s | 1.10 s, 1.12 s |
| `usa-1000long.txt` | 21.9 s, 22.4 s | 8.84 s, 8.70 s |

### On the numbers this README used to carry

An earlier version of this file reported 12.059 s for `usa-50000short.txt`, and `map/readme_class.txt` reports 5.677 s for what should be the same run.
They cannot both be right, neither reproduces here, and I have no way to tell which machine or build produced which, so both are gone rather than republished.

The vertex counts did carry over, which is the part that does not depend on the machine.
Excluding the restored unreachable query, `usa-1000long.txt` averages exactly 9599.0 vertices visited, and `usa-50000short.txt` averages 426.4, both matching what I recorded at the time.

I have also dropped the claim that the 4-ary heap was worth 20 to 30 percent on its own.
I never measured the heap in isolation, only the three optimisations together.

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
