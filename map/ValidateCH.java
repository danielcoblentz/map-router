/*************************************************************************
 *  Compilation:  javac ValidateCH.java
 *  Execution:    java ValidateCH file < input.txt
 *  Dependencies: EuclideanGraph.java Dijkstra.java ContractionHierarchy.java
 *                In.java StdIn.java
 *
 *  Headless check and benchmark. Answers every query on stdin with both
 *  Dijkstra (with A*) and the contraction hierarchy, and fails if
 *
 *    - the two distances differ by more than a rounding error,
 *    - one finds a path and the other does not, or
 *    - the unpacked CH path is not a chain of real roads from s to d whose
 *      length matches the reported distance.
 *
 *  Prints preprocessing cost, total query time for each method and the
 *  average number of vertices each one settled.
 *
 ****************************************************************************/

import java.util.HashSet;
import java.util.Set;

public class ValidateCH {
    private static final double TOLERANCE = 1e-6;

    public static void main(String[] args) {
        if (args.length < 1) {
            System.err.println("Usage: java ValidateCH <graph-file> < <query-file>");
            System.exit(2);
        }
        EuclideanGraph G = new EuclideanGraph(new In(args[0]));
        System.err.println("Done reading the graph " + args[0]);

        // set of roads, to check that an unpacked path uses only real edges
        Set<Long> roads = new HashSet<>();
        for (int v = 0; v < G.V(); v++)
            for (IntIterator it = G.neighbors(v); it.hasNext(); )
                roads.add(key(v, it.next()));

        long t0 = System.nanoTime();
        ContractionHierarchy ch = new ContractionHierarchy(G);
        long preprocessNanos = System.nanoTime() - t0;

        Dijkstra dijkstra = new Dijkstra(G);

        int queries = 0, unreachable = 0, failures = 0;
        long dijkstraNanos = 0, chNanos = 0;
        long dijkstraVisited = 0, chVisited = 0;

        while (!StdIn.isEmpty()) {
            int s = StdIn.readInt();
            int d = StdIn.readInt();
            queries++;

            long a = System.nanoTime();
            dijkstra.enableAStar(d);
            double expected = dijkstra.distance(s, d);
            long b = System.nanoTime();
            double actual = ch.distance(s, d);
            long c = System.nanoTime();
            dijkstraNanos += b - a;
            chNanos += c - b;
            dijkstraVisited += dijkstra.visitedCount();
            chVisited += ch.visitedCount();

            String problem = check(G, roads, ch, s, d, expected, actual);
            if (expected == Double.MAX_VALUE && problem == null) unreachable++;
            if (problem != null) {
                failures++;
                if (failures <= 20)
                    System.out.println("FAIL query " + queries + " (" + s + " -> " + d + "): " + problem);
            }
        }

        System.out.println("=== " + args[0] + " ===");
        System.out.println("Vertices / roads       : " + G.V() + " / " + G.E());
        System.out.printf ("Preprocessing          : %.3f s%n", preprocessNanos / 1e9);
        System.out.println("Shortcuts added        : " + ch.shortcutCount());
        System.out.println("Upward edges           : " + ch.upwardEdgeCount());
        System.out.println("Queries                : " + queries + " (" + unreachable + " unreachable)");
        if (queries > 0) {
            System.out.printf ("Dijkstra+A* query time : %.3f s, avg settled %.1f%n",
                               dijkstraNanos / 1e9, (double) dijkstraVisited / queries);
            System.out.printf ("CH query time          : %.3f s, avg settled %.1f%n",
                               chNanos / 1e9, (double) chVisited / queries);
        }
        System.out.println("Mismatches             : " + failures);
        System.exit(failures == 0 ? 0 : 1);
    }

    // null if the CH answer is consistent with Dijkstra, else a description
    static String check(EuclideanGraph G, Set<Long> roads, ContractionHierarchy ch,
                        int s, int d, double expected, double actual) {
        boolean expectedReachable = expected != Double.MAX_VALUE;
        boolean actualReachable = actual != Double.MAX_VALUE;
        if (expectedReachable != actualReachable)
            return "Dijkstra " + (expectedReachable ? "found" : "found no")
                 + " path but CH " + (actualReachable ? "found one" : "did not");
        if (!expectedReachable) return ch.path() == null ? null : "CH returned a path for an unreachable pair";

        if (Math.abs(expected - actual) > TOLERANCE * Math.max(1.0, expected))
            return "distance " + actual + " but Dijkstra says " + expected;

        int[] p = ch.path();
        if (p == null) return "CH reported a distance but no path";
        if (p[0] != s || p[p.length - 1] != d)
            return "path runs " + p[0] + " -> " + p[p.length - 1];
        double length = 0.0;
        for (int i = 1; i < p.length; i++) {
            if (!roads.contains(key(p[i - 1], p[i])))
                return "path uses " + p[i - 1] + "-" + p[i] + ", which is not a road";
            length += G.distance(p[i - 1], p[i]);
        }
        if (Math.abs(length - actual) > TOLERANCE * Math.max(1.0, actual))
            return "unpacked path is " + length + " long but CH reported " + actual;
        return null;
    }

    private static long key(int u, int v) {
        int lo = Math.min(u, v), hi = Math.max(u, v);
        return ((long) lo << 32) | hi;
    }
}
