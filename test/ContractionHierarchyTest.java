/*************************************************************************
 *  Unit tests for the contraction hierarchy.
 *
 *  The hand-checked graph is the same one DijkstraTest uses:
 *
 *  Vertices: 0(0,0) 1(10,0) 2(20,0) 3(0,10) 4(10,10) 5(0,50)
 *            6(100,100) 7(110,100)
 *  Edges:    0-1 1-2 0-3 3-4 4-2 1-4 0-5 6-7
 *
 *  The random graphs are checked against plain Dijkstra on every pair of
 *  vertices, which covers shortcuts, unpacking, ties and disconnected
 *  components far more thoroughly than hand-picked cases can.
 *****************************************************************/

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Duration;
import java.util.HashSet;
import java.util.Random;
import java.util.Set;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

public class ContractionHierarchyTest {

    @TempDir
    static Path dir;

    private static EuclideanGraph hand;

    @BeforeAll
    public static void buildHandGraph() throws Exception {
        hand = load("hand.txt",
            "8 8",
            "0 0 0", "1 10 0", "2 20 0", "3 0 10", "4 10 10", "5 0 50",
            "6 100 100", "7 110 100",
            "0 1", "1 2", "0 3", "3 4", "4 2", "1 4", "0 5", "6 7");
    }

    private static EuclideanGraph load(String name, String... lines) throws Exception {
        Path file = dir.resolve(name);
        try (PrintWriter out = new PrintWriter(Files.newBufferedWriter(file))) {
            for (String line : lines) out.println(line);
        }
        In in = new In(file.toString());
        EuclideanGraph G = new EuclideanGraph(in);
        in.close();
        return G;
    }

    // random points joined to a few near neighbours, which gives a road-like
    // graph with several components, some parallel edges and some ties
    private static EuclideanGraph randomGraph(String name, int V, long seed) throws Exception {
        Random rng = new Random(seed);
        int[] x = new int[V], y = new int[V];
        for (int v = 0; v < V; v++) { x[v] = rng.nextInt(1000); y[v] = rng.nextInt(1000); }
        StringBuilder edges = new StringBuilder();
        int E = 0;
        for (int v = 0; v < V; v++) {
            int links = rng.nextInt(3);   // 0, 1 or 2 links out of v
            for (int k = 0; k < links; k++) {
                // pick the closest of a few random candidates
                int bestW = -1;
                long bestD = Long.MAX_VALUE;
                for (int t = 0; t < 8; t++) {
                    int w = rng.nextInt(V);
                    long dx = x[v] - x[w], dy = y[v] - y[w];
                    if (w != v && dx * dx + dy * dy < bestD) { bestD = dx * dx + dy * dy; bestW = w; }
                }
                if (bestW >= 0) { edges.append(v).append(' ').append(bestW).append('\n'); E++; }
            }
        }
        String[] lines = new String[1 + V + 1];
        lines[0] = V + " " + E;
        for (int v = 0; v < V; v++) lines[1 + v] = v + " " + x[v] + " " + y[v];
        lines[1 + V] = edges.toString().trim();
        return load(name, lines);
    }

    private static String captured(Runnable r) {
        PrintStream original = System.out;
        ByteArrayOutputStream buffer = new ByteArrayOutputStream();
        System.setOut(new PrintStream(buffer));
        try { r.run(); } finally { System.setOut(original); }
        return buffer.toString().trim();
    }

    // the path must start at s, end at d, use only real roads, and add up
    // to the reported distance
    private static void assertValidPath(EuclideanGraph G, int[] p, int s, int d, double distance) {
        assertNotNull(p, "no path for " + s + " -> " + d);
        assertEquals(s, p[0]);
        assertEquals(d, p[p.length - 1]);
        double length = 0.0;
        for (int i = 1; i < p.length; i++) {
            boolean road = false;
            for (IntIterator it = G.neighbors(p[i - 1]); it.hasNext(); )
                if (it.next() == p[i]) road = true;
            assertTrue(road, p[i - 1] + "-" + p[i] + " is not a road");
            length += G.distance(p[i - 1], p[i]);
        }
        assertEquals(distance, length, 1e-9);
    }

    @Test
    public void findsTheHandCheckedPath() {
        ContractionHierarchy ch = new ContractionHierarchy(hand);
        assertEquals(20.0, ch.distance(0, 2), 1e-9);
        assertArrayEquals(new int[] {0, 1, 2}, ch.path());
        assertEquals("2-1-0", captured(() -> ch.showPath(2, 0)));
    }

    @Test
    public void showPathMatchesDijkstraFormat() {
        ContractionHierarchy ch = new ContractionHierarchy(hand);
        Dijkstra dijkstra = new Dijkstra(hand);
        dijkstra.compute(5, 2);
        String expected = captured(() -> dijkstra.showPath(2, 5));
        assertEquals(expected, captured(() -> ch.showPath(2, 5)));
    }

    @Test
    public void sourceEqualToDestinationIsZero() {
        ContractionHierarchy ch = new ContractionHierarchy(hand);
        assertEquals(0.0, ch.distance(4, 4), 0.0);
        assertArrayEquals(new int[] {4}, ch.path());
    }

    @Test
    public void unreachableDestinationIsReported() {
        ContractionHierarchy ch = new ContractionHierarchy(hand);
        assertEquals(Double.MAX_VALUE, ch.distance(6, 2));
        assertNull(ch.path());
        assertEquals("No path from 6 to 2", captured(() -> ch.showPath(2, 6)));
        assertTimeoutPreemptively(Duration.ofSeconds(5), () -> ch.drawPath(6, 2));
    }

    @Test
    public void repeatedQueriesOnOneInstanceStayCorrect() {
        ContractionHierarchy ch = new ContractionHierarchy(hand);
        assertEquals(20.0, ch.distance(0, 2), 1e-9);
        assertEquals(Double.MAX_VALUE, ch.distance(7, 0));
        assertEquals(50.0, ch.distance(0, 5), 1e-9);
        assertEquals(10.0, ch.distance(6, 7), 1e-9);
        assertEquals(20.0, ch.distance(0, 2), 1e-9);
        assertArrayEquals(new int[] {0, 1, 2}, ch.path());
    }

    @Test
    public void contractionOrderIsAPermutation() {
        ContractionHierarchy ch = new ContractionHierarchy(hand);
        Set<Integer> ranks = new HashSet<>();
        for (int v = 0; v < hand.V(); v++) ranks.add(ch.rank(v));
        assertEquals(hand.V(), ranks.size());
        for (int r = 0; r < hand.V(); r++) assertTrue(ranks.contains(r));
    }

    @Test
    public void agreesWithDijkstraOnEveryPairOfRandomGraphs() throws Exception {
        for (long seed = 1; seed <= 4; seed++) {
            EuclideanGraph G = randomGraph("random" + seed + ".txt", 150, seed);
            ContractionHierarchy ch = new ContractionHierarchy(G);
            Dijkstra dijkstra = new Dijkstra(G);
            for (int s = 0; s < G.V(); s++) {
                for (int d = 0; d < G.V(); d++) {
                    double expected = dijkstra.distance(s, d);
                    double actual = ch.distance(s, d);
                    if (expected == Double.MAX_VALUE) {
                        assertEquals(Double.MAX_VALUE, actual, "seed " + seed + ": " + s + " -> " + d);
                        assertNull(ch.path());
                    } else {
                        assertEquals(expected, actual, 1e-9, "seed " + seed + ": " + s + " -> " + d);
                        assertValidPath(G, ch.path(), s, d, actual);
                    }
                }
            }
        }
    }

    @Test
    public void searchSettlesFewerVerticesThanDijkstraOnALargerGraph() throws Exception {
        EuclideanGraph G = randomGraph("large.txt", 3000, 99);
        ContractionHierarchy ch = new ContractionHierarchy(G);
        Dijkstra dijkstra = new Dijkstra(G);
        Random rng = new Random(5);
        long chSettled = 0, dijkstraSettled = 0;
        for (int q = 0; q < 200; q++) {
            int s = rng.nextInt(G.V()), d = rng.nextInt(G.V());
            assertEquals(dijkstra.distance(s, d), ch.distance(s, d), 1e-9);
            chSettled += ch.visitedCount();
            dijkstraSettled += dijkstra.visitedCount();
        }
        assertTrue(chSettled < dijkstraSettled,
                   "CH settled " + chSettled + " vs Dijkstra " + dijkstraSettled);
    }
}
