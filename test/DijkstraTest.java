/*************************************************************************
 *  Unit tests for the search, on a graph small enough to check by hand.
 *
 *  Vertices: 0(0,0) 1(10,0) 2(20,0) 3(0,10) 4(10,10) 5(0,50)
 *            6(100,100) 7(110,100)
 *  Edges:    0-1 1-2 0-3 3-4 4-2 1-4 0-5 6-7
 *
 *  Shortest 0 to 2 is 0-1-2 at 10 + 10 = 20. Every other route runs
 *  through the diagonal edge 4-2 of length sqrt(200) and costs 34.14.
 *  Vertices 6 and 7 form a separate component, so 6 cannot reach 2.
 *****************************************************************/

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Duration;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

public class DijkstraTest {

    private static EuclideanGraph G;

    @BeforeAll
    public static void buildGraph(@TempDir Path dir) throws Exception {
        Path file = dir.resolve("hand.txt");
        try (PrintWriter out = new PrintWriter(Files.newBufferedWriter(file))) {
            out.println("8 8");
            out.println("0 0 0");
            out.println("1 10 0");
            out.println("2 20 0");
            out.println("3 0 10");
            out.println("4 10 10");
            out.println("5 0 50");
            out.println("6 100 100");
            out.println("7 110 100");
            out.println("0 1");
            out.println("1 2");
            out.println("0 3");
            out.println("3 4");
            out.println("4 2");
            out.println("1 4");
            out.println("0 5");
            out.println("6 7");
        }
        In in = new In(file.toString());
        G = new EuclideanGraph(in);
        in.close();   // release the handle so  can clean up
    }

    // capture what showPath() writes, since pred[] is not exposed
    private static String pathOf(Dijkstra dijkstra, int s, int d) {
        PrintStream original = System.out;
        ByteArrayOutputStream buffer = new ByteArrayOutputStream();
        System.setOut(new PrintStream(buffer));
        try {
            dijkstra.showPath(d, s);
        } finally {
            System.setOut(original);
        }
        return buffer.toString().trim();
    }

    @Test
    public void plainDijkstraFindsTheHandCheckedPath() {
        Dijkstra dijkstra = new Dijkstra(G);
        assertEquals(20.0, dijkstra.distance(0, 2), 1e-9);
        dijkstra.compute(0, 2);
        assertEquals("2-1-0", pathOf(dijkstra, 0, 2));
    }

    @Test
    public void aStarAgreesWithPlainDijkstra() {
        Dijkstra plain = new Dijkstra(G);
        Dijkstra guided = new Dijkstra(G);

        plain.compute(0, 2);
        String plainPath = pathOf(plain, 0, 2);

        guided.enableAStar(2);
        guided.compute(0, 2);
        String guidedPath = pathOf(guided, 0, 2);

        assertEquals(plainPath, guidedPath);
        assertEquals("2-1-0", guidedPath);

        guided.enableAStar(2);
        assertEquals(20.0, guided.distance(0, 2), 1e-9);
    }

    @Test
    public void aStarNeverExaminesMoreVerticesThanPlainDijkstra() {
        Dijkstra plain = new Dijkstra(G);
        plain.compute(0, 2);

        Dijkstra guided = new Dijkstra(G);
        guided.enableAStar(2);
        guided.compute(0, 2);

        org.junit.jupiter.api.Assertions.assertTrue(
            guided.visitedCount() <= plain.visitedCount(),
            "A* visited " + guided.visitedCount() + " vs " + plain.visitedCount());
    }

    @Test
    public void repeatedQueriesOnOneInstanceStayCorrect() {
        Dijkstra dijkstra = new Dijkstra(G);
        assertEquals(20.0, dijkstra.distance(0, 2), 1e-9);
        assertEquals(10.0, dijkstra.distance(0, 3), 1e-9);
        assertEquals(50.0, dijkstra.distance(0, 5), 1e-9);
        // lazy reset means dist[] and pred[] still hold values from earlier
        // queries, so run the first query again and expect the same answer
        assertEquals(20.0, dijkstra.distance(0, 2), 1e-9);
        dijkstra.compute(0, 2);
        assertEquals("2-1-0", pathOf(dijkstra, 0, 2));
    }

    @Test
    public void unreachableDestinationIsReportedRatherThanHanging() {
        Dijkstra dijkstra = new Dijkstra(G);
        dijkstra.enableAStar(2);
        dijkstra.compute(6, 2);
        assertEquals("No path from 6 to 2", pathOf(dijkstra, 6, 2));

        // drawPath used to walk a stale pred[] chain forever on an
        // unreachable destination; it must now return at once
        assertTimeoutPreemptively(Duration.ofSeconds(5), () -> dijkstra.drawPath(6, 2));
    }
}
