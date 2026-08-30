/*************************************************************************
 *  Unit tests for the 4-ary indexed priority queue.
 *************************************************************************/

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class IndexPQTest {

    @BeforeEach
    public void resetCounters() {
        IndexPQ.insertCount = 0;
        IndexPQ.changeCount = 0;
        IndexPQ.delMinCount = 0;
        IndexPQ.maxPQSize  = 0;
    }

    // read a private field of an IndexPQ so the heap shape can be inspected
    private static Object peek(IndexPQ pq, String name) throws Exception {
        Field f = IndexPQ.class.getDeclaredField(name);
        f.setAccessible(true);
        return f.get(pq);
    }

    // every node must be no larger than each of its up to four children,
    // and qp[] must stay the exact inverse of pq[]
    private static void assertHeapInvariant(IndexPQ pq) throws Exception {
        int[] heap = (int[]) peek(pq, "pq");
        int[] inverse = (int[]) peek(pq, "qp");
        double[] priority = (double[]) peek(pq, "priority");
        int n = (Integer) peek(pq, "N");
        for (int i = 1; i <= n; i++) {
            assertEquals(i, inverse[heap[i]], "qp[] is not the inverse of pq[] at " + i);
            int firstChild = 4 * (i - 1) + 2;
            for (int c = firstChild; c < firstChild + 4 && c <= n; c++) {
                assertTrue(priority[heap[i]] <= priority[heap[c]],
                           "heap order broken between " + i + " and child " + c);
            }
        }
    }

    @Test
    public void delMinReturnsKeysInPriorityOrder() throws Exception {
        IndexPQ pq = new IndexPQ(200);
        Random rng = new Random(42);
        double[] given = new double[200];
        for (int k = 0; k < 200; k++) {
            given[k] = rng.nextDouble() * 1000.0;
            pq.insert(k, given[k]);
        }
        assertHeapInvariant(pq);

        List<Integer> order = new ArrayList<>();
        double previous = Double.NEGATIVE_INFINITY;
        while (!pq.isEmpty()) {
            int k = pq.delMin();
            assertTrue(given[k] >= previous, "delMin returned " + given[k] + " after " + previous);
            previous = given[k];
            order.add(k);
        }
        assertEquals(200, order.size());
        assertEquals(200, order.stream().distinct().count());
    }

    @Test
    public void heapInvariantHoldsThroughInterleavedOperations() throws Exception {
        IndexPQ pq = new IndexPQ(64);
        Random rng = new Random(7);
        for (int k = 0; k < 64; k++) pq.insert(k, rng.nextDouble());
        for (int round = 0; round < 40; round++) {
            pq.change(rng.nextInt(64), rng.nextDouble());
            assertHeapInvariant(pq);
        }
        for (int i = 0; i < 20; i++) {
            pq.delMin();
            assertHeapInvariant(pq);
        }
    }

    @Test
    public void changeLowersPriorityAndPromotesTheKey() {
        IndexPQ pq = new IndexPQ(10);
        pq.insert(1, 5.0);
        pq.insert(2, 7.0);
        pq.insert(3, 9.0);
        pq.change(3, 1.0);
        assertEquals(3, pq.delMin());
        assertEquals(1, pq.delMin());
        assertEquals(2, pq.delMin());
    }

    @Test
    public void changeRaisingPriorityDemotesTheKey() {
        IndexPQ pq = new IndexPQ(10);
        pq.insert(1, 1.0);
        pq.insert(2, 2.0);
        pq.insert(3, 3.0);
        pq.change(1, 99.0);
        assertEquals(2, pq.delMin());
        assertEquals(3, pq.delMin());
        assertEquals(1, pq.delMin());
    }

    @Test
    public void insertCountTracksEveryInsert() {
        IndexPQ pq = new IndexPQ(10);
        assertEquals(0, IndexPQ.insertCount);
        for (int k = 0; k < 6; k++) pq.insert(k, k);
        assertEquals(6, IndexPQ.insertCount);
        pq.change(0, -1.0);
        assertEquals(6, IndexPQ.insertCount, "change() must not be counted as an insert");
        pq.delMin();
        assertEquals(6, IndexPQ.insertCount, "delMin() must not be counted as an insert");
    }

    @Test
    public void countersRecordChangesDelMinsAndPeakSize() {
        IndexPQ pq = new IndexPQ(10);
        for (int k = 0; k < 4; k++) pq.insert(k, k);
        pq.change(2, 0.5);
        pq.change(3, 0.25);
        pq.delMin();
        assertEquals(4, IndexPQ.insertCount);
        assertEquals(2, IndexPQ.changeCount);
        assertEquals(1, IndexPQ.delMinCount);
        assertEquals(4, IndexPQ.maxPQSize);
    }

    @Test
    public void clearEmptiesTheQueueForReuse() throws Exception {
        IndexPQ pq = new IndexPQ(10);
        for (int k = 0; k < 5; k++) pq.insert(k, k);
        pq.clear();
        assertTrue(pq.isEmpty());

        pq.insert(9, 3.0);
        pq.insert(8, 1.0);
        assertHeapInvariant(pq);
        assertEquals(8, pq.delMin());
        assertEquals(9, pq.delMin());
        assertTrue(pq.isEmpty(), "keys from before clear() must not reappear");
    }
}
