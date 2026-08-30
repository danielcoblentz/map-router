/*********************************************************************
 *  Indirect priority queue.
 *
 *  The priority queue maintains its own copy of the priorities,
 *  unlike the one in Algorithms in Java.
 *
 *  This code is from "Algorithms in Java, Third Edition,
 *  by Robert Sedgewick, Addison-Wesley, 2003.
 *

 *********************************************************************/

 public class IndexPQ {
    private int N;              // number of elements on PQ
    private int[] pq;           // heap, 1-based: pq[1] is min
    private int[] qp;           // inverse: qp[pq[i]] = i
    private double[] priority;  // priority[k] = value for key k


    //tracking metrics
    public static int insertCount = 0;
    public static int changeCount = 0;
    public static int delMinCount = 0;
    public static int maxPQSize = 0;


    public IndexPQ(int maxN) {
        pq = new int[maxN + 1];
        qp = new int[maxN + 1];
        priority = new double[maxN + 1];
        N = 0;
    }

    public boolean isEmpty() { return N == 0; }

    // insert key k with given priority
    public void insert(int k, double val) {
        insertCount++;
        N++;
        qp[k] = N;
        pq[N] = k;
        priority[k] = val;
        fixUp(N);
        maxPQSize = Math.max(maxPQSize, N);
    }

    // remove and return the key with smallest priority
    public int delMin() {
        delMinCount++;
        int min = pq[1];
        exch(1, N);
        N--;
        fixDown(1);
        return min;
    }

    // decrease/increase the priority of key k
    public void change(int k, double val) {
        changeCount++;
        priority[k] = val;
        int idx = qp[k];
        fixUp(idx);
        fixDown(idx);
    }

    // swap pq[i] and pq[j]
    private void exch(int i, int j) {
        int a = pq[i]; pq[i] = pq[j]; pq[j] = a;
        qp[pq[i]] = i;
        qp[pq[j]] = j;
    }

    // true if priority[pq[i]] > priority[pq[j]]
    private boolean greater(int i, int j) {
        return priority[pq[i]] > priority[pq[j]];
    }

    // move item at k up until heap-order restored (used after insertion)
    private void fixUp(int k) {
        while (k > 1) {
            int parent = (k + 2) / 4;    
            if (!greater(parent, k)) break;
            exch(parent, k);
            k = parent;
        }
    }

    // move item at k down until heap-order restored(used after removnig the min)
    private void fixDown(int k) {
        while (true) {
            int firstChild = 4 * (k - 1) + 2;  // index of first of up to 4 children
            if (firstChild > N) break;
            // find smallest among up to 4 children
            int best = firstChild;
            for (int offset = 1; offset < 4; offset++) {
                int idx = firstChild + offset;
                if (idx <= N && greater(best, idx)) {
                    best = idx;
                }
            }
            if (!greater(k, best)) break;
            exch(k, best);
            k = best;
        }
    }
}
