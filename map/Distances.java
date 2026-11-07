/*************************************************************************
 *  Compilation:  javac Distances.java
 *  Execution:    java Distances file < input.txt
 *  Dependencies: EuclideanGraph.java Dijkstra.java In.java StdIn.java
 *
 *  Reads in a map from a file, and repeatedly reads in two integers s
 *  and d from standard input, and prints the distance of the shortest
 *  path from s to d to standard output.
 *
 ****************************************************************************/


public class Distances {

    public static void main(String[] args) {

        // read in the graph from a file
        In graphin = new In(args[0]);
        EuclideanGraph G = new EuclideanGraph(graphin);
        System.err.println("Done reading the graph " + args[0]);

        // optional second argument "ch" answers with the contraction hierarchy
        boolean useCH = args.length > 1 && args[1].equalsIgnoreCase("ch");
        Dijkstra dijkstra = useCH ? null : new Dijkstra(G);
        ContractionHierarchy ch = null;
        if (useCH) {
            long t0 = System.currentTimeMillis();
            ch = new ContractionHierarchy(G);
            System.err.println("Built contraction hierarchy in "
                               + (System.currentTimeMillis() - t0) + " ms, "
                               + ch.shortcutCount() + " shortcuts");
        }
        System.err.println("Enter query pairs from stdin");

        // read in the s-d pairs from standard input
        while(!StdIn.isEmpty()) {
            int s = StdIn.readInt();
            int d = StdIn.readInt();
            System.out.println(useCH ? ch.distance(s, d) : dijkstra.distance(s, d));
        }
    }
}
