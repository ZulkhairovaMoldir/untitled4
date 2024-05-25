public class Vertex<V> {
    private V data;
    private Map<Vertex<V>, Double> adjacentVertices = new HashMap<>();

    public Vertex(V data) {
        this.data = data;
    }

    public void addAdjacentVertex(Vertex<V> destination, double weight) {
        adjacentVertices.put(destination, weight);
    }

    public V getData() {
        return data;
    }

    public Map<Vertex<V>, Double> getAdjacentVertices() {
        return adjacentVertices;
    }
}
public class Edge<Vertex> {
    private Vertex source;
    private Vertex dest;
    private Double weight;

    public Edge(Vertex source, Vertex dest, Double weight) {
        this.source = source;
        this.dest = dest;
        this.weight = weight;
    }

    public Vertex getSource() {
        return source;
    }

    public Vertex getDest() {
        return dest;
    }

    public Double getWeight() {
        return weight;
    }
}
public class WeightedGraph<V> {
    private Map<Vertex<V>, List<Edge<Vertex<V>>>> map = new HashMap<>();

    public void addVertex(Vertex<V> vertex) {
        map.put(vertex, new ArrayList<>());
    }

    public void addEdge(Vertex<V> source, Vertex<V> dest, double weight) {
        Edge<Vertex<V>> edge = new Edge<>(source, dest, weight);
        map.get(source).add(edge);
        source.addAdjacentVertex(dest, weight);
    }

    public List<Edge<Vertex<V>>> getEdges(Vertex<V> vertex) {
        return map.get(vertex);
    }

    public Set<Vertex<V>> getVertices() {
        return map.keySet();
    }
}
public class BreadthFirstSearch<V> {
    public void bfs(Vertex<V> start) {
        Set<Vertex<V>> visited = new HashSet<>();
        Queue<Vertex<V>> queue = new LinkedList<>();

        queue.add(start);
        visited.add(start);

        while (!queue.isEmpty()) {
            Vertex<V> current = queue.poll();
            System.out.println("Visited: " + current.getData());

            for (Vertex<V> neighbor : current.getAdjacentVertices().keySet()) {
                if (!visited.contains(neighbor)) {
                    queue.add(neighbor);
                    visited.add(neighbor);
                }
            }
        }
    }
}
public class DijkstraSearch<V> {
    public Map<Vertex<V>, Double> dijkstra(Vertex<V> start) {
        Map<Vertex<V>, Double> distances = new HashMap<>();
        PriorityQueue<Vertex<V>> pq = new PriorityQueue<>(Comparator.comparing(distances::get));

        for (Vertex<V> vertex : start.getAdjacentVertices().keySet()) {
            distances.put(vertex, Double.MAX_VALUE);
        }
        distances.put(start, 0.0);
        pq.add(start);

        while (!pq.isEmpty()) {
            Vertex<V> current = pq.poll();

            for (Map.Entry<Vertex<V>, Double> neighborEntry : current.getAdjacentVertices().entrySet()) {
                Vertex<V> neighbor = neighborEntry.getKey();
                double weight = neighborEntry.getValue();
                double newDist = distances.get(current) + weight;

                if (newDist < distances.get(neighbor)) {
                    distances.put(neighbor, newDist);
                    pq.add(neighbor);
                }
            }
        }

        return distances;
    }
}
public class Main {
    public static void main(String[] args) {

        Vertex<String> v1 = new Vertex<>("A");
        Vertex<String> v2 = new Vertex<>("B");
        Vertex<String> v3 = new Vertex<>("C");
        Vertex<String> v4 = new Vertex<>("D");


        WeightedGraph<String> graph = new WeightedGraph<>();
        graph.addVertex(v1);
        graph.addVertex(v2);
        graph.addVertex(v3);
        graph.addVertex(v4);


        graph.addEdge(v1, v2, 1);
        graph.addEdge(v1, v3, 4);
        graph.addEdge(v2, v3, 2);
        graph.addEdge(v2, v4, 5);
        graph.addEdge(v3, v4, 1);


        System.out.println("Breadth-First Search:");
        BreadthFirstSearch<String> bfs = new BreadthFirstSearch<>();
        bfs.bfs(v1);


        System.out.println("\nDijkstra's Algorithm:");
        DijkstraSearch<String> dijkstra = new DijkstraSearch<>();
        Map<Vertex<String>, Double> distances = dijkstra.dijkstra(v1);

        for (Map.Entry<Vertex<String>, Double> entry : distances.entrySet()) {
            System.out.println("Distance from " + v1.getData() + " to " + entry.getKey().getData() + " is " + entry.getValue());
        }
    }
}
