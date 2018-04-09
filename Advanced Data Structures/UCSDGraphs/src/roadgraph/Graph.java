package roadgraph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;
import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author Dominik Sudwischer
 *
 *         A class which represents a graph and implements some basic pathfinding
 *         algorithms It stores the graph as a Map of node IDs to a Map of
 *         destination node IDs to a PriorityQueue of Edges. This enables us to
 *         quickly find whether there is an edge from one node to another and even to
 *         store multiple edges between the same two nodes in an order depending on
 *         their length.
 *
 */

public class Graph {

	// Member variables
	private Map<Integer, HashMap<Integer, PriorityQueue<Edge>>> graphVertices;
	private final double TOLERANCE = 1e-15;
	private int nextFreeId;

	public enum Algorithm {
		ALGORITHM_DIJKSTRA, ALGORITHM_ASTAR
	}

	// Constructor
	/**
	 * Create a new empty Graph
	 */
	public Graph() {
		graphVertices = new HashMap<Integer, HashMap<Integer, PriorityQueue<Edge>>>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return graphVertices.size();
	}

	/**
	 * Returns the next free ID for a vertex to take
	 * 
	 * @return The next free ID
	 */
	public int getNextId() {
		return nextFreeId;
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The IDs of the vertices in the Graph as a set.
	 */
	public Set<Integer> getVertices() {
		return new HashSet<Integer>(graphVertices.keySet());
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		int numEdges = 0;
		for (HashMap<Integer, PriorityQueue<Edge>> edgeListToDestination : graphVertices
		        .values()) {
			for (PriorityQueue<Edge> edge : edgeListToDestination.values()) {
				numEdges++;
			}
		}
		return numEdges;
	}

	/**
	 * Add another vertex to the graph and assign the next free number to it.
	 * 
	 * @return The number assigned
	 */
	public int addVertex() {
		int next = nextFreeId;
		graphVertices.put(next, new HashMap<Integer, PriorityQueue<Edge>>());
		nextFreeId++;
		return next;
	}

	/**
	 * Check whether a node is included in the graph.
	 * 
	 * @param id
	 *            The id of the node in question.
	 * @return true if the id is contained, else false.
	 */
	private boolean containsVertex(int id) {
		return graphVertices.containsKey(id);
	}

	/**
	 * Adds a directed edge to the graph from node1 to node2.
	 * 
	 * @param node1
	 *            The ID of the start node
	 * @param node2
	 *            The ID of the end node
	 * @param An
	 *            Edge object representing the edge from node1 to node2
	 * @throws IllegalArgumentException
	 */
	public void addEdge(int node1, int node2, Edge edge)
	        throws IllegalArgumentException {
		if (!containsVertex(node1) || !containsVertex(node2)) {
			throw new IllegalArgumentException("At least one of the nodes specified"
			        + "is not contained in the graph.");
		}
		HashMap<Integer, PriorityQueue<Edge>> fromNode1To = graphVertices.get(node1);
		if (!fromNode1To.containsKey(node2)) {
			PriorityQueue<Edge> edgeQueue = new PriorityQueue<Edge>();
			fromNode1To.put(node2, edgeQueue);
		}
		fromNode1To.get(node2).add(edge);
	}

	/**
	 * Helper Method for the exploration step of BFS. This method polls the next
	 * vertex to explore from the queue, adds it to the set of visited vertices and
	 * checks all (unvisited) neighbor vertices. If one of these is the goal, it will
	 * return true, else false. It also stores the parent vertex of each vertex (that
	 * is, the vertex from that it has been explored).
	 * 
	 * @param nodeQueue
	 *            A queue that stores the next nodes to be explored
	 * @param nodeMap
	 *            A map that stores a relationship between locations and node objects
	 *            has been explored
	 * @param nodeSearched
	 *            A technical parameter for the visualization. See writeup on course
	 *            website
	 * @param goal
	 *            The target location to reach
	 * @return true if the target has been found, false otherwise
	 */
	private boolean bfsExplorationStep(Queue<SimpleNode> nodeQueue,
	        Map<GeographicPoint, SimpleNode> nodeMap,
	        Consumer<GeographicPoint> nodeSearched, GeographicPoint goal) {
		SimpleNode currentNode = nodeQueue.poll();
		GeographicPoint currentPoint = currentNode.getLocation();
		nodeSearched.accept(currentPoint);
		currentNode.visit();
		for (Edge edge : graphVertices.get(currentPoint)) {
			GeographicPoint endOfEdge = edge.getEnd();
			if (!nodeMap.containsKey(endOfEdge)) {
				nodeMap.put(endOfEdge, new SimpleNode(endOfEdge));
			}
			SimpleNode neighborNode = nodeMap.get(endOfEdge);
			if (!neighborNode.isVisited()) {
				neighborNode.setParent(currentPoint);
				if (neighborNode.getLocation().distance(goal) <= TOLERANCE) {
					// In this case, we have reached the goal
					return true;
				}
				nodeQueue.add(neighborNode);
			}
		}
		return false;
	}

	/**
	 * A method to reconstruct the optimal (in terms of number of nodes on the path)
	 * path found, including start and end. The method follows the path along the
	 * parent relationship of vertices (in terms of exploration order).
	 * 
	 * @param start
	 *            The location of the start
	 * @param goalNode
	 *            A node object that corresponds to the end of the path
	 * @param nodeMap
	 *            A map that stores the location to Node relationship
	 * @return A LinkedList representing the shortest path.
	 */
	private List<GeographicPoint> reconstructPath(GeographicPoint start,
	        SimpleNode goalNode,
	        Map<GeographicPoint, ? extends SimpleNode> nodeMap) {
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		path.add(0, goalNode.getLocation());
		GeographicPoint currentNode = goalNode.getParent();
		while (currentNode.distance(start) > TOLERANCE) {
			path.add(0, currentNode);
			currentNode = nodeMap.get(currentNode).getParent();
		}
		path.add(0, start);
		return path;
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest (unweighted) path
	 *         from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how to
	 *            use it.
	 * @return The list of intersections that form the shortest (unweighted) path
	 *         from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
	        Consumer<GeographicPoint> nodeSearched) {
		if (!graphVertices.containsKey(start) || !graphVertices.containsKey(goal)) {
			return null; // This is due to the automatic grader which
			             // expects null as an output
			// if no path exists... I am aware this is bad style.
		}
		if (start.distance(goal) <= TOLERANCE) {
			List<GeographicPoint> path = new LinkedList<GeographicPoint>();
			path.add(start);
			return path;
		}
		// Initialize variables
		Map<GeographicPoint, SimpleNode> nodeMap = new HashMap<GeographicPoint, SimpleNode>();
		Queue<SimpleNode> nodeQueue = new LinkedList<SimpleNode>();
		SimpleNode startNode = new SimpleNode(start);
		nodeMap.put(start, startNode);
		startNode.visit();
		nodeQueue.add(startNode);
		// Main loop of BFS
		while (!nodeQueue.isEmpty()) {
			boolean foundGoal = bfsExplorationStep(nodeQueue, nodeMap, nodeSearched,
			        goal);
			if (foundGoal) {
				return reconstructPath(start, nodeMap.get(goal), nodeMap);
			}
		}
		return null; // For grader reasons
	}

	/**
	 * A method that estimates the distance from a specific node to the goal In
	 * Dijkstra's algorithm, this will always be equal to 0. For A*, this should be
	 * an underestimate of the length of the shortest path. By Minkowski's
	 * inequality, the Euclidean distance of two points is a valid choice here.
	 * 
	 * @param pt
	 *            The current location
	 * @param goal
	 *            The target location
	 * @param algorithm
	 *            Either DIJKSTRA or ASTAR
	 * @return The (under)estimated distance between pt and goal
	 */
	private double distanceHeuristic(GeographicPoint pt, GeographicPoint goal,
	        Algorithm algorithm) {
		switch (algorithm) {
		case ALGORITHM_DIJKSTRA:
			return 0.0;
		case ALGORITHM_ASTAR:
			return pt.distance(goal);
		default:
			return 0.0;
		}
	}

	private void enqueueEdge(GeographicPoint goal, Edge edge,
	        GeographicPoint current, Map<GeographicPoint, Node> nodeMap,
	        PriorityQueue<Node> nodeQueue, Algorithm algorithm) {
		GeographicPoint endOfEdge = edge.getEnd();
		if (!nodeMap.containsKey(endOfEdge)) {
			nodeMap.put(endOfEdge, new Node(endOfEdge));
		}
		Node currentNode = nodeMap.get(current);
		Node endOfEdgeNode = nodeMap.get(endOfEdge);
		// Check if the achievable distance is less than the stored
		// distance.
		// Achievable distance: distance to current +
		// length of edge +
		// (estimated) distance from end of edge to goal (0 for
		// dijkstra)
		double distance = currentNode.getDistance() + edge.getWeight();
		double estimatedTotalDistance = distance
		        + distanceHeuristic(endOfEdge, goal, algorithm);
		endOfEdgeNode.setPriority(estimatedTotalDistance);
		if (endOfEdgeNode.getDistance() > distance) {
			endOfEdgeNode.setDistance(distance);
			endOfEdgeNode.setParent(current);
			nodeQueue.add(endOfEdgeNode);
		}
	}

	/**
	 * Perform a step of the A* or Dijsktra exploration routine
	 * 
	 * @param start
	 *            The start vertex
	 * @param goal
	 *            The end vertex
	 * @param currentNode
	 *            The node that is currently being explored
	 * @param nodeQueue
	 *            The PriorityQueue that stores vertices and their priorities
	 * @param pathInfo
	 *            A Map containing a PathInfo object for vertices in the graph
	 * @param algorithm
	 *            Either DIJKSTRA or ASTAR
	 * @param nodeSearched
	 *            A technical parameter for the visualisation. See writeup.
	 * @return true if the shortest path to the goal has been found, else false
	 */
	private boolean advancedExplorationStep(GeographicPoint start,
	        GeographicPoint goal, Node currentNode, PriorityQueue<Node> nodeQueue,
	        Map<GeographicPoint, Node> nodeMap, Algorithm algorithm,
	        Consumer<GeographicPoint> nodeSearched) {
		GeographicPoint current = currentNode.getLocation();
		if (!currentNode.isVisited()) {
			nodeSearched.accept(current);
			currentNode.visit();
			if (current.distance(goal) < TOLERANCE) {
				return true;
			}
			for (Edge edge : graphVertices.get(current)) {
				enqueueEdge(goal, edge, current, nodeMap, nodeQueue, algorithm);
			}
		}
		return false;
	}

	/**
	 * A pathfinding method for Dijkstra's algorithm and A*.
	 * 
	 * @param start
	 *            The start node
	 * @param end
	 *            the goal node
	 * @param algorithm
	 *            Either DIJKSTRA or ASTAR
	 * @param nodeSearched
	 *            A technical parameter
	 * @return The shortest path between start and end as a List<GeographicPoint>
	 */

	private PathInfoWrapper findPath(GeographicPoint start, GeographicPoint goal,
	        Algorithm algorithm, Consumer<GeographicPoint> nodeSearched) {
		// Input checks
		if (!graphVertices.containsKey(start) || !graphVertices.containsKey(goal)) {
			return null; // This is due to the automatic grader which
			             // expects null as an output
			// if no path exists... I am aware this is bad style.
		}
		if (start.distance(goal) <= TOLERANCE) {
			List<GeographicPoint> path = new ArrayList<GeographicPoint>();
			path.add(start);
			return new PathInfoWrapper(path, start.distance(goal));
		}

		// Initialize variables
		Map<GeographicPoint, Node> nodeMap = new HashMap<GeographicPoint, Node>();
		PriorityQueue<Node> explorationQueue = new PriorityQueue<Node>();
		Node startNode = new Node(start);
		startNode.setPriority(0.0);
		startNode.setDistance(0.0);
		explorationQueue.add(startNode);
		nodeMap.put(start, startNode);
		// Actual algorithm
		while (!explorationQueue.isEmpty()) {
			Node currentNode = explorationQueue.poll();
			if (advancedExplorationStep(start, goal, currentNode, explorationQueue,
			        nodeMap, algorithm, nodeSearched)) {
				List<GeographicPoint> path = reconstructPath(start,
				        nodeMap.get(goal), nodeMap);
				return new PathInfoWrapper(path, currentNode.getDistance());
			}
		}
		return null;
	}

	/**
	 * A public interface to find the shortest path. The input and output are exactly
	 * the same as for its private version. This version creates the consumer on its
	 * own.
	 */
	public PathInfoWrapper findPath(GeographicPoint start, GeographicPoint goal,
	        Algorithm algorithm) {
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return findPath(start, goal, algorithm, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start to
	 *         goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start,
	        GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm Note: Due to the
	 * grader, this method does not throw any exceptions.
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how to
	 *            use it.
	 * @return The list of intersections that form the shortest path from start to
	 *         goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start,
	        GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
		return findPath(start, goal, Algorithm.ALGORITHM_DIJKSTRA, nodeSearched)
		        .getPath();
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start to
	 *         goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start,
	        GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how to
	 *            use it.
	 * @return The list of intersections that form the shortest path from start to
	 *         goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start,
	        GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
		return findPath(start, goal, Algorithm.ALGORITHM_ASTAR, nodeSearched)
		        .getPath();
	}

	public static void main(String[] args) {
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");

		// You can use this method for testing.

		/*
		 * Here are some test cases you should try before you attempt the Week 3 End
		 * of Week Quiz, EVEN IF you score 100% on the programming assignment.
		 */

		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println(
		        "Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart, testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,
		        testEnd);

		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println(
		        "Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart, testEnd);
		testroute2 = testMap.aStarSearch(testStart, testEnd);

		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println(
		        "Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart, testEnd);
		testroute2 = testMap.aStarSearch(testStart, testEnd);

		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start, end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start, end);

		// TODO: Remove
		System.out.println(route2.toString());
		Set<GeographicPoint> otherNodes = new HashSet<GeographicPoint>();
		// otherNodes.add(new GeographicPoint(32.8674388,
		// -117.2190213));
		otherNodes.add(end);
		otherNodes.add(new GeographicPoint(32.8697828, -117.2244506));
		TravelingSalespersonProblemSolver tsp = new TravelingSalespersonProblemSolver(
		        theMap, start, otherNodes, true);
		// System.out.println(tsp.getApproximateSolution(10));

		MapGraph tspTestMap = new MapGraph();
		GeographicPoint a = new GeographicPoint(0.0, 0.0);
		GeographicPoint b = new GeographicPoint(1.0, 0.0);
		GeographicPoint c = new GeographicPoint(0.0, -1.0);
		GeographicPoint d = new GeographicPoint(1.0, -1.0);
		tspTestMap.addVertex(a);
		tspTestMap.addVertex(b);
		tspTestMap.addVertex(c);
		tspTestMap.addVertex(d);
		tspTestMap.addEdge(a, b, "", "", 5.0);
		tspTestMap.addEdge(a, c, "", "", 6.0);
		tspTestMap.addEdge(a, d, "", "", 12.0);
		tspTestMap.addEdge(b, a, "", "", 5.0);
		tspTestMap.addEdge(b, c, "", "", 6.0);
		tspTestMap.addEdge(b, d, "", "", 8.0);
		tspTestMap.addEdge(c, a, "", "", 6.0);
		tspTestMap.addEdge(c, b, "", "", 6.0);
		tspTestMap.addEdge(c, d, "", "", 5.0);
		tspTestMap.addEdge(d, a, "", "", 12.0);
		tspTestMap.addEdge(d, b, "", "", 8.0);
		tspTestMap.addEdge(d, c, "", "", 5.0);

		Set<GeographicPoint> otherNodes2 = new HashSet<GeographicPoint>();
		otherNodes2.add(b);
		otherNodes2.add(c);
		otherNodes2.add(d);
		TravelingSalespersonProblemSolver tsp2 = new TravelingSalespersonProblemSolver(
		        tspTestMap, a, otherNodes2, true);
		List<GeographicPoint> solution = tsp2.getApproximateSolution(100);
		System.out.println(solution);
		System.out.println(tsp2.getTotalRouteLength(solution));
	}
}