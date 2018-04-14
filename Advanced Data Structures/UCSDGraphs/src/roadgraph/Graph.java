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

public abstract class Graph {

	// Member variables
	private final Map<Integer, HashMap<Integer, PriorityQueue<Edge>>> graphVertices;
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
			numEdges += edgeListToDestination.values().size();
		}
		return numEdges;
	}

	/**
	 * Add another vertex to the graph and assign the next free number to it.
	 * 
	 * @return The number assigned
	 */
	public int addVertex() {
		int next = getNextId();
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
	 * @param nodeIdQueue
	 *            A queue that stores the next nodes' IDs to be explored
	 * @param nodeMap
	 *            A map that stores a relationship between node IDs and the
	 *            SimpleNode objects
	 * @param nodeSearched
	 *            A technical parameter for the visualization. See writeup on course
	 *            website
	 * @param goalNodeId
	 *            The node id of the target location to reach
	 * @param visited
	 *            A set of visited nodes
	 * @param parents
	 *            A Map that contains parent relationships between nodes
	 * @param locations
	 *            A parameter that is required to support the "nodeSearched"
	 *            parameter. It contains A mapping of node IDs to geographic
	 *            locations.
	 * @return true if the target has been found, false otherwise
	 */
	private boolean bfsExplorationStep(Queue<Integer> nodeIdQueue,
			Consumer<GeographicPoint> nodeSearched, int goalNodeId,
			Set<Integer> visited, Map<Integer, Integer> parents,
			Map<Integer, GeographicPoint> locations) {
		int currentNodeId = nodeIdQueue.poll();
		try {
			nodeSearched.accept(locations.get(currentNodeId));
		}
		catch (Exception e) {
		}
		visited.add(currentNodeId);
		for (int neighborId : graphVertices.get(currentNodeId).keySet()) {
			if (!visited.contains(neighborId)) {
				parents.put(neighborId, currentNodeId);
				if (neighborId == goalNodeId) {
					// In this case, we have reached the goal
					return true;
				}
				nodeIdQueue.add(neighborId);
			}
		}
		return false;
	}

	/**
	 * A method to reconstruct the optimal (in terms of number of nodes on the path)
	 * path found, including start and end. The method follows the path along the
	 * parent relationship of vertices (in terms of exploration order).
	 * 
	 * @param startId
	 *            The ID of the start node
	 * @param goalId
	 *            The ID of the goal node
	 * @param parents
	 *            A map that stores parent relationships between nodes
	 * @return A LinkedList of node IDs representing the shortest path.
	 */
	private List<Integer> reconstructPath(int startId, int goalId,
			Map<Integer, Integer> parents) {
		List<Integer> path = new LinkedList<Integer>();
		path.add(goalId);
		int currentNodeId = parents.get(goalId);
		while (currentNodeId != startId) {
			path.add(0, currentNodeId);
			currentNodeId = parents.get(currentNodeId);
		}
		path.add(0, startId);
		return path;
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The node id of the starting location
	 * @param goal
	 *            The node id of the goal location
	 * @return The list of intersections that form the shortest (unweighted) path
	 *         from start to goal (including both start and goal).
	 */
	public List<Integer> bfs(Integer start, Integer goal,
			Map<Integer, GeographicPoint> locations) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp, locations);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param startId
	 *            The starting location node ID
	 * @param goalId
	 *            The goal location node ID
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how to
	 *            use it.
	 * @return The list of intersections that form the shortest (unweighted) path
	 *         from start to goal (including both start and goal).
	 */
	public List<Integer> bfs(int startId, int goalId,
			Consumer<GeographicPoint> nodeSearched,
			Map<Integer, GeographicPoint> locations) {
		if (!graphVertices.containsKey(startId)
				|| !graphVertices.containsKey(goalId)) {
			return null; // This is due to the automatic grader which
			// expects null as an output
			// if no path exists... I am aware this is bad style.
		}
		if (startId == goalId) {
			List<Integer> path = new LinkedList<Integer>();
			path.add(startId);
			return path;
		}
		// Initialize variables
		Queue<Integer> nodeIdQueue = new LinkedList<Integer>();
		Set<Integer> visited = new HashSet<Integer>();
		Map<Integer, Integer> parents = new HashMap<Integer, Integer>();
		nodeIdQueue.add(startId);
		// Main loop of BFS
		while (!nodeIdQueue.isEmpty()) {
			boolean foundGoal = bfsExplorationStep(nodeIdQueue, nodeSearched, goalId,
					visited, parents, locations);
			if (foundGoal) {
				return reconstructPath(startId, goalId, parents);
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
	private double distanceHeuristic(int from, int to, Algorithm algorithm) {
		switch (algorithm) {
		case ALGORITHM_DIJKSTRA:
			return 0.0;
		case ALGORITHM_ASTAR:
			return distanceEstimate(from, to);
		default:
			return 0.0;
		}
	}

	/**
	 * A method to estimate the distance between two nodes. This must not be an
	 * overestimate.
	 * 
	 * @param location1
	 *            The first node id
	 * @param location2
	 *            The second node id
	 * @return The distance estimate given between location1 and location2
	 */
	protected abstract double distanceEstimate(int location1, int location2);

	/**
	 * A method to enqeue an Edge to the priority queue for Dijkstra or A*.
	 * 
	 * @param goalId
	 *            The goal node id
	 * @param currentNodeId
	 *            The Id of the current node
	 * @param nextNodeId
	 *            The Id of the node at the end of the next edge to consider
	 * @param nodeQueue
	 *            The queue of nodes to explore
	 * @param distances
	 *            A map that contains distances from the start node to other nodes
	 * @param parents
	 *            A map that stores the parent of each node on the shortest path from
	 *            the start node to that particular node
	 * @param algorithm
	 *            Either DIJSKTRA or ASTAR
	 */
	private void enqueueNode(int goalId, int currentNodeId, int nextNodeId,
			PriorityQueue<PriorityVertex> nodeQueue, Map<Integer, Double> distances,
			Map<Integer, Integer> parents, Algorithm algorithm) {
		// Check if the achievable distance is less than the stored
		// distance.
		// Achievable distance: distance to current +
		// length of edge +
		// (estimated) distance from end of edge to goal (0 for
		// Dijkstra)
		double distance = distances.get(currentNodeId) + graphVertices
				.get(currentNodeId).get(nextNodeId).peek().getWeight();
		double estimatedTotalDistance = distance
				+ distanceHeuristic(nextNodeId, goalId, algorithm);
		if (!distances.containsKey(nextNodeId)
				|| distances.get(nextNodeId) > distance) {
			distances.put(nextNodeId, distance);
			parents.put(nextNodeId, currentNodeId);
			nodeQueue.add(new PriorityVertex(nextNodeId, estimatedTotalDistance));
		}
	}

	/**
	 * Perform a step of the A* or Dijsktra exploration routine
	 * 
	 * @param startId
	 *            The start vertex
	 * @param goalId
	 *            The end vertex
	 * @param currentNodeId
	 *            The node that is currently being explored
	 * @param nodeQueue
	 *            The PriorityQueue that stores vertices and their priorities
	 * @param visited
	 *            A set of visited nodes
	 * @param parents
	 *            A Map that contains parent relationships between nodes
	 * @param distances
	 *            A map that contains distances from the start node to other nodes
	 * @param parents
	 *            A map that stores the parent of each node on the shortest path from
	 *            the start node to that particular node
	 * @param algorithm
	 *            Either DIJKSTRA or ASTAR
	 * @param nodeSearched
	 *            A technical parameter for the visualisation. See writeup.
	 * @param locations
	 *            A parameter that is required to support the "nodeSearched"
	 *            parameter. It contains A mapping of node IDs to geographic
	 *            locations.
	 * @return true if the shortest path to the goal has been found, else false
	 */
	private boolean advancedExplorationStep(int startId, int goalId,
			int currentNodeId, PriorityQueue<PriorityVertex> nodeQueue,
			Set<Integer> visited, Map<Integer, Double> distances,
			Map<Integer, Integer> parents, Algorithm algorithm,
			Consumer<GeographicPoint> nodeSearched,
			Map<Integer, GeographicPoint> locations) {
		if (!visited.contains(currentNodeId)) {
			try {
				nodeSearched.accept(locations.get(currentNodeId));
			}
			catch (Exception e) {
			}
			visited.add(currentNodeId);
			if (currentNodeId == goalId) {
				return true;
			}
			for (int nextNodeId : graphVertices.get(currentNodeId).keySet()) {
				enqueueNode(goalId, currentNodeId, nextNodeId, nodeQueue, distances,
						parents, algorithm);
			}
		}
		return false;
	}

	/**
	 * A pathfinding method for Dijkstra's algorithm and A*.
	 * 
	 * @param startId
	 *            The start node ID
	 * @param goalId
	 *            the goal node ID
	 * @param algorithm
	 *            Either DIJKSTRA or ASTAR
	 * @param nodeSearched
	 *            A technical parameter
	 * @return The shortest path between start and end as a List<Integer>
	 */

	private PathInfoWrapper findPath(int startId, int goalId, Algorithm algorithm,
			Consumer<GeographicPoint> nodeSearched,
			Map<Integer, GeographicPoint> locations) {
		// Input checks
		if (!graphVertices.containsKey(startId)
				|| !graphVertices.containsKey(goalId)) {
			return null; // This is due to the automatic grader which
			// expects null as an output
			// if no path exists... I am aware this is bad style.
		}
		if (startId == goalId) {
			List<Integer> path = new ArrayList<Integer>();
			path.add(startId);
			return new PathInfoWrapper(path, 0.0);
		}

		// Initialize variables
		PriorityQueue<PriorityVertex> explorationQueue = new PriorityQueue<PriorityVertex>();
		Set<Integer> visited = new HashSet<Integer>();
		Map<Integer, Integer> parents = new HashMap<Integer, Integer>();
		Map<Integer, Double> distances = new HashMap<Integer, Double>();
		distances.put(startId, 0.0);
		PriorityVertex startNode = new PriorityVertex(startId, 0.0);
		explorationQueue.add(startNode);
		// Actual algorithm
		while (!explorationQueue.isEmpty()) {
			PriorityVertex currentNode = explorationQueue.poll();
			if (advancedExplorationStep(startId, goalId, currentNode.getId(),
					explorationQueue, visited, distances, parents, algorithm,
					nodeSearched, locations)) {
				List<Integer> path = reconstructPath(startId, goalId, parents);
				return new PathInfoWrapper(path, distances.get(goalId));
			}
		}
		return null;
	}

	/**
	 * A wrapper method to find the shortest path between two nodes
	 * 
	 * @param startId
	 *            The ID of the start node
	 * @param goalId
	 *            The ID of the goal node
	 * @param algorithm
	 *            Either DIJKSTRA or ASTAR
	 * @param locations
	 *            A parameter to support the GUI
	 * @return
	 */
	public PathInfoWrapper findPath(int startId, int goalId, Algorithm algorithm,
			Map<Integer, GeographicPoint> locations) {
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return findPath(startId, goalId, algorithm, temp, locations);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param startId
	 *            The ID of the starting location
	 * @param goalId
	 *            The ID of the goal location
	 * @param locations
	 *            A parameter to support the GUI
	 * @return The list of intersections that form the shortest path from start to
	 *         goal (including both start and goal).
	 */
	public List<Integer> dijkstra(int startId, int goalId,
			Map<Integer, GeographicPoint> locations) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(startId, goalId, temp, locations);
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
	 * @param locations
	 *            A parameter to support the GUI
	 * @return The list of intersections that form the shortest path from start to
	 *         goal (including both start and goal).
	 */
	public List<Integer> dijkstra(int startId, int goalId,
			Consumer<GeographicPoint> nodeSearched,
			Map<Integer, GeographicPoint> locations) {
		return findPath(startId, goalId, Algorithm.ALGORITHM_DIJKSTRA, locations)
				.getPath();
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param startId
	 *            The starting location
	 * @param goalId
	 *            The goal location
	 * @param locations
	 *            A parameter to support the GUI
	 * @return The list of intersections that form the shortest path from start to
	 *         goal (including both start and goal).
	 */
	public List<Integer> aStarSearch(int startId, int goalId,
			Map<Integer, GeographicPoint> locations) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(startId, goalId, temp, locations);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param startId
	 *            The starting location
	 * @param goalId
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how to
	 *            use it.
	 * @param locations
	 *            A parameter to support the GUI
	 * @return The list of intersections that form the shortest path from start to
	 *         goal (including both start and goal).
	 */
	public List<Integer> aStarSearch(int startId, int goalId,
			Consumer<GeographicPoint> nodeSearched,
			Map<Integer, GeographicPoint> locations) {
		return findPath(startId, goalId, Algorithm.ALGORITHM_ASTAR, locations)
				.getPath();
	}
}