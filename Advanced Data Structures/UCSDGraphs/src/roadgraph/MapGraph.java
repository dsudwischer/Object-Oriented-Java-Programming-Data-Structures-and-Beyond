/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


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
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between edges
 * The graph is stored as a HashMap from GeographicPoint to Set<Edge>. Each key in the
 * HashMap is a vertex and the corresponding Set<Edge> contains all edges that go out
 * of that vertex.
 * Using sets allows for fast (constant time) checking whether a vertex or an edge is already
 * contained in the graph. Also, sets fit well into a setting of unique keys (vertices).
 */
public class MapGraph
{
	// Member variables
	private Map<GeographicPoint, Set<Edge>> edges;
	private final double TOLERANCE = 1e-15;
	public enum Algorithm
	{
		ALGORITHM_DIJKSTRA, ALGORITHM_ASTAR
	}

	// Constructor
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		edges = new HashMap<GeographicPoint, Set<Edge>>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return edges.size();
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		Set<GeographicPoint> vertices = new HashSet<GeographicPoint>(edges.size());
		for(GeographicPoint vertex : edges.keySet())
		{
			vertices.add(vertex);
		}
		return vertices;
	}

	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		int numEdges = 0;
		for(Set<Edge> edgeSet : edges.values())
		{
			numEdges += edgeSet.size();
		}
		return numEdges;
	}


	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if(location == null || edges.containsKey(location))
		{
			return false;
		}
		edges.put(location,  new HashSet<Edge>());
		return true;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException
	{
		if(from == null || to == null || roadName == null || roadType == null ||
				length < 0 || !edges.containsKey(from) || !edges.containsKey(to))
		{
			throw new IllegalArgumentException();
		}
		Edge edge = new Edge(from, to, roadName, roadType, length);
		edges.get(from).add(edge);
	}

	/** Helper Method for the exploration step of BFS. This method polls the next vertex
	 * to explore from the queue, adds it to the set of visited vertices and checks all
	 * (unvisited) neighbor vertices. If one of these is the goal, it will return true, else
	 * false. It also stores the parent vertex of each vertex (that is, the vertex from that 
	 * it has been explored).
	 * 
	 * @param nodeQueue A queue that stores the next nodes to be explored
	 * @param nodeMap A map that stores a relationship between locations and node objects
	 * has been explored
	 * @param nodeSearched A technical parameter for the visualization. See writeup
	 * on course website
	 * @param goal The target location to reach
	 * @return true if the target has been found, false otherwise
	 */
	private boolean bfsExplorationStep(Queue<SimpleNode> nodeQueue,
			Map<GeographicPoint, SimpleNode> nodeMap,
			Consumer<GeographicPoint> nodeSearched,
			GeographicPoint goal)
	{
		SimpleNode currentNode = nodeQueue.poll();
		GeographicPoint currentPoint = currentNode.getLocation();
		nodeSearched.accept(currentPoint);
		currentNode.visit();
		for(Edge edge : edges.get(currentPoint))
		{
			GeographicPoint endOfEdge = edge.getEnd();
			if(!nodeMap.containsKey(endOfEdge))
			{
				nodeMap.put(endOfEdge, new SimpleNode(endOfEdge));
			}
			SimpleNode neighborNode = nodeMap.get(endOfEdge);
			if(!neighborNode.isVisited())
			{
				neighborNode.setParent(currentPoint);
				if(neighborNode.getLocation().distance(goal) <= TOLERANCE)
				{
					// In this case, we have reached the goal
					return true;
				}
				nodeQueue.add(neighborNode);
			}
		}
		return false;
	}

	
	/** A method to reconstruct the optimal (in terms of number of nodes on the path)
	 * path found, including start and end. The method follows the path
	 * along the parent relationship of vertices (in terms of exploration order).
	 * 
	 * @param start The location of the start
	 * @param goalNode A node object that corresponds to the end of the path
	 * @param nodeMap A map that stores the location to Node relationship
	 * @return A LinkedList representing the shortest path.
	 */
	private List<GeographicPoint> reconstructPath(GeographicPoint start,
			SimpleNode goalNode, Map<GeographicPoint, ? extends SimpleNode> nodeMap)
	{
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		path.add(0, goalNode.getLocation());
		GeographicPoint currentNode = goalNode.getParent();
		while(currentNode.distance(start) > TOLERANCE)
		{
			path.add(0, currentNode);
			currentNode = nodeMap.get(currentNode).getParent();
		}
		path.add(0, start);
		return path;
	}


	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {};
		return bfs(start, goal, temp);
	}

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{	
		if(!edges.containsKey(start) || !edges.containsKey(goal))
		{
			return null; // This is due to the automatic grader which expects null as an output
			// if no path exists... I am aware this is bad style.
		}
		if(start.distance(goal) <= TOLERANCE)
		{
			List<GeographicPoint> path = new LinkedList<GeographicPoint>();
			path.add(start);
			return path;
		}
		// Initialize variables
		Map<GeographicPoint, SimpleNode> nodeMap =
				new HashMap<GeographicPoint, SimpleNode>();
		Queue<SimpleNode> nodeQueue = new LinkedList<SimpleNode>();
		SimpleNode startNode = new SimpleNode(start);
		nodeMap.put(start, startNode);
		startNode.visit();
		nodeQueue.add(startNode);
		// Main loop of BFS
		while(!nodeQueue.isEmpty())
		{
			boolean foundGoal = bfsExplorationStep(nodeQueue, nodeMap, nodeSearched, goal);
			if(foundGoal)
			{
				return reconstructPath(start, nodeMap.get(goal), nodeMap);
			}
		}
		return null; // For grader reasons
	}

	/** A method that estimates the distance from a specific node to the goal
	 * In Dijkstra's algorithm, this will always be equal to 0. For A*, this 
	 * should be an underestimate of the length of the shortest path.
	 * By Minkowski's inequality, the Euclidean distance of two points is a valid
	 * choice here.
	 * 
	 * @param pt The current location
	 * @param goal The target location
	 * @param algorithm Either DIJKSTRA or ASTAR
	 * @return The (under)estimated distance between pt and goal
	 */
	private double distanceHeuristic(GeographicPoint pt, GeographicPoint goal,
			Algorithm algorithm)
	{
		switch(algorithm)
		{
		case ALGORITHM_DIJKSTRA: return 0.0;
		case ALGORITHM_ASTAR: return pt.distance(goal); 	
		default: return 0.0;
		}
	}

	
	private void enqueueEdge(GeographicPoint goal, Edge edge, GeographicPoint current,
			Map<GeographicPoint, Node> nodeMap,
			PriorityQueue<Node> nodeQueue,
			Algorithm algorithm)
	{
			GeographicPoint endOfEdge = edge.getEnd();
			if(!nodeMap.containsKey(endOfEdge))
			{
				nodeMap.put(endOfEdge, new Node(endOfEdge));
			}
			Node currentNode = nodeMap.get(current);
			Node endOfEdgeNode = nodeMap.get(endOfEdge);
			// Check if the achievable distance is less than the stored
			// distance.
			// Achievable distance: distance to current +
			// length of edge +
			// (estimated) distance from end of edge to goal (0 for dijkstra)
			double distance = currentNode.getDistance()
					+ edge.getWeight();
			double estimatedTotalDistance = distance +
					distanceHeuristic(endOfEdge, goal, algorithm);
			endOfEdgeNode.setPriority(estimatedTotalDistance);
			if(endOfEdgeNode.getDistance() > distance)
			{
				endOfEdgeNode.setDistance(distance);
				endOfEdgeNode.setParent(current);
				nodeQueue.add(endOfEdgeNode);
			}
	}
	
	/** Perform a step of the A* or Dijsktra exploration routine
	 * 
	 * @param start The start vertex
	 * @param goal The end vertex
	 * @param currentNode The node that is currently being explored
	 * @param nodeQueue The PriorityQueue that stores vertices and their priorities
	 * @param pathInfo A Map containing a PathInfo object for vertices in the graph
	 * @param algorithm Either DIJKSTRA or ASTAR
	 * @param nodeSearched A technical parameter for the visualisation. See writeup.
	 * @return true if the shortest path to the goal has been found, else false
	 */
	private boolean advancedExplorationStep(GeographicPoint start,
			GeographicPoint goal,
			Node currentNode,
			PriorityQueue<Node> nodeQueue,
			Map<GeographicPoint, Node> nodeMap,
			Algorithm algorithm,
			Consumer<GeographicPoint> nodeSearched)
	{
		GeographicPoint current = currentNode.getLocation();
		if(!currentNode.isVisited())
		{
			nodeSearched.accept(current);
			currentNode.visit();
			if(current.distance(goal) < TOLERANCE)
			{
				return true;
			}
			for(Edge edge : edges.get(current))
			{
				enqueueEdge(goal, edge, current, nodeMap, nodeQueue, algorithm);
			}
		}
		return false;
	}

	/** A pathfinding method for Dijkstra's algorithm and A*.
	 * 
	 * @param start The start node
	 * @param end the goal node
	 * @param algorithm Either DIJKSTRA or ASTAR
	 * @param nodeSearched A technical parameter
	 * @return The shortest path between start and end as a List<GeographicPoint>
	 */

	private List<GeographicPoint> findPath(GeographicPoint start, GeographicPoint goal,
			Algorithm algorithm, Consumer<GeographicPoint> nodeSearched)
	{
		// Input checks
		if(!edges.containsKey(start) || !edges.containsKey(goal))
		{
			return null; // This is due to the automatic grader which expects null as an output
			// if no path exists... I am aware this is bad style.
		}
		if(start.distance(goal) <= TOLERANCE)
		{
			List<GeographicPoint> path = new LinkedList<GeographicPoint>();
			path.add(start);
			return path;
		}

		// Initialize variables
		Map<GeographicPoint, Node> nodeMap =
				new HashMap<GeographicPoint, Node>();
		PriorityQueue<Node> explorationQueue = new PriorityQueue<Node>();
		Node startNode = new Node(start);
		startNode.setPriority(0.0);
		startNode.setDistance(0.0);
		explorationQueue.add(startNode);
		nodeMap.put(start, startNode);
		// Actual algorithm
		while(!explorationQueue.isEmpty())
		{
			Node currentNode = explorationQueue.poll();
			if(advancedExplorationStep(start, goal, currentNode, explorationQueue, nodeMap,
					algorithm, nodeSearched))
			{
				List<GeographicPoint> path = reconstructPath(start, (SimpleNode) nodeMap.get(goal), nodeMap);
				return path;
			}
		}		
		return null;
	}


	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {};
		return dijkstra(start, goal, temp);
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * Note: Due to the grader, this method does not throw any exceptions.
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
			GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched)
	{
		return findPath(start, goal, Algorithm.ALGORITHM_DIJKSTRA, nodeSearched);
	}


	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {};
		return aStarSearch(start, goal, temp);
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
			GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched)
	{
		return findPath(start, goal, Algorithm.ALGORITHM_ASTAR, nodeSearched);
	}


	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");

		// You can use this method for testing.  


		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */

		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);


		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);


		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);



		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);


		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
	}

}
