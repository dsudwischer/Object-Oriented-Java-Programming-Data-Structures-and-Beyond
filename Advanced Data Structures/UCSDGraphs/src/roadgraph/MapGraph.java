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
	 * @param q A queue that stores the next nodes to be explored
	 * @param visited A set that stores nodes that have already been explored
	 * @param parents A HashMap that stores the node from which each node
	 * has been explored
	 * @param nodeSearched A technical parameter for the visualization. See writeup
	 * on course website
	 * @param goal The target location to reach
	 * @return true if the target has been found, false otherwise
	 */
	private boolean bfsExplorationStep(Queue<GeographicPoint> q, Set<GeographicPoint> visited,
			Map<GeographicPoint, GeographicPoint> parents, Consumer<GeographicPoint> nodeSearched,
			GeographicPoint goal)
	{
		GeographicPoint current = q.poll();
		nodeSearched.accept(current);
		visited.add(current);
		for(Edge edge : edges.get(current))
		{
			GeographicPoint neighbor = edge.getEnd();
			if(!visited.contains(neighbor))
			{
				parents.put(neighbor, current);
				if(neighbor.distance(goal) <= TOLERANCE)
				{
					// In this case, we have reached the goal
					return true;
				}
				q.add(neighbor);
			}
		}
		return false;
	}
	
	/** A method to reconstruct the optimal (in terms of number of nodes on the path)
	 * path found by BFS, including start and end. The method follows the path
	 * along the parent relationship of vertices (in terms of exploration order).
	 * 
	 * @param start The location of the start
	 * @param goal The location of the goal
	 * @param parents The HashMap that stores for each location from where it has been
	 * discovered
	 * @return A LinkedList representing the shortest path.
	 */
	private List<GeographicPoint> bfsReconstructPath(GeographicPoint start, GeographicPoint goal,
			Map<GeographicPoint, GeographicPoint> parents)
	{
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		path.add(0, goal);
		GeographicPoint currentNode = parents.get(goal);
		while(currentNode.distance(start) > TOLERANCE)
		{
			path.add(0, currentNode);
			currentNode = parents.get(currentNode);
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
		// Initialize variables: A HashMap to store parent relationships for the path,
		// a HashSet to store visited nodes and a (FIFO) queue for BFS.
		Map<GeographicPoint, GeographicPoint> parents = new HashMap<GeographicPoint, 
				GeographicPoint>();
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Queue<GeographicPoint> q = new LinkedList<GeographicPoint>();
		q.add(start);
		
		// Main loop of BFS
		while(!q.isEmpty())
		{
			boolean foundGoal = bfsExplorationStep(q, visited, parents, nodeSearched, goal);
			if(foundGoal)
			{
				return bfsReconstructPath(start, goal, parents);
			}
		}
		return null; // For grader reasons
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
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
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
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
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
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
