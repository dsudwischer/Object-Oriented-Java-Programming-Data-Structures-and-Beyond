/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import geography.GeographicPoint;
import util.GraphLoader;

/**
 * An extension of the abstract Graph Class
 * 
 * @author UCSD MOOC development team and YOU
 */
public class MapGraph extends Graph {
	// Member variables
	Map<Integer, GeographicPoint> locations;
	Map<GeographicPoint, Integer> locationToIdMap;

	// Constructor
	public MapGraph() {
		super();
		locations = new HashMap<Integer, GeographicPoint>();
		locationToIdMap = new HashMap<GeographicPoint, Integer>();
	}

	/**
	 * A method to retrieve a node ID
	 * 
	 * @param point
	 *            The point whose ID to retrieve
	 * @return The ID of the point
	 */
	public Integer getNodeId(GeographicPoint point) {
		return locationToIdMap.get(point);
	}

	/**
	 * A method to get the node with the specified ID
	 * 
	 * @param id
	 *            The id of the node to retrieve
	 * @return A reference to the point with the specified ID
	 */
	public GeographicPoint getNodeById(int id) {
		return locations.get(id);
	}

	/**
	 * A method to add a vertex to the graph
	 * 
	 * @param point
	 *            The point to add as a vertex
	 */
	public void addVertex(GeographicPoint point) {
		int id = super.getNextId();
		locations.put(id, point);
		locationToIdMap.put(point, id);
		super.addVertex();
	}

	/**
	 * A method to add a (directed) edge between two nodes
	 * 
	 * @param from
	 *            The start location
	 * @param to
	 *            The goal location
	 * @param roadName
	 *            The name of the road
	 * @param roadType
	 *            The type of the road
	 * @param weight
	 *            The weight (i.e. cost associated with taking the road) of the road
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double weight) {
		Edge edge = new Edge(weight);
		super.addEdge(getNodeId(from), getNodeId(to), edge);
	}

	/**
	 * A method to convert a List<Integer> to a List<GeographicPoint> of
	 * corresponding locations.
	 * 
	 * @param nodeIds
	 *            A List<Integer> of node IDs
	 * @return A List<GeographicPoint> with each entry corresponding to the location
	 *         of that node id
	 */
	private List<GeographicPoint> buildPathFromNodeIds(List<Integer> nodeIds) {
		System.out.println(nodeIds);
		if (nodeIds == null) {
			return null; // For grader reasons...
		}
		List<GeographicPoint> path = new ArrayList<GeographicPoint>(nodeIds.size());
		for (int nodeId : nodeIds) {
			path.add(getNodeById(nodeId));
		}
		return path;
	}

	/**
	 * A method to find the shortest path between two nodes with breadth first search
	 * 
	 * @param start
	 *            The start location
	 * @param end
	 *            The goal location
	 * @return The shortest path as a list of GeographicPoint instances
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint end) {
		List<Integer> intPath = bfs(getNodeId(start), getNodeId(end), locations);
		return buildPathFromNodeIds(intPath);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see roadgraph.Graph#distanceEstimate(int, int)
	 */
	@Override
	protected double distanceEstimate(int location1, int location2) {
		return getNodeById(location1).distance(getNodeById(location2));
	}

	/**
	 * A method that returns the shortest path from one location to another
	 * 
	 * @param start
	 *            The start location
	 * @param end
	 *            The goal
	 * @return A list that resembles the graph
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start,
			GeographicPoint end) {
		List<Integer> nodeIds = aStarSearch(getNodeId(start), getNodeId(end),
				locations);
		return buildPathFromNodeIds(nodeIds);
	}

	/**
	 * A method that returns the shortest path from one location to another
	 * 
	 * @param start
	 *            The start location
	 * @param end
	 *            The goal
	 * @return A list that resembles the graph
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start,
			GeographicPoint end) {
		List<Integer> nodeIds = dijkstra(getNodeId(start), getNodeId(end),
				locations);
		return buildPathFromNodeIds(nodeIds);
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
		Set<Integer> otherNodes = new HashSet<Integer>();
		// otherNodes.add(new GeographicPoint(32.8674388, -117.2190213));
		otherNodes.add(theMap.getNodeId(end));
		otherNodes.add(
				theMap.getNodeId(new GeographicPoint(32.8697828, -117.2244506)));
		TravelingSalespersonProblemSolver tsp = new TravelingSalespersonProblemSolver(
				theMap, theMap.getNodeId(start), otherNodes, true);
		System.out.println(tsp.getApproximateSolution(10));

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

		Set<Integer> otherNodes2 = new HashSet<Integer>();
		otherNodes2.add(tspTestMap.getNodeId(b));
		otherNodes2.add(tspTestMap.getNodeId(c));
		otherNodes2.add(tspTestMap.getNodeId(d));
		TravelingSalespersonProblemSolver tsp2 = new TravelingSalespersonProblemSolver(
				(Graph) tspTestMap, tspTestMap.getNodeId(a), otherNodes2, true);
		List<Integer> solution = tsp2.getApproximateSolution(100);
		System.out.println(solution);
		System.out.println(tsp2.getTotalRouteLength(solution));

	}
}
