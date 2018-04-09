package roadgraph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import geography.GeographicPoint;
import roadgraph.MapGraph.Algorithm;

/**
 * This class computes approximate solutions to the traveling salesperson problem
 * (TSP) using the greedy approach and the 2-Opt algorithm.
 * Note that this implementation does not care whether a node has been visited more
 * than once.
 * 
 * @author Dominik Sudwischer
 *
 */
public class TravelingSalespersonProblemSolver
{
	// Member variables
	private final GeographicPoint startNode;
	private final Set<GeographicPoint> otherNodes;
	private final boolean saveDistances;
	private Map<Pair<GeographicPoint, GeographicPoint>, Double> distances;
	private final MapGraph mapGraph;


	// Constructor
	public TravelingSalespersonProblemSolver(MapGraph mapGraph, GeographicPoint startNode,
			Set<GeographicPoint> otherNodes, boolean saveDistances)
	{
		this.mapGraph = mapGraph;
		this.startNode = startNode;
		this.otherNodes = otherNodes;
		this.saveDistances = saveDistances;
		if(saveDistances)
		{
			distances = new HashMap<Pair<GeographicPoint, GeographicPoint>, Double>();
		}
	}

	public GeographicPoint getStartNode()
	{
		return startNode;
	}

	/**
	 * This method returns a copy of the set of nodes that have to be visited by
	 * the traveling salesperson.
	 * @return A set of nodes that have to be visited, excluding the starting location
	 */
	public Set<GeographicPoint> getOtherNodes()
	{
		Set<GeographicPoint> otherNodesCopy = new HashSet<GeographicPoint>();
		for(GeographicPoint pt : otherNodes)
		{
			otherNodesCopy.add(pt);
		}
		return otherNodesCopy;
	}

	/**
	 * This method uses A* to compute the shortest path from start to end an returns
	 * its length.
	 * @param start The starting location
	 * @param end The goal location
	 * @return The distance from start to end on the shortest path. If no path
	 * exists, infinity is returned.
	 */
	private double computeDistanceWithAStar(GeographicPoint start,
			GeographicPoint end)
	{
		PathInfoWrapper pathInfo = mapGraph.findPath(start, end,
				Algorithm.ALGORITHM_ASTAR);
		if(pathInfo == null)
		{
			// If no path exists, the distance is infinity
			return Double.POSITIVE_INFINITY;
		}
		else
		{
			return pathInfo.getDistance();
		}
	}

	/**
	 * @param start The starting location
	 * @param end The goal location
	 * @param distance The distance from start to end
	 */
	private void saveDistance(GeographicPoint start,
			GeographicPoint end, double distance)
	{
		distances.put(new Pair<GeographicPoint, GeographicPoint>(start, end), distance);
	}

	/**
	 * This method tries to retrieve the distance between start and end. If it is
	 * not stored in the distances map, it will be computed.
	 * @param start the starting location
	 * @param end the goal location
	 * @return The distance from start to end. If no path exists, this returns infinity.
	 */
	private double getDistance(GeographicPoint start,
			GeographicPoint end)
	{
		Double distance = null;
		Pair<GeographicPoint, GeographicPoint> routeStartEndPair = 
				new Pair<GeographicPoint, GeographicPoint>(start, end);
		if(saveDistances)
		{
			distance = distances.get(routeStartEndPair);
		}
		if(distance == null)
		{
			// In this case, we either do not save distances or have not stored
			// this distance before
			distance = computeDistanceWithAStar(start, end);
		}
		if(saveDistances)
		{
			/* 
			 * TODO: It might be a good idea to actually store not only the path 
			 * from start to end, but also between each pair of middle points.
			 * However, I am uncertain about the performance benefits of this.
			 * It would require O(path.size()^2) operations to store everything
			 * along the path and it would require modifications to the mapGraph
			 * class in order to be able to retrieve distances from one point
			 * to another quickly. I can imagine two ways to do this:
			 * 		1) Store the graph as
			 * 			Map<GeographicPoint, HashMap<GeographicPoint, Edge>> to have
			 * 			constant time access to those distances.
			 * 		2) While running a search, store all the shortest paths along
			 * 			the way in a
			 * 			HashMap<Pair<GeographicPoint, GeographicPoint>, Double>.
			 * 			This is probably the fastest way, but also the most
			 * 			memory consuming way.
			 */
			saveDistance(start, end, distance);
		}
		return distance;
	}

	/**
	 * This method returns the next location that the traveling salesperson should
	 * visit
	 * @param currentLocation The current location of the salesperson
	 * @param toVisit The set of (remaining) locations to visit
	 * @return The location that should be visited next
	 */
	private GeographicPoint greedyGetNextLocation(GeographicPoint currentLocation,
			Set<GeographicPoint> toVisit)
	{
		GeographicPoint currentBestCandidate = null;
		double currentBestDistance = Double.POSITIVE_INFINITY;
		for(GeographicPoint potentialNextLocation : toVisit)
		{
			// Keep track of the closest node that is still left to visit
			double distanceToPotentialNextLocation =
					getDistance(currentLocation, potentialNextLocation);
			if(distanceToPotentialNextLocation < currentBestDistance)
			{
				currentBestCandidate = potentialNextLocation;
				currentBestDistance = distanceToPotentialNextLocation;
			}
		}
		return currentBestCandidate;
	}

	/**
	 * A method that compares the difference in distances between two routes
	 * constructed by the 2-Opt algorithm. Note that we only have to check the
	 * part of the route that has changed.
	 * @param route1 The first route to consider
	 * @param route2 The second route to consider
	 * @param firstNode 
	 * @param secondNode
	 * @return The difference in route lengths
	 */
	private double getRouteDistanceDifference(List<GeographicPoint> route1,
			List<GeographicPoint> route2, int firstNode, int secondNode)
	{
		return getRouteSegmentLength(route1, firstNode, secondNode + 1) -
				getRouteSegmentLength(route2, firstNode, secondNode + 1);
	}

	/**
	 * Returns the length of the segment that starts in start and ends in end
	 * @param route The route
	 * @param start The first node of the segment
	 * @param end The last node of the segment
	 * @return The distance from start to end along that route segment
	 */
	private double getRouteSegmentLength(List<GeographicPoint> route, int start,
			int end)
	{
		double distance = 0.0;
		for(int i = start; i < end; i++)
		{
			distance += getDistance(route.get(i), route.get(i+1));
		}
		return distance;
	}
	
	/**
	 * Returns the total length of a route
	 * @param route The route whose length is to be returned
	 * @return The length of the specified route
	 */
	public double getTotalRouteLength(List<GeographicPoint> route)
	{
		return getRouteSegmentLength(route, 0, route.size()-1);
	}

	/**
	 * Wrapper method to find an approximate solution for the TSP.
	 * It uses a greedy algorithm to find an initial solution and
	 * improves it with 2-Opt.
	 * @param iterations The number of complete cycles in 2-Opt. If < 0,
	 * 2-Opt will not be used
	 * @return The shortest path found
	 */
	public List<GeographicPoint> getApproximateSolution(int numIterations)
	{
		TravelingSalesperson tsp =
				new TravelingSalesperson(getStartNode(), getOtherNodes());
		try
		{
			tsp.greedySolve();
		}
		catch(TspNoPathAvailableException e)
		{
			System.out.println(e.getMessage());
		}
		try
		{
			tsp.twoOpt(numIterations);
		}
		catch(IllegalArgumentException e)
		{
			System.out.println(e.getMessage());
		}
		return tsp.path;
	}

	/**
	 * A class that represents the traveling salesperson. It keeps track of the current
	 * location of the salesperson and updates his properties.
	 * @author Dominik Sudwischer
	 *
	 */
	private class TravelingSalesperson
	{
		private List<GeographicPoint> path;
		private GeographicPoint currentLocation;
		private final Set<GeographicPoint> toVisit;

		private TravelingSalesperson(GeographicPoint startLocation,
				Set<GeographicPoint> toVisit)
		{
			path = new ArrayList<>(toVisit.size() + 2);
			path.add(startLocation);
			currentLocation = startLocation;
			this.toVisit = toVisit;
		}

		private void extendPath(GeographicPoint nextStop)
		{
			path.add(nextStop);
		}

		private void setLocation(GeographicPoint newLocation)
		{
			currentLocation = newLocation;
		}

		/**
		 * Whenever a the traveling salesman visits a location, it can be removed
		 * from the set of locations to visit.
		 * @param visitedLocation The location that has been visited recently
		 */
		private void visitLocation(GeographicPoint visitedLocation)
		{
			toVisit.remove(visitedLocation);
		}

		/**
		 * A method that is called whenever the salesperson travels to another
		 * location. It keeps track of his location, the path he walked and
		 * tracks that another location has been visited.
		 * @param nextLocation The location that will be visited
		 */
		private void travelToNextLocation(GeographicPoint nextLocation)
		{
			visitLocation(nextLocation);
			setLocation(nextLocation);
			// Add the next location to the next path
			extendPath(nextLocation);
		}

		/**
		 * This method lets the salesperson return home.
		 * @throws TspNoPathAvailableException When no path home is available, 
		 * this exception is thrown
		 */
		private void getBackHome() throws TspNoPathAvailableException
		{
			double distanceToHome =
					getDistance(currentLocation, startNode);
			if(distanceToHome == Double.POSITIVE_INFINITY)
			{
				throw new TspNoPathAvailableException();
			}
			extendPath(startNode);
		}

		/**
		 * This method chooses the next location to visit and lets the salesperson
		 * go there.
		 * @throws TspNoPathAvailableException if no path is available to any of the
		 * remaining locations.
		 */
		private void visitNextLocation() throws TspNoPathAvailableException
		{
			GeographicPoint nextLocation =
					greedyGetNextLocation(currentLocation, toVisit);
			// If this is still null, there exists no path from the current
			// location to any of the still to visit nodes
			if(nextLocation == null)
			{
				throw new TspNoPathAvailableException();
			}
			travelToNextLocation(nextLocation);
		}

		/**
		 * A method to solve the TSP using a greedy approach.
		 * @return The path that is found by the greedy algorithm. This is not
		 * necessarily the optimal path.
		 * @throws TspNoPathAvailableException if the salesperson gets stuck
		 * on his route
		 */
		public List<GeographicPoint> greedySolve() throws TspNoPathAvailableException
		{
			// While there are nodes to visit, repeat the greedy procedure
			while(toVisit.size() > 0)
			{
				visitNextLocation();
			}
			// Now we have to return to the start
			getBackHome();
			return path;
		}

		/**
		 * This method implements the 2-Opt optimization algorithm for approximate
		 * solutions to the TSP. It removes two edges and repairs the route from that.
		 * One could either try all possible combinations (on the order of O(#nodes)
		 * operations) or choose nodes at random. We will use the systematic approach
		 * here.
		 * @param numIterations The number of iterations
		 * @return The sequence of nodes the traveling salesman should visit (in that
		 * particular order)
		 */
		public void twoOpt(int numIterations) throws IllegalArgumentException
		{
			if(numIterations < 0)
			{
				throw new IllegalArgumentException();
			}
			if(path.size() <= 3)
			{
				// In this case, there is at most 1 other node (in addition to the
				// starting location, so the greedy TSP algorithm will find the
				// optimal solution, i.e. here is nothing to be done.
				return;
			}
			for(int iteration = 0; iteration < numIterations; iteration++)
			{
				for(int firstNode = 0; firstNode < path.size() - 3; firstNode++)
				{
					for(int secondNode = firstNode + 2; secondNode < path.size() - 1;
							secondNode++)
					{
						// Now that the two nodes are chosen, we will do the following:
						// We will take the path up to and including node firstNode and
						// append it to our new path.
						// Then, we will go to node secondNode and append it to the
						// path. From there on, we will add nodes to the route backwards
						// until we stop next to node firstNode. From there on, we will
						// append the remaining nodes to the route in the normal order.
						List<GeographicPoint> newRoute =
								twoOptSwap(firstNode, secondNode);
						double benefit = getRouteDistanceDifference(path, newRoute,
								firstNode, secondNode);
						if(benefit > 0.0)
						{
							path = newRoute;
						}
					}
				}
			}
			return;
		}

		/**
		 * A helper method to do a single 2-Opt swap step
		 * @param firstNode The first node of the swap
		 * @param secondNode The second node of the swap
		 * @param tsp The TravelingSalesperson including the path found by the
		 * greedy algorithm
		 * @return A LinkedList that contains the (swapped) path
		 */
		private List<GeographicPoint> twoOptSwap(int firstNode, int secondNode)
		{
			List<GeographicPoint> newRoute = new LinkedList<GeographicPoint>();
			for(int i = 0; i <= firstNode; i++)
			{
				// We add the first firstNode+1 nodes to the path
				newRoute.add(path.get(i));
			}
			for(int i = secondNode; i > firstNode; i--)
			{
				// Then, we add everything from secondNode down to and including
				// firstNode+1 to the route
				newRoute.add(path.get(i));
			}
			for(int i = secondNode + 1; i < path.size(); i++)
			{
				newRoute.add(path.get(i));
			}
			return newRoute;
		}
	}
}
