package graph;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import graph.grader.EgoGrader;

public class CapGraph implements Graph
{

	// Member variables
	private Map<Integer, HashMap<Integer, Integer>> nodeToEdgeMap;
	
	// Constructor
	public CapGraph()
	{
		nodeToEdgeMap = new HashMap<Integer, HashMap<Integer, Integer>>();
	}
	
	/** (non-Javadoc)
	 * @see graph.Graph#addVertex(int)
	 */
	@Override
	public void addVertex(int num)
	{
		if(nodeToEdgeMap.containsKey(num))
		{
			return;
		}
		HashMap<Integer, Integer> edgeEndPoints = new HashMap<Integer, Integer>();
		nodeToEdgeMap.put(num, edgeEndPoints);
	}

	/** (non-Javadoc)
	 * @see graph.Graph#addEdge(int, int)
	 */
	@Override
	public void addEdge(int from, int to)
	{
		/* We have to allow for multi-edges, so there is no check whether such an
		 * edge is already contained
		 */
		if(!nodeToEdgeMap.containsKey(from))
		{
			addVertex(from);
		}
		HashMap<Integer, Integer> neighborMultiplicityMap = nodeToEdgeMap.get(from);
		Integer multiplicity = neighborMultiplicityMap.get(to);
		if(multiplicity == null)
		{
			neighborMultiplicityMap.put(to, 1);
		}
		else
		{
			neighborMultiplicityMap.put(to, multiplicity + 1);
		}
	}
	
	public void addMultiEdge(int from, int to, int multiplicity)
	{
		if(!nodeToEdgeMap.containsKey(from))
		{
			addVertex(from);
		}
		nodeToEdgeMap.get(from).put(to, multiplicity);
	}

	/** (non-Javadoc)
	 * @see graph.Graph#getEgonet(int)
	 */
	@Override
	public Graph getEgonet(int center)
	{
		/** We first find all nodes in the Egonet (these are the one-hop-neighbors).
		 *  The we add all edges between those.
		 */
		Graph egoNet = new CapGraph();
		Set<Integer> egoNetNodes = getOneHopDestinations(center);
		egoNetNodes.add(center);
		for(int from : egoNetNodes)
		{
			HashMap<Integer, Integer> destinationMultiplicityMap =
					nodeToEdgeMap.get(from);
			for(int to : destinationMultiplicityMap.keySet())
			{
				if(egoNetNodes.contains(to))
				{
					egoNet.addMultiEdge(from, to, destinationMultiplicityMap.get(to));
				}
			}
		}
		return egoNet;
	}

	/** (non-Javadoc)
	 * @see graph.Graph#getSCCs()
	 */
	@Override
	public List<Graph> getSCCs()
	{
		// TODO Auto-generated method stub
		return null;
	}

	/** (non-Javadoc)
	 * @see graph.Graph#exportGraph()
	 */
	@Override
	public HashMap<Integer, HashSet<Integer>> exportGraph()
	{
		HashMap<Integer, HashSet<Integer>> graph =
				new HashMap<Integer, HashSet<Integer>>(nodeToEdgeMap.size());
		for(int from : nodeToEdgeMap.keySet())
		{
			graph.put(from, getOneHopDestinations(from));
		}
		return graph;
	}
	
	/**
	 * A method to return a set of IDs of one-hop destinations
	 * @param from The starting point
	 * @return The set of points that can be reached from "from" via one edge
	 */
	private HashSet<Integer> getOneHopDestinations(int from)
			throws IndexOutOfBoundsException
	{
		if(!nodeToEdgeMap.containsKey(from))
		{
			throw new IndexOutOfBoundsException();
		}
		Set<Integer> keys = nodeToEdgeMap.get(from).keySet();
		HashSet<Integer> destinations = new HashSet<Integer>(keys.size());
		for(int to : keys)
		{
			destinations.add(to);
		}
		return destinations;
	}
	
	/**
	 * A method to return the number of nodes in the graph
	 * @return The number of nodes in the graph
	 */
	public int getNumNodes()
	{
		return this.nodeToEdgeMap.size();
	}
	
	@Override
	public String toString()
	{
		StringBuilder sb = new StringBuilder();
		for(int nodeId : nodeToEdgeMap.keySet())
		{
			sb.append("Node# ").append(nodeId).append(":\n");
			for(int endPoint : nodeToEdgeMap.get(nodeId).keySet())
			{
				sb.append("\t->\t").append(endPoint).append("\n");
			}
		}
		return sb.toString();
	}
	
	public static void main(String[] args)
	{
		CapGraph graph = new CapGraph();
		graph.addEdge(25, 18);
		graph.addEdge(25,  23);
		graph.addEdge(25,  65);
		graph.addEdge(23,  18);
		graph.addEdge(23,  65);
		graph.addEdge(23,  25);
		graph.addEdge(65,  25);
		graph.addEdge(65,  23);
		graph.addEdge(18,  23);
		graph.addEdge(18,  25);
		//System.out.println(graph);
		System.out.println(graph.getEgonet(25));
	}
}