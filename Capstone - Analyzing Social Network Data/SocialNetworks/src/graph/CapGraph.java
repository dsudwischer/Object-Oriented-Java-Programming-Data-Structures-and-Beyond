package graph;

import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;


public class CapGraph implements Graph
{

	// Member variables
	private Map<Integer, HashMap<Integer, Integer>> nodeToEdgeMap;

	// Constructor
	public CapGraph() {
		nodeToEdgeMap = new HashMap<Integer, HashMap<Integer, Integer>>();
	}

	/** (non-Javadoc)
	 * @see graph.Graph#addVertex(int)
	 */
	@Override
	public void addVertex(int num) {
		if(nodeToEdgeMap.containsKey(num)) {
			return;
		}
		HashMap<Integer, Integer> edgeEndPoints = new HashMap<Integer, Integer>();
		nodeToEdgeMap.put(num, edgeEndPoints);
	}

	/** (non-Javadoc)
	 * @see graph.Graph#addEdge(int, int)
	 */
	@Override
	public void addEdge(int from, int to) {
		/* We have to allow for multi-edges, so there is no check whether such an
		 * edge is already contained
		 */
		if(!nodeToEdgeMap.containsKey(from)) {
			addVertex(from);
		}
		HashMap<Integer, Integer> neighborMultiplicityMap = nodeToEdgeMap.get(from);
		Integer multiplicity = neighborMultiplicityMap.get(to);
		if(multiplicity == null) {
			neighborMultiplicityMap.put(to, 1);
		}
		else {
			neighborMultiplicityMap.put(to, multiplicity + 1);
		}
	}

	/*
	 * (non-Javadoc)
	 * @see graph.Graph#addMultiEdge(int, int, int)
	 */
	@Override
	public void addMultiEdge(int from, int to, int multiplicity) {
		if(!nodeToEdgeMap.containsKey(from)) {
			addVertex(from);
		}
		HashMap<Integer, Integer> edgeMultiplicity = nodeToEdgeMap.get(from);
		int prevMultiplicity = 0;
		if(edgeMultiplicity.containsKey(to)) {
			prevMultiplicity = edgeMultiplicity.get(to);
		}
		edgeMultiplicity.put(to, prevMultiplicity + multiplicity);
	}

	/*
	 * (non-Javadoc)
	 * @see graph.Graph#getNodes()
	 */
	@Override
	public Set<Integer> getNodes() {
		return nodeToEdgeMap.keySet();
	}

	/*
	 * (non-Javadoc)
	 * @see graph.Graph#getNeighborsMultiplicity(int)
	 */
	@Override
	public Map<Integer, Integer> getNeighborsMultiplicity(int from) {
		Map<Integer, Integer> neighborsMultiplicities = new HashMap<Integer, Integer>();
		if(nodeToEdgeMap.containsKey(from)) {
			for(Map.Entry<Integer, Integer> neighborMultiplicity :
				nodeToEdgeMap.get(from).entrySet()) {
				neighborsMultiplicities.put(neighborMultiplicity.getKey(),
						neighborMultiplicity.getValue());
			}
		}
		return neighborsMultiplicities;
	}

	/** (non-Javadoc)
	 * @see graph.Graph#getEgonet(int)
	 */
	@Override
	public Graph getEgonet(int center) {
		/** We first find all nodes in the Egonet (these are the one-hop-neighbors).
		 *  The we add all edges between those.
		 */
		Graph egoNet = new CapGraph();
		Set<Integer> egoNetNodes = getOneHopDestinations(center);
		egoNetNodes.add(center);
		for(int from : egoNetNodes) {
			HashMap<Integer, Integer> destinationMultiplicityMap =
					nodeToEdgeMap.get(from);
			for(int to : destinationMultiplicityMap.keySet()) {
				if(egoNetNodes.contains(to)) {
					egoNet.addMultiEdge(from, to, destinationMultiplicityMap.get(to));
				}
			}
		}
		return egoNet;
	}

	/**
	 * This method performs a depth-first-search exploration stack in a given graph.
	 * While doing so, it keeps track of the post orders via a stack.
	 * @param postOrderStack The Stack to track the post orders of the nodes.
	 * @param visited A Set to store nodes that have already been visited.
	 * @param currentNode The current node ID.
	 */
	private void dfsExplorationStep(Deque<Integer> postOrderStack, Set<Integer> visited,
			int currentNode) {
		visited.add(currentNode);
		if(nodeToEdgeMap.containsKey(currentNode)) {
			Set<Integer> neighbors = nodeToEdgeMap.get(currentNode).keySet();
			for(int neighborId : neighbors) {
				if(!visited.contains(neighborId)) {
					dfsExplorationStep(postOrderStack, visited, neighborId);
				}
			}
		}
		// When all neighbors are finished, we push the currentNode to the post order
		// stack.
		postOrderStack.push(currentNode);
	}

	/**
	 * A method to add nodes along the way of DFS to a graph
	 * @param graphToBuild The graph that shall be extended by nodes
	 * @param superGraph The embracing graph including edges
	 * @param currentNode The id of the current node
	 * @param visited A Set of nodes that have already been visited
	 */
	private void addNodesToScc(Graph graphToBuild, Graph superGraph, int currentNode,
			Set<Integer> visited) {
		visited.add(currentNode);
		// NOTE: This does not include any edges!
		graphToBuild.addVertex(currentNode);
		// Add all neighbor node IDs that have not been visited yet
		for(int neighborId :
			superGraph.getNeighborsMultiplicity(currentNode).keySet()) {
			if(!visited.contains(neighborId)) {
				addNodesToScc(graphToBuild, superGraph, neighborId, visited);
			}
		}
	}

	/**
	 * A method that performs depth-first search to build a stack of post orders
	 * @return A stack of node IDs according to their post order
	 */
	private Deque<Integer> getPostOrderStack() {
		Deque<Integer> nodes = new LinkedList<Integer>();
		Deque<Integer> postOrderStack = new LinkedList<Integer>();
		for(int nodeId : nodeToEdgeMap.keySet()) {
			nodes.push(nodeId);
		}
		Set<Integer> visited = new HashSet<Integer>(getNumNodes());
		while(!nodes.isEmpty()) {
			int currentNode = nodes.pop();
			if(!visited.contains(currentNode)) {
				dfsExplorationStep(postOrderStack, visited, currentNode);
			}
		}
		return postOrderStack;
	}

	/*
	 * (non-Javadoc)
	 * @see graph.Graph#getTransposed()
	 */
	public CapGraph getTransposed() {
		CapGraph transposed = new CapGraph();
		for(Map.Entry<Integer, HashMap<Integer, Integer>> nodeToEdge :
			nodeToEdgeMap.entrySet()) {
			int start = nodeToEdge.getKey();
			transposed.addVertex(start);
			for(Map.Entry<Integer, Integer> multiplicities :
				nodeToEdge.getValue().entrySet()) {
				int end = multiplicities.getKey();
				int multiplicity = multiplicities.getValue();
				transposed.addMultiEdge(end, start, multiplicity);
			}
		}
		return transposed;
	}

	/** (non-Javadoc)
	 * @see graph.Graph#getSCCs()
	 */
	@Override
	public List<Graph> getSCCs() {
		Set<Integer> visited = new HashSet<Integer>();
		Deque<Integer> postOrderStack = getPostOrderStack();
		List<Graph> sccList = new LinkedList<Graph>();
		Graph transposed = getTransposed();
		while(!postOrderStack.isEmpty()) {
			int currentNode = postOrderStack.pop();
			if(!visited.contains(currentNode)) {
				Graph scc = new CapGraph();
				addNodesToScc(scc, transposed, currentNode, visited);
				sccList.add(scc.getTransposed());
			}
		}
		return sccList;
	}

	/** (non-Javadoc)
	 * @see graph.Graph#exportGraph()
	 */
	@Override
	public HashMap<Integer, HashSet<Integer>> exportGraph() {
		HashMap<Integer, HashSet<Integer>> graph =
				new HashMap<Integer, HashSet<Integer>>(nodeToEdgeMap.size());
		for(int from : nodeToEdgeMap.keySet()) {
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
			throws IndexOutOfBoundsException {
		if(!nodeToEdgeMap.containsKey(from)) {
			throw new IndexOutOfBoundsException();
		}
		Set<Integer> keys = nodeToEdgeMap.get(from).keySet();
		HashSet<Integer> destinations = new HashSet<Integer>(keys.size());
		for(int to : keys) {
			destinations.add(to);
		}
		return destinations;
	}

	/**
	 * A method to return the number of nodes in the graph
	 * @return The number of nodes in the graph
	 */
	public int getNumNodes() {
		return this.nodeToEdgeMap.size();
	}

	/*
	 * (non-Javadoc)
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		for(int nodeId : nodeToEdgeMap.keySet()) {
			sb.append("Node# ").append(nodeId).append(":\n");
			for(int endPoint : nodeToEdgeMap.get(nodeId).keySet()) {
				sb.append("\t->\t").append(endPoint).append("\n");
			}
		}
		return sb.toString();
	}

	public static void main(String[] args) {
		CapGraph graph = new CapGraph();
		graph.addEdge(1, 2);
		graph.addEdge(2, 3);
		graph.addEdge(3,  4);
		graph.addEdge(1,  5);
		graph.addEdge(5,  6);
		graph.addEdge(6,  7);
		graph.addEdge(7,  8);
		graph.addEdge(8, 5);
		//System.out.println(graph + "\n\n\n\n");
		//System.out.println(graph.getTransposed() + "\n\n\n\n");
		System.out.println(graph.getSCCs());
	}
}