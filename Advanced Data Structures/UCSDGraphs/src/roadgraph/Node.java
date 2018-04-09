package roadgraph;

import geography.GeographicPoint;

/** A class to store details about nodes in a pathfinding procedure. This class
 * stores the distance from the start node, the parent node (i.e. the node that
 * precedes this node on the shortest path from the start to this node) and whether
 * this node has already been visited.
 * @author Dominik Sudwischer
 *
 */
class Node extends SimpleNode implements Comparable<Node> 
{
	// Member variables
	private double distance;
	private double priority;
	
	// Constructor
	public Node(GeographicPoint location)
	{
		super(location);
		distance = Double.POSITIVE_INFINITY;
		priority = -1;
	}
	
	// Methods
	public double getDistance()
	{
		return distance;
	}
	
	public void setDistance(double newDistance)
	{
		distance = newDistance;
	}
	
	public double getPriority()
	{
		return priority;
	}
	
	public void setPriority(double priority)
	{
		this.priority = priority;
	}
	
	// This method enables the use of a priority queue with the Node class.
	// In A* and Dijkstra's algorithm, this is a necessary property.
	@Override
	public int compareTo(Node other)
	{
		return ((Double) priority).compareTo(other.getPriority());
	}
}
