package roadgraph;

import geography.GeographicPoint;

/*
 * A basic class to represent edges in a graph.
 * Each edge has specific start and end locations, a name, a road type and a weight.
 * The weight member variable can be used to store information about how long it takes
 * to pass this edge.
 */

public class Edge implements Comparable<Edge>
{
	// Member variables
	private GeographicPoint start;
	private GeographicPoint end;
	private String name;
	private String roadType;
	private double weight;
	
	// Methods
	// Getters
	public GeographicPoint getStart()
	{
		return start;
	}
	
	public GeographicPoint getEnd()
	{
		return end;
	}
	
	public double getWeight()
	{
		return weight;
	}
	
	public String getName()
	{
		return name;
	}
	
	public String getType()
	{
		return roadType;
	}
	
	// Constructor
	public Edge(GeographicPoint start, GeographicPoint end, String name, String roadType,
			double weight)
	{
		this.start = start;
		this.end = end;
		this.name = name;
		this.roadType = roadType;
		this.weight = weight;
	}
	
	/** A method to compare two edges. An edge is shorter than another edge if and only if
	 * its weight is less than the weight of the other edge.
	 * @param otherEdge The edge to compare this edge to
	 * @return The comparison result, i.e. 1 if this.weigth > otherEdge.weight,
	 * 0 if equal, -1 if the other edge has more weight.
	 */
	@Override
	public int compareTo(Edge otherEdge)
	{
		return ((Double) weight).compareTo(otherEdge.weight);
	}
}
