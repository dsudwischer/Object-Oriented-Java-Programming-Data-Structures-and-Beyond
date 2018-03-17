package roadgraph;

import geography.GeographicPoint;

/*
 * A basic class to represent edges in a graph.
 * Each edge has specific start and end locations, a name, a road type and a weight.
 * The weight member variable can be used to store information about how long it takes
 * to pass this edge.
 */

public class Edge
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
}
