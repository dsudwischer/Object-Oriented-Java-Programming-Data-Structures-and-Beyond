package roadgraph;

import geography.GeographicPoint;

public class SimpleNode
{
	// Member variables
	private boolean visited;
	private GeographicPoint parent;
	private GeographicPoint location;
	
	// Constructor
	public SimpleNode(GeographicPoint location)
	{
		visited = false;
		parent = null;
		this.location = location;
	}
	
	// Methods
	public boolean isVisited()
	{
		return visited;
	}
	
	public void visit()
	{
		visited = true;
	}
	
	public GeographicPoint getParent()
	{
		return parent;
	}
	
	public void setParent(GeographicPoint newParent)
	{
		parent = newParent;
	}
	
	public GeographicPoint getLocation()
	{
		return location;
	}
}
