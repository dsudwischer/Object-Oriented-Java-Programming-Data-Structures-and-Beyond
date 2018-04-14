package roadgraph;

import java.util.List;

/**
 * A simple class that wraps paths and path lengths in a single object. The getter returns
 * a reference to the original list.
 * 
 * @author Dominik Sudwischer
 */

public class PathInfoWrapper
{
	private final List<Integer> path;
	private final double distance;
	
	public PathInfoWrapper(List<Integer> path, double distance)
	{
		this.path = path;
		this.distance = distance;
	}
	
	public List<Integer> getPath()
	{
		return path;
	}
	
	public double getDistance()
	{
		return distance;
	}
}
