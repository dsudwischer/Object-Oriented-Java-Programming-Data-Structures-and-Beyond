package roadgraph;

import java.util.List;

import geography.GeographicPoint;

/**
 * A simple class that wraps paths and path lengths in a single object.
 * @author Dominik Sudwischer
 */

public class PathInfoWrapper
{
	private final List<GeographicPoint> path;
	private final double distance;
	
	public PathInfoWrapper(List<GeographicPoint> path, double distance)
	{
		this.path = path;
		this.distance = distance;
	}
	
	public List<GeographicPoint> getPath()
	{
		return path;
	}
	
	public double getDistance()
	{
		return distance;
	}
}
