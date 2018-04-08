package graph;

/**
 * An interface to provide a basic functionality for edges
 * @author Dominik Sudwischer
 *
 */
public interface Edge
{	
	/**
	 * Returns the end point of the edge
	 * @return The end point of the edge
	 */
	public int getEndPoint();
	
	/**
	 * Sets the end point of the edge
	 * @param endPoint The end point of the edge
	 */
	public void setEndPoint(int endPoint);
}
