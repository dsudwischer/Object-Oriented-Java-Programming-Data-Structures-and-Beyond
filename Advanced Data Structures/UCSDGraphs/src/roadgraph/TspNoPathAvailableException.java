package roadgraph;


public class TspNoPathAvailableException extends Exception
{
	/**
	 * This exception is thrown whenever there is no valid path from the
	 * specified location to a location that still has to be visited.
	 */
	private static final long serialVersionUID = 1L;

	public TspNoPathAvailableException()
	{
		super("The algorithm failed to find a valid path.");
	}
}
