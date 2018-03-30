package roadgraph;

/**
 * This simple class stores two values of any kind. 
 * @author Dominik Sudwischer
 *
 * @param <T1> The first value to store
 * @param <T2> The second value to store
 */
public class Pair<T1, T2>
{
	private final T1 t1;
	private final T2 t2;
	
	public Pair(T1 t1, T2 t2)
	{
		this.t1 = t1;
		this.t2 = t2;
	}
	
	public T1 getFirst()
	{
		return t1;
	}
	
	public T2 getSecond()
	{
		return t2;
	}
}
