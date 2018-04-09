package roadgraph;

/**
 * This simple class stores two values of any kind. 
 * @author Dominik Sudwischer
 *
 * @param <T1> The first value to store
 * @param <T2> The second value to store
 */
public class Pair<T1, T2> {
	private final T1 t1;
	private final T2 t2;
	
	public Pair(T1 t1, T2 t2) {
		this.t1 = t1;
		this.t2 = t2;
	}
	
	public T1 getFirst() {
		return t1;
	}
	
	public T2 getSecond() {
		return t2;
	}
	
	/*
	 * We are overriding the default hashCode() function here in order to be conform
	 * with the fact that two pairs are equal if and only if their two components.
	 * This change is required to support the equals(Object) method.
	 * are equal.
	 * (non-Javadoc)
	 * @see java.lang.Object#hashCode()
	 */
	public int hashCode() {
		int t2HashCode = t2.hashCode();
		return t1.hashCode() + t2HashCode * t2HashCode;
	}
	
	public boolean equals(Object other) {
		// Note that this requires T1 and T2 have suitable "equals" methods!
		return this.getClass() == other.getClass() &&
				this.t1 == ((Pair<T1, T2>) other).t1 &&
				this.t2 == ((Pair<T1, T2>) other).t2;
	}
}
