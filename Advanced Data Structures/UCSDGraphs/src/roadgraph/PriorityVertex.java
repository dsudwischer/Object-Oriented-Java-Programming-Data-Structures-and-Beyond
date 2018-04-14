package roadgraph;

public class PriorityVertex implements Comparable<PriorityVertex> {
	
	// Member variables
	private final int id;
	private final double priority;
	
	public PriorityVertex(int id, double priority) {
		this.id = id;
		this.priority = priority;
	}
	
	public int getId() {
		return id;
	}
	
	public double getPriority() {
		return priority;
	}

	@Override
	public int compareTo(PriorityVertex other) {
		return ((Double) priority).compareTo((Double) other.getPriority());
	}
}
