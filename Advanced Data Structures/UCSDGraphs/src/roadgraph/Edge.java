package roadgraph;

public class Edge implements Comparable<Edge> {

	private double weight;
	
	public Edge(double weight) {
		this.weight = weight;
	}
	
	public double getPriority() {
		return -weight;
	}
	
	public double getWeight() {
		return weight;
	}

	@Override
	public int compareTo(Edge otherEdge) {
		return Double.compare(this.getPriority(), otherEdge.getPriority());
	}
}
