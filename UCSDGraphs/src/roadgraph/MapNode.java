package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;


public class MapNode implements Comparable {
	
	/** The list of edges out of this node */
	private HashSet<MapEdge> edges;
	/** The latitude and longitude of this node */
	private GeographicPoint location;
	
	/** Distance between nodes */
	private double distance;
	private double actualDistance;
	
	/** 
	 * Create a new MapNode at a given Geographic location
	 * @param loc the location of this node
	 */
	MapNode(GeographicPoint loc) {
		location = loc;
		edges    = new HashSet<MapEdge>();
		distance = 0.0;
	}
	
	/**
	 * return the edges out of this node
	 * @return a set containing all the edges out of this node.
	 */
	public Set<MapEdge> getEdges() {
		return edges;
	}
	
	/**
	 * Get the geographic location that this node represents
	 * @return the geographic location of this node
	 */
	public GeographicPoint getLocation() {
		return location;
	}
	
	/**
	 * Add an edge that is outgoing from this node in the graph
	 * @param edge The edge to be added
	 */
	public void addEdge(MapEdge edge)
	{
		edges.add(edge);
	}
	
	/**  
	 * Return the neighbors of this MapNode 
	 * @return a set containing all the neighbors of this node
	 */
	public Set<MapNode> getNeighbors()
	{
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for (MapEdge edge : edges) {
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}

	/**
	 * Add distance
	 * @param d
	 */
	public void setDistance(double d) {
		distance = d;
	}

	/**
	 * Return distance
	 * @return distance
	 */
	public double getDistance() {
		return distance;
	}
	
	/**
	 * Add actualDistance
	 * @param actualDistance
	 */
	public void setActualDistance(double actualDistance) {
		this.actualDistance = actualDistance;
	}
	
	/**
	 * Return actualDistance
	 * @return actualDistance
	 */
	public double getActualDistance() {
		return actualDistance;
	}
	
	/**
	 * Implement Comparable
	 */
	public int compareTo(Object o) {
		// convert to map node, may throw exception
		MapNode m = (MapNode)o; 
		return ((Double)this.getDistance()).compareTo((Double) m.getDistance());
	}
	
	/** Returns whether two nodes are equal.
	 * Nodes are considered equal if their locations are the same, 
	 * even if their street list is different.
	 * @param o the node to compare to
	 * @return true if these nodes are at the same location, false otherwise
	 */
	@Override
	public boolean equals(Object o)
	{
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode)o;
		return node.location.equals(this.location);
	}
	
	/** Because we compare nodes using their location, we also 
	 * may use their location for HashCode.
	 * @return The HashCode for this node, which is the HashCode for the 
	 * underlying point
	 */
	@Override
	public int hashCode()
	{
		return location.hashCode();
	}
	
	/** ToString to print out a MapNode object
	 *  @return the string representation of a MapNode
	 */
	@Override
	public String toString()
	{
		String toReturn = "[NODE at location (" + location + ")";
		toReturn += " intersects streets: ";
		for (MapEdge e: edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += "]";
		return toReturn;
	}

	// For debugging, output roadNames as a String.
	public String roadNamesAsString()
	{
		String toReturn = "(";
		for (MapEdge e: edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += ")";
		return toReturn;
	}
}