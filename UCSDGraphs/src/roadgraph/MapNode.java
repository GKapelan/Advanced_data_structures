package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

public class MapNode{
	
	private HashSet<MapEdge> edges;
	
	private GeographicPoint location;
	
	MapNode(GeographicPoint loc) {
		location = loc;
		edges = new HashSet<MapEdge>();
	}
	
	public HashSet<MapEdge> getEdges() {
		return edges;
	}
	
	public GeographicPoint getLocation() {
		return location;
	}
	
	public void addEdge(MapEdge edge)
	{
		edges.add(edge);
	}
	
	public Set<MapNode> getNeighbors()
	{
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for (MapEdge edge : edges) {
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}
}
