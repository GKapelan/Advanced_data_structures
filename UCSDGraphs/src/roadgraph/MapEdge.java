package roadgraph;

public class MapEdge {
	
	private MapNode start;
	private MapNode goal;
	
	public MapEdge(MapNode s, MapNode g) {
		// TODO Auto-generated constructor stub
		
		start = s;
		goal  = g;
	}
	
	public MapNode getStart() {
		return start;
	}

	public MapNode getGoal() {
		return goal;
	}
	
	public MapNode getOtherNode(MapNode node)
	{
		MapNode result = null;
		if (node.equals(start)) 
			result = goal;
		else if (node.equals(goal))
			result = start;
		return result;
	}
}
