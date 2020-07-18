package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	
	/** The two end points of the edge */
	private MapNode start;
	private MapNode goal;
	/** The name of the road */
	private String roadName;
	/** The type of the road */
	private String roadType;
	/** The length of the road segment, in km */
	private double length;
	static final double DEFAULT_LENGTH = 0.01;
	
	/** Create a new MapEdge object
	 * 
	 * @param roadName
	 * @param n1  The point at one end of the segment
	 * @param n2  The point at the other end of the segment
	 * 
	 */
	MapEdge(String roadName, MapNode n1, MapNode n2) 
	{
		this(roadName, "", n1, n2, DEFAULT_LENGTH);
	}
	
	/** 
	 * Create a new MapEdge object
	 * @param roadName  The name of the road
	 * @param roadType  The type of the road
	 * @param n1 The point at one end of the segment
	 * @param n2 The point at the other end of the segment
	 */
	MapEdge(String rName, String rType, MapNode n1, MapNode n2) 
	{
		this(rName, rType, n1, n2, DEFAULT_LENGTH);
	}
	
	/** 
	 * Create a new MapEdge object
	 * @param roadName  The name of the road
	 * @param roadType  The type of the road
	 * @param n1 The point at one end of the segment
	 * @param n2 The point at the other end of the segment
	 * @param length The length of the road segment
	 */	
	MapEdge(String rName, String rType,	MapNode n1, MapNode n2, double l) 
	{
		roadName = rName;
		start = n1;
		goal  = n2;
		roadType = rType;
		length   = l;
	}
	
	/**
	 * Return the location of the start point
	 * @return The location of the start point as a GeographicPoint
	 */
	public MapNode getStart() {
		return start;
	}

	/**
	 * return the MapNode for the goal point
	 * @return the MapNode for the goal point
	 */
	public MapNode getGoal() {
		return goal;
	}
	
	/**
	 * Return the location of the start point
	 * @return The location of the start point as a GeographicPoint
	 */
	GeographicPoint getStartPoint()
	{
		return start.getLocation();
	}
	
	/**
	 * Return the location of the goal point
	 * @return The location of the goal point as a GeographicPoint
	 */
	GeographicPoint getEndPoint()
	{
		return goal.getLocation();
	}
	
	/**
	 * Get the road's name
	 * @return the name of the road that this edge is on
	 */
	public String getRoadName() {
		return roadName;
	}

	/**
	 * return the MapNode for the end point
	 * @return the MapNode for the end point
	 */
	public String getRoadType() {
		return roadType;
	}
	
	/**
	 * Return the length of this road segment
	 * @return the length of the road segment
	 */
	public double getLength() {
		return length;
	}	
		
	/**
	 * Given one of the nodes involved in this edge, get the other one
	 * @param node The node on one side of this edge
	 * @return the other node involved in this edge
	 */
	MapNode getOtherNode(MapNode node)
	{
		if (node.equals(start)) 
			return goal;
		else if (node.equals(goal))
			return start;
		throw new IllegalArgumentException("Looking for " +
			"a point that is not in the edge");
	}
	
	/**
	 * Return a String representation for this edge.
	 */
	@Override
	public String toString()
	{
		String toReturn = "[EDGE between ";
		toReturn += "\n\t" + start.getLocation();
		toReturn += "\n\t" + goal.getLocation();
		toReturn += "\nRoad name: " + roadName + " Road type: " + roadType +
				" Segment length: " + String.format("%.3g", length) + "km";
		
		return toReturn;
	}
}