/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
		
	HashMap<GeographicPoint, MapNode> gPointMapNode;
	HashSet<MapEdge> edges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		gPointMapNode = new HashMap<GeographicPoint, MapNode>();
		edges = new HashSet<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return gPointMapNode.values().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return gPointMapNode.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return edges.size();
	}
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		if (location == null) {
			return false;
		}
		
		MapNode n = gPointMapNode.get(location);
		if (n == null) {
			n = new MapNode(location);
			gPointMapNode.put(location, n);
			return true;
		}
		else {
			System.out.println("Warning: Node at location " + location + " already exists in the graph.");
			return false;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		
		MapNode n1 = gPointMapNode.get(from);
		MapNode n2 = gPointMapNode.get(to);
		
		MapEdge edge = new MapEdge(roadName, roadType,n1, n2,length);
		edges.add(edge);
		n1.addEdge(edge);
	}

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		chkGeographicPoint(start, goal);
		
		MapNode startNode = gPointMapNode.get(start);
		MapNode goalNode  = gPointMapNode.get(goal);
		
		chkNode(startNode,goalNode);
		
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		boolean found = bfsSearch(startNode, goalNode, parentMap, nodeSearched);
		
		if (!found) {
			System.out.printf("No path from %s to %s", start,goal);
			return null;
		}
		
		return constructPath(startNode, goalNode, parentMap);
	}
	
	/**
	 * Find the path from start to goal using breadth first search
	 * @param start The starting location
	 * @param goal The goal location
	 * @param parentMap the HashNode map of children and their parents
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return True if we found a goal location
	 */
	private boolean bfsSearch(MapNode start, MapNode goal, HashMap<MapNode, MapNode> parentMap, Consumer<GeographicPoint> nodeSearched) {
		
		HashSet<MapNode> visited = new HashSet<MapNode>();
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		toExplore.add(start);
		boolean found = false;
		
		while (!toExplore.isEmpty()) {
			MapNode curr = toExplore.remove();
			
			if (curr == goal) {
				found = true;
				break;
			}
			
			Set<MapNode> neighbors = curr.getNeighbors();
			for (MapNode next : neighbors) {
				if (!visited.contains(next)) {
					visited.add(next);
					parentMap.put(next, curr);
					toExplore.add(next);
				}
			}
		}
		return found;
	}	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
				
		chkGeographicPoint(start, goal);
		
		MapNode startNode = gPointMapNode.get(start);
		MapNode goalNode  = gPointMapNode.get(goal);
		
		chkNode(startNode,goalNode);
		
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		
		// initialize distance for all nodes
		for (MapNode n : gPointMapNode.values()) {
			n.setDistance(Double.POSITIVE_INFINITY);
		}
		
		boolean found = dijkstraSearch(startNode, goalNode, toExplore, visited, parentMap, nodeSearched);
		
		if (!found) {
			System.out.println("No path found from " + start + " to " + goal);
			return null;
		}
		
		return constructPath(startNode, goalNode, parentMap);
	}
	
	/**
	 * Dijkstra search algorithm
	 * @param startNode The start node
	 * @param endNode The goal node
	 * @param toExplore The PriorityQueue ordering distance from start to goal
	 * @param visited The HashSet visited map 
	 * @param parentMap The HashNode map of children and their parents
	 * @param nodeSearched nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return True if we found a goal node
	 * 
	 * Enqueue {S,0} onto the PQ
	 * while PQ is not empty:
	 *   dequeue node curr from front of queue
	 *   if (curr is not visited)
	 *     add curr to visited set
	 *     if (curr == G) return parent map
	 *     for each of curr's neighbors, n, not in visited set:
	 *       if path through curr to n is shorter
	 *         update curr as n's parent in parent map
	 *         enqueue {n,distance} into the PQ
	 * if we get here then there's no path from S to G
	 */
	private boolean dijkstraSearch(MapNode startNode, MapNode endNode, PriorityQueue<MapNode> toExplore, 
			HashSet<MapNode> visited, HashMap<MapNode, MapNode> parentMap, Consumer<GeographicPoint> nodeSearched) {
		
		startNode.setDistance(0);
		toExplore.add(startNode);
		
		int count = 0;
		boolean found = false;
				
		while (!toExplore.isEmpty()) {
			MapNode curr = toExplore.remove();
			count++;
			
			System.out.printf("\nDIJKSTRA visiting %s", curr);
			if (curr.equals(endNode)) {
				found = true;
				System.out.printf("\nNodes visited in Dijkstra search: %s", count);
				break;
			}
			
			if(!visited.contains(curr)) {
				visited.add(curr);
				Set<MapEdge> edges = curr.getEdges();
				for (MapEdge edge : edges) {
					MapNode neighbor = edge.getGoal();
					if (!visited.contains(neighbor)) {
						double currDist = edge.getLength() + curr.getDistance();
						if (currDist < neighbor.getDistance()) {
							parentMap.put(neighbor, curr);
							neighbor.setDistance(currDist);
							toExplore.add(neighbor);
						}
					}
				}
			}
		}
		
		return found;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		chkGeographicPoint(start, goal);
		
		MapNode startNode = gPointMapNode.get(start);
		MapNode goalNode  = gPointMapNode.get(goal);
		
		chkNode(startNode,goalNode);
		
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		
		// initialize distance for all nodes
		for (MapNode n : gPointMapNode.values()) {
			n.setDistance(Double.POSITIVE_INFINITY);
			n.setActualDistance(Double.POSITIVE_INFINITY);
		}
		
		boolean found = aStarSearch(startNode, goalNode, toExplore, visited, parentMap, nodeSearched);
		
		if (!found) {
			System.out.println("No path found from " + start + " to " + goal);
			return null;
		}
		
		return constructPath(startNode, goalNode, parentMap);
	}
	
	/**
	 * A-Star search algorithm
	 * @param startNode The start node
	 * @param goalNode The goal node
	 * @param toExplore The PriorityQueue ordering distance from start to goal
	 * @param visited The HashSet visited map 
	 * @param parentMap The HashNode map of children and their parents
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return True if we found a goal node
	 */
	private boolean aStarSearch(MapNode startNode,MapNode goalNode,PriorityQueue<MapNode> toExplore,
			HashSet<MapNode> visited,HashMap<MapNode,MapNode> parentMap,Consumer<GeographicPoint> nodeSearched) {
		
		startNode.setDistance(0);
		startNode.setActualDistance(0);

		toExplore.add(startNode);
		
		int count = 0;
		boolean found = false;

		while (!toExplore.isEmpty()) {
			MapNode next = toExplore.remove();
            count++;

    		//System.out.print("\nA* visiting" + next + "\nActual = " + next.getActualDistance() + ", Prev: " + next.getDistance());
            System.out.printf("\nA* visiting %s \nActual = %s, Prev: %s", next, next.getActualDistance(), next.getDistance());
			if (next.equals(goalNode)) {
				found = true;
				System.out.printf("\nNodes visited in aStar search: %s", count);
				break;
			}
			if(!visited.contains(next)) {
				visited.add(next);
				Set<MapEdge> edges = next.getEdges();
				for (MapEdge edge : edges) {
					MapNode neighbor = edge.getGoal();
					if (!visited.contains(neighbor)) {

						double currDist = edge.getLength()+next.getActualDistance();
						double prevDist = currDist+ (neighbor.getLocation()).distance(goalNode.getLocation());
						if(prevDist < neighbor.getDistance()){							
							parentMap.put(neighbor, next);
							neighbor.setActualDistance(currDist);
							neighbor.setDistance(prevDist);
							toExplore.add(neighbor);
						}
					}
				}
			}
		}	
		return found;
	}

	/**
	 * This method for checking start and goal GeographicPoint's for null value
	 * @param s start GeographicPoint
	 * @param g goal GeographicPoint
	 */
	private void chkGeographicPoint(GeographicPoint s, GeographicPoint g) {
		if (s == null || g == null)
			throw new NullPointerException("Cannot finde route");
	}
	
	/**
	 * This method for checking start and goal MapNode's for null value
	 * @param s start MapNode
	 * @param g goal MapNode
	 */
	private void chkNode(MapNode s, MapNode g) {
		if (s == null || g == null) {
			throw new NullPointerException("No path exists");
		}		
	}
	
	/**
	 * Reconstruct a path from start to goal using the parentMap
	 * @param start The starting location
	 * @param goal The goal location
	 * @param parentMap the HashNode map of children and their parents
	 * @return he list of intersections that form the shortest path from start to goal (including both start and goal).
	 */
	private List<GeographicPoint> constructPath(MapNode start, MapNode goal, HashMap<MapNode, MapNode> parentMap){
		
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode curr = goal;
		
		while (curr != start) {
			path.addFirst(curr.getLocation());
			curr = parentMap.get(curr);
		}
		
		path.addFirst(start.getLocation());
		return path;
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.print("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		System.out.printf("\nTest 1 Dijkstra: %s", testroute);
		System.out.printf("\nTest 1 aStarSearch: %s", testroute2);		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.print("\n\nTest 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		System.out.printf("\nTest 2 Dijkstra: %s", testroute);
		System.out.printf("\nTest 2 aStarSearch: %s", testroute2);			
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.print("\n\nTest 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		System.out.printf("\nTest 3 Dijkstra: %s", testroute);
		System.out.printf("\nTest 3 aStarSearch: %s", testroute2);			
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \n\nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		
		System.out.printf("\nTest UTC.MAP Dijkstra: %s", route);
		System.out.printf("\nTest UTC.MAP aStarSearch: %s", route2);	

		/*
		 * Week 3 MyTest 
		 */
		System.out.println("\n\nTesting My MapGraph");
		System.out.print("Making a new map...");
		MapGraph myMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", myMap);
		System.out.println("DONE.");
		List<GeographicPoint> routeBFS = myMap.bfs(new GeographicPoint(1.0, 1.0), new GeographicPoint(8.0, -1.0));
		System.out.println("Route BFS: " + routeBFS);
		System.out.println("End of my MapGraph testing");		
	}
	
}
