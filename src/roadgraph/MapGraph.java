/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
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
 * @author UCSD MOOC development team, modified by Monika and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	// Maintain both nodes and edges as you will need to
	// be able to look up nodes by lat/lon or by roads
	// that contain those nodes.
	private HashMap<GeographicPoint,List<MapEdge>> nodes_outedges;
		
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		nodes_outedges = new HashMap<GeographicPoint, List<MapEdge>>();
		
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		
		return nodes_outedges.keySet().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return nodes_outedges.keySet();
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
		if (location == null || nodes_outedges.keySet().contains(location)) {
			return false;
		}
		List me = new ArrayList<MapEdge>();
		nodes_outedges.put(location, me);
		// 3. TO DO
		// check if the vertex is in the graph
		// if not, create an empty list of edges and insert the pair into the graph
		// return true
		// otherwise - if already in the graph, return false
		
		return true;
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

		List<MapEdge> fromEdges = nodes_outedges.get(from);
		List<MapEdge> toEdges = nodes_outedges.get(to);
		// check nodes are valid
		if (fromEdges == null)
			throw new NullPointerException("addEdge: pt1:"+from+"is not in graph");
		
		if (toEdges == null)
			throw new NullPointerException("addEdge: pt2:"+to+"is not in graph");

		MapEdge edge = new MapEdge(roadName, roadType, to, length);
		if (!fromEdges.contains(to))
			fromEdges.add(edge);
		
	}
		
	/** 
	 * Get a set of neighbor nodes from a GeographicPoint
	 * @param node  The node to get the neighbors from
	 * @return A set containing the GeographicPoint objects that are the neighbors 
	 * 	of node
	 */
	private Set<GeographicPoint> getNeighbors(GeographicPoint node) {
			Set<GeographicPoint> myneighbors = new HashSet<GeographicPoint>();			
			List<MapEdge> meEdges = nodes_outedges.get(node);
			
			// 4. TO DO
			// construct the set and return
			for (MapEdge m: meEdges) {
				myneighbors.add(m.getEndPoint());
			}
			return myneighbors;
	}
	
	// 5. TO DO add a method that given a vertex, returns a list of its edges
	public List<MapEdge> getEdges(GeographicPoint node) {
		if (!nodes_outedges.keySet().contains(node)) {
			return null;
		}
		return nodes_outedges.get(node);
		
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
	 * @param nodeSearched A hook for visualization.  
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, 
			 					     Consumer<GeographicPoint> nodeSearched)
	{
			
		// Setup - checking validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		
		if (nodes_outedges.get(start) == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (nodes_outedges.get(goal) == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		// setup to begin BFS
		HashMap<GeographicPoint,GeographicPoint> parentMap = new HashMap<GeographicPoint,GeographicPoint>();
		Queue<GeographicPoint> toExplore = new LinkedList<GeographicPoint>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		toExplore.add(start);
		visited.add(start);
		GeographicPoint next = null;
		
		
		while (!toExplore.isEmpty()) {
			next = toExplore.remove();
			if (next.equals(goal)) {
				
				break;
				
			}
				
			List<MapEdge> mapEdges = getEdges(next);
			
			for (MapEdge m: mapEdges) {
				
				if (!visited.contains(m.getEndPoint())) {
					visited.add(m.getEndPoint());
					parentMap.put(m.getEndPoint(), next);
					toExplore.add(m.getEndPoint());
				}
					
			}
			
			 
			
			// 6. TO DO
			
		}
		if (!next.equals(goal)) {
			System.out.println("No path found from " + start + " to " + goal);
			return null;
		}
		// Reconstructing the parent path
		List<GeographicPoint> path =
				reconstructPath(parentMap, start, goal);

		return path;
	
	}
	


	/** Reconstruct a path from start to goal using the parentMap
	 *
	 * @param parentMap the HashNode map of children and their parents
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	private List<GeographicPoint>
	reconstructPath(HashMap<GeographicPoint,GeographicPoint> parentMap,
					GeographicPoint start, GeographicPoint goal)
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		GeographicPoint current = goal;

		while (!current.equals(start)) {
			path.addFirst(current);
			current = parentMap.get(current);
		}

		// adding start
		path.addFirst(start);
		return path;
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
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		
		if (nodes_outedges.get(start) == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (nodes_outedges.get(goal) == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		// setup to begin dijkstra
		HashMap<GeographicPoint,GeographicPoint> parentMap = new HashMap<GeographicPoint,GeographicPoint>();
		PriorityQueue<DijkstraNode> toExplore = new PriorityQueue<DijkstraNode>();
		// YOU WILL NEED TO DEFINE AN INNER CLASS THAT ENCAPSULATES GEOGRAPHIC POINT AND DISTANCE TO THE NODE
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		//Initialize table to store distances. Key is Point under examination, The value stores that node and the shortest known distance.
		HashMap<GeographicPoint, DijkstraNode> table = new HashMap<GeographicPoint, DijkstraNode>();
		for (GeographicPoint g : nodes_outedges.keySet()) {
			table.put(g, new DijkstraNode(g, Double.POSITIVE_INFINITY));
		}
		DijkstraNode next = new DijkstraNode(start, 0);
		toExplore.add(next);
		while (!toExplore.isEmpty()) {
			
			next = toExplore.remove();
			
			if (!visited.contains(next.g)) {
				visited.add(next.g);
				
				if (next.g.equals(goal)) {
					break;
				}
				
				List<MapEdge> mapEdges = getEdges(next.g);
				
				for (MapEdge m: mapEdges) {
					
					if (!visited.contains(m.getEndPoint())) {
						if (next.g.distance(goal) + next.distance + m.getLength() < table.get(m.getEndPoint()).distance) {
							table.get(m.getEndPoint()).distance = next.g.distance(goal) + next.distance + m.getLength();
							toExplore.add(table.get(m.getEndPoint()));
							parentMap.put(m.getEndPoint(), next.g);
						}
						
					}
						
				}
			}
			
			
		}	
		if (!next.g.equals(goal)) {
			System.out.println("We could not find a path from start to goal");
			return null;
		}
			List<GeographicPoint> path = reconstructPath(parentMap, start, goal);

		return path;
		
	}
	private class DijkstraNode implements Comparable<DijkstraNode>{
		GeographicPoint g;
		double distance;
		DijkstraNode(GeographicPoint theV, double theLength) {
			g = theV;
			distance = theLength;
		}
		public int compareTo(DijkstraNode other) {
			return Double.compare(distance, other.distance);
		}
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
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		
		if (nodes_outedges.get(start) == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (nodes_outedges.get(goal) == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		// setup to begin A*
		HashMap<GeographicPoint,GeographicPoint> parentMap = new HashMap<GeographicPoint,GeographicPoint>();
		PriorityQueue<DijkstraNode> toExplore = new PriorityQueue<DijkstraNode>();
		// YOU WILL NEED TO DEFINE AN INNER CLASS THAT ENCAPSULATES GEOGRAPHIC POINT AND DISTANCE TO THE NODE
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		//Initialize table to store distances. Key is Point under examination, The value stores that node and the shortest known distance.
		HashMap<GeographicPoint, DijkstraNode> table = new HashMap<GeographicPoint, DijkstraNode>();
		for (GeographicPoint g : nodes_outedges.keySet()) {
			table.put(g, new DijkstraNode(g, Double.POSITIVE_INFINITY));
		}
		DijkstraNode next = new DijkstraNode(start, 0);
		toExplore.add(next);
		while (!toExplore.isEmpty()) {
			
			next = toExplore.remove();
			
			if (!visited.contains(next.g)) {
				visited.add(next.g);
				
				if (next.g.equals(goal)) {
					break;
				}
				
				List<MapEdge> mapEdges = getEdges(next.g);
				
				for (MapEdge m: mapEdges) {
					
					if (!visited.contains(m.getEndPoint())) {
						if (next.distance + m.getLength() < table.get(m.getEndPoint()).distance) {
							table.get(m.getEndPoint()).distance = next.distance + m.getLength();
							toExplore.add(table.get(m.getEndPoint()));
							parentMap.put(m.getEndPoint(), next.g);
						}
						
					}
						
				}
			}
			
			
		}	
		if (!next.g.equals(goal)) {
			System.out.println("We could not find a path from start to goal");
			return null;
		}
			List<GeographicPoint> path = reconstructPath(parentMap, start, goal);

		return path;
		
		
	}


	
	// you can use this method for testing

	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		
		
		System.out.println("Number of vertices: " + theMap.getNumVertices());
		System.out.println("Vertex with edges: ");
		
	//	for (GeographicPoint key : theMap.getVertices())
	//		System.out.println(key + "=" + theMap.getEdges(key));
		
		
		System.out.println("DONE.");
		// You can use this method for testing.  
		
		
		MapGraph theMap1 = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap1);
		
		System.out.println("Number of vertices: " + theMap1.getNumVertices());
		System.out.println("Vertex with edges: ");
		
		//for (GeographicPoint key : theMap1.getVertices())
		//	System.out.println(key + "=" + theMap1.getEdges(key));
		
		System.out.println("DONE.");

		/*GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap1.dijkstra(start,end, (x) -> {});
		List<GeographicPoint> route2 = theMap1.aStarSearch(start,end, (x) -> {});
		
		System.out.println(route);
		System.out.println(route2);*/

		
		
	}
	
	
}
