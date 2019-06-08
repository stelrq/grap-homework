/**
 * 
 */
package roadgraph;

import geography.GeographicPoint;

/**
 * @author UCSD Intermediate Programming MOOC team, modified by Monika
 *
 * A directed edge in a map graph from Node start to Node end
 */
class MapEdge 
{
	/** The name of the road */
	private String roadName;
	
	/** The type of the road */
	private String roadType;
	
	/** The end point of the edge */
	private GeographicPoint end;
	
	
	/** The length of the road segment, in km */
	private double length;
	
	static final double DEFAULT_LENGTH = 0.01;
	
	
	/** Create a new MapEdge object
	 * 
	 * @param roadName
	 * @param n2  The point at the other end of the segment
	 * 
	 */
	MapEdge(String roadName, GeographicPoint n2) 
	{
		this(roadName, "", n2, DEFAULT_LENGTH);
	}
	
	/** 
	 * Create a new MapEdge object
	 * @param roadName  The name of the road
	 * @param roadType  The type of the road
	 * @param n2 The point at the other end of the segment
	 */
	MapEdge(String roadName, String roadType, GeographicPoint n2) 
	{
		this(roadName, roadType, n2, DEFAULT_LENGTH);
	}
	
	/** 
	 * Create a new MapEdge object
	 * @param roadName  The name of the road
	 * @param roadType  The type of the road
	 * @param n2 The point at the other end of the segment
	 * @param length The length of the road segment
	 */	
	MapEdge(String roadName, String roadType,
			GeographicPoint n2, double length) 
	{
		this.roadName = roadName;
		end = n2;
		this.roadType = roadType;
		this.length = length;
	}
	
	/**
	 * Return the location of the end point
	 * @return The location of the end point as a GeographicPoint
	 */
	GeographicPoint getEndPoint()
	{
		return end;
	}
	
	/**
	 * Return the length of this road segment
	 * @return the length of the road segment
	 */
	double getLength()
	{
		return length;
	}
	
	/**
	 * Get the road's name
	 * @return the name of the road that this edge is on
	 */
	public String getRoadName()
	{
		return roadName;
	}

	/**
	 * Return a String representation for this edge.
	 */
	@Override
	public String toString()
	{
		String toReturn = "[EDGE to ";
		toReturn += "\n\t" + end;
		toReturn += "\nRoad name: " + roadName + " Road type: " + roadType +
				" Segment length: " + String.format("%.3g", length) + "km";
		
		return toReturn;
	}

}
