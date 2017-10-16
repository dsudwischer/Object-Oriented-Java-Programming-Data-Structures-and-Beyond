package module3;

//Java utilities libraries
import java.util.ArrayList;
//import java.util.Collections;
//import java.util.Comparator;
import java.util.List;

//Processing library
import processing.core.PApplet;

//Unfolding libraries
import de.fhpotsdam.unfolding.UnfoldingMap;
import de.fhpotsdam.unfolding.marker.Marker;
import de.fhpotsdam.unfolding.data.PointFeature;
import de.fhpotsdam.unfolding.marker.SimplePointMarker;
import de.fhpotsdam.unfolding.providers.Google;
import de.fhpotsdam.unfolding.providers.MBTilesMapProvider;
import de.fhpotsdam.unfolding.utils.MapUtils;

//Parsing library
import parsing.ParseFeed;

/** EarthquakeCityMap
 * An application with an interactive map displaying earthquake data.
 * Author: UC San Diego Intermediate Software Development MOOC team
 * @author Dominik Sudwischer
 * Date: 20171016
 * */
public class EarthquakeCityMap extends PApplet {

	// You can ignore this.  It's to keep eclipse from generating a warning.
	private static final long serialVersionUID = 1L;

	// IF YOU ARE WORKING OFFLINE, change the value of this variable to true
	private static final boolean offline = true;
	
	// Less than this threshold is a light earthquake
	public static final float THRESHOLD_MODERATE = 5;
	// Less than this threshold is a minor earthquake
	public static final float THRESHOLD_LIGHT = 4;
	
	// Radius values for markers
	public static final float RADIUS_LIGHT = 8.0f;
	public static final float RADIUS_MODERATE = 14.0f;
	public static final float RADIUS_HEAVY = 20.0f;

	// For a continuous color spectrum, we will declare minimum and maximum values
	// for red and green.
	public static final int RED_LOW = 0;
	public static final int RED_HIGH = 255;
	public static final int GREEN_LOW = 0;
	public static final int GREEN_HIGH = 255;
	
	// For earthquake colors, we will use the maximum color for EQs of magnitude 10.0 and higher
	public static final float MAX_EQ = 6.0f;
	public static final float MIN_EQ = 2.0f;
	
	/** This is where to find the local tiles, for working without an Internet connection */
	public static String mbTilesString = "blankLight-1-3.mbtiles";
	
	// The map
	private UnfoldingMap map;
	
	//feed with magnitude 2.5+ Earthquakes
	private String earthquakesURL = "https://earthquake.usgs.gov/earthquakes/feed/v1.0/summary/2.5_week.atom";

	
	public void setup() {
		size(950, 600, OPENGL);

		if (offline) {
		    map = new UnfoldingMap(this, 200, 20, width - 220, height - 40, new MBTilesMapProvider(mbTilesString));
		    earthquakesURL = "2.5_week.atom"; 	// Same feed, saved Aug 7, 2015, for working offline
		}
		else {
			map = new UnfoldingMap(this, 200, 20, width - 220, height - 40, new Google.GoogleMapProvider());
			// IF YOU WANT TO TEST WITH A LOCAL FILE, uncomment the next line
			//earthquakesURL = "2.5_week.atom";
		}

	    map.zoomToLevel(2);
	    MapUtils.createDefaultEventDispatcher(this, map);	
			
	    // The List you will populate with new SimplePointMarkers
	    List<Marker> markers = new ArrayList<Marker>();

	    //Use provided parser to collect properties for each earthquake
	    //PointFeatures have a getLocation method
	    List<PointFeature> earthquakes = ParseFeed.parseEarthquake(this, earthquakesURL);
	    
	    //TODO (Step 3): Add a loop here that calls createMarker (see below) 
	    // to create a new SimplePointMarker for each PointFeature in 
	    // earthquakes.  Then add each new SimplePointMarker to the 
	    // List markers (so that it will be added to the map in the line below)
	    for(PointFeature pf : earthquakes) {
	    	SimplePointMarker spm = createMarker(pf);
	    	markers.add(spm);
	    }
	    
	    
	    // Add the markers to the map so that they are displayed
	    map.addMarkers(markers);
	}
		
	/* createMarker: A suggested helper method that takes in an earthquake 
	 * feature and returns a SimplePointMarker for that earthquake
	 * 
	 * In step 3 You can use this method as-is.  Call it from a loop in the 
	 * setup method.  
	 * 
	 * TODO (Step 4): Add code to this method so that it adds the proper 
	 * styling to each marker based on the magnitude of the earthquake.  
	*/
	private SimplePointMarker createMarker(PointFeature feature)
	{  
		// To print all of the features in a PointFeature (so you can see what they are)
		// uncomment the line below.  Note this will only print if you call createMarker 
		// from setup
		//System.out.println(feature.getProperties());
		
		// Create a new SimplePointMarker at the location given by the PointFeature
		SimplePointMarker marker = new SimplePointMarker(feature.getLocation());
		
		Object magObj = feature.getProperty("magnitude");
		float mag = Float.parseFloat(magObj.toString());
		
		// Here is an example of how to use Processing's color method to generate 
	    // an int that represents the color yellow.  
	    //int yellow = color(255, 255, 0);
		
		// TODO (Step 4): Add code below to style the marker's size and color 
	    // according to the magnitude of the earthquake.  
	    // Don't forget about the constants THRESHOLD_MODERATE and 
	    // THRESHOLD_LIGHT, which are declared above.
	    // Rather than comparing the magnitude to a number directly, compare 
	    // the magnitude to these variables (and change their value in the code 
	    // above if you want to change what you mean by "moderate" and "light")
	    
		// Ok, so here is the plan: for the radius of the marker, I will use three
		// different discrete values. Values below THRESHOLD_LIGHT will have a small
		// radius, values higher than THRESHOLD_MODERATE will have a large radius.
		if(mag > THRESHOLD_MODERATE) {
			marker.setRadius(RADIUS_HEAVY);
		}
		else if(mag > THRESHOLD_LIGHT) {
			marker.setRadius(RADIUS_MODERATE);
		}
		else {
			marker.setRadius(RADIUS_LIGHT);
		}
		
		// Next, we will use a continuous color spectrum to mark the values.  
		// We will use a maximum value of MAX_EQ with pure red and MIN_EQ with pure green.
		if(mag > MAX_EQ) {
			mag = MAX_EQ;
		}
		else if(mag < MIN_EQ) {
			mag = MIN_EQ;
		}
		
		mag = normalize(mag, MIN_EQ, MAX_EQ);
	    marker.setColor(color((int) 255 * cap(mag), (int) 255 * activate((1 - mag)), 0));
		
	    // Finally return the marker
	    return marker;
	}
	
	// The next method scales a value in a given range to a number between 0 and 1 with a
	// linear transformation.
	private float normalize(float val, float oldMin, float oldMax)
	{
		oldMax -= oldMin;
		val -= oldMin;
		oldMin = 0.0f;
		return val / oldMax;
	}
	
	private float cap(float val) {
		return Math.min(2 * val, 1.0f);
	}
	
	private float activate(float val) {
		return Math.max(0.0f, 2 * (val - 0.05f));
	}
	
	public void draw() {
	    background(120);
	    map.draw();
	    addKey();
	}

	private void addEntryToKey(float radius, String s, int dist, int centerOffset,
								int xCenter, int yCenter, int r, int g, int b)
	{
		fill(r, g, b);
		ellipse(xCenter, yCenter, radius, radius);
		fill(0);
		text(s, xCenter + dist, yCenter + centerOffset);
	}
	
	// helper method to draw key in GUI
	// TODO: Implement this method to draw the key
	private void addKey() 
	{	
		// Remember you can use Processing's graphics methods here
		fill(200);
		int rectLengthY = 220;
		int rectStartY = height / 2 - rectLengthY / 2;
		int rectStartX = 20;
		rect(rectStartX, rectStartY, 160, rectLengthY, 7);
		addEntryToKey(RADIUS_HEAVY, "Mag > " + THRESHOLD_MODERATE,
						(int) (1.5 * RADIUS_HEAVY / 2), 4, 30 + rectStartX,
						30 + rectStartY, 255, 0, 0);
		addEntryToKey(RADIUS_MODERATE, THRESHOLD_LIGHT + " < Mag <= " + THRESHOLD_MODERATE,
						(int) (1.5 * RADIUS_HEAVY / 2), 4, 30 + rectStartX,
						30 + rectStartY + 2 * (int) RADIUS_HEAVY, 255, 255, 0);
		addEntryToKey(RADIUS_LIGHT, "Mag <= " + THRESHOLD_MODERATE,
						(int) (1.5 * RADIUS_HEAVY / 2), 4, 30 + rectStartX,
						30 + rectStartY + 4 * (int) RADIUS_HEAVY, 0, 255, 0);
		text("Colors represent the\nmagnitudefrom green\n(minor) to red (moderate)\nin a continuous spectrum.",
				10 + rectStartX, 30 + rectStartY + 6 * (int) RADIUS_HEAVY);
	}
}
