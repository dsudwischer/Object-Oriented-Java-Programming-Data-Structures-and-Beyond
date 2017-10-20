package module6;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import de.fhpotsdam.unfolding.UnfoldingMap;
import de.fhpotsdam.unfolding.data.Feature;
import de.fhpotsdam.unfolding.data.GeoJSONReader;
import de.fhpotsdam.unfolding.data.PointFeature;
import de.fhpotsdam.unfolding.data.ShapeFeature;
import de.fhpotsdam.unfolding.geo.Location;
import de.fhpotsdam.unfolding.marker.AbstractShapeMarker;
import de.fhpotsdam.unfolding.marker.Marker;
import de.fhpotsdam.unfolding.marker.MultiMarker;
import de.fhpotsdam.unfolding.providers.Google;
import de.fhpotsdam.unfolding.providers.MBTilesMapProvider;
import de.fhpotsdam.unfolding.utils.MapUtils;
import module6.EarthquakeMarker;
import module6.CommonMarker;
import parsing.ParseFeed;
import processing.core.PApplet;

/** TODO:
 * Write functions to
 * - hide all airports
 * - make all connected airports visible again
 */

/** EarthquakeCityMap
 * An application with an interactive map displaying earthquake data.
 * Author: UC San Diego Intermediate Software Development MOOC team
 * @author Dominik Sudwischer
 * Date: July 17, 2015
 * */
public class EarthquakeCityMap extends PApplet {

	// We will use member variables, instead of local variables, to store the data
	// that the setUp and draw methods will need to access (as well as other methods)
	// You will use many of these variables, but the only one you should need to add
	// code to modify is countryQuakes, where you will store the number of earthquakes
	// per country.

	// You can ignore this.  It's to get rid of eclipse warnings
	private static final long serialVersionUID = 1L;

	// IF YOU ARE WORKING OFFILINE, change the value of this variable to true
	private static final boolean offline = true;

	/** This is where to find the local tiles, for working without an Internet connection */
	public static String mbTilesString = "blankLight-1-3.mbtiles";



	//feed with magnitude 2.5+ Earthquakes
	private String earthquakesURL = "https://earthquake.usgs.gov/earthquakes/feed/v1.0/summary/2.5_week.atom";

	// The files containing city names and info and country names and info
	private String cityFile = "city-data.json";
	private String countryFile = "countries.geo.json";

	// The map
	private UnfoldingMap map;

	// Markers for each city
	private List<Marker> cityMarkers;
	// Markers for each earthquake
	private List<Marker> quakeMarkers;

	// A List of country markers
	private List<Marker> countryMarkers;

	// A list of marker for airports and routes
	private List<Marker> airportMarkers;
	//HashMap<Integer, Location> airports;
	HashMap<Integer, AirportMarker> airports;
	HashMap<Integer, List<Integer>> outgoingRoutes;

	// NEW IN MODULE 5
	private CommonMarker lastSelected;
	private CommonMarker lastClicked;
	private boolean drawLinesBetweenOceanQuakeAndCities = false;
	private boolean drawLinesBetweenAirports = false;

	public void setup() {		
		// (1) Initializing canvas and map tiles
		size(900, 700, OPENGL);
		if (offline) {
			map = new UnfoldingMap(this, 200, 50, 650, 600, new MBTilesMapProvider(mbTilesString));
			earthquakesURL = "2.5_week.atom";  // The same feed, but saved August 7, 2015
		}
		else {
			map = new UnfoldingMap(this, 200, 50, 650, 600, new Google.GoogleMapProvider());
			// IF YOU WANT TO TEST WITH A LOCAL FILE, uncomment the next line
			//earthquakesURL = "2.5_week.atom";
		}
		MapUtils.createDefaultEventDispatcher(this, map);

		// FOR TESTING: Set earthquakesURL to be one of the testing files by uncommenting
		// one of the lines below.  This will work whether you are online or offline
		//earthquakesURL = "test1.atom";
		//earthquakesURL = "test2.atom";

		// Uncomment this line to take the quiz
		//earthquakesURL = "quiz2.atom";


		// (2) Reading in earthquake data and geometric properties
		//     STEP 1: load country features and markers
		List<Feature> countries = GeoJSONReader.loadData(this, countryFile);
		countryMarkers = MapUtils.createSimpleMarkers(countries);

		//     STEP 2: read in city data
		List<Feature> cities = GeoJSONReader.loadData(this, cityFile);
		cityMarkers = new ArrayList<Marker>();
		for(Feature city : cities) {
			cityMarkers.add(new CityMarker(city));
		}

		//     STEP 3: read in earthquake RSS feed
		List<PointFeature> earthquakes = ParseFeed.parseEarthquake(this, earthquakesURL);
		quakeMarkers = new ArrayList<Marker>();

		for(PointFeature feature : earthquakes) {
			//check if LandQuake
			if(isLand(feature)) {
				quakeMarkers.add(new LandQuakeMarker(feature));
			}
			// OceanQuakes
			else {
				quakeMarkers.add(new OceanQuakeMarker(feature));
			}
		}

		//	STEP 4: read in airport data
		// get features from airport data
		List<PointFeature> features = ParseFeed.parseAirports(this, "airports.dat");

		// list for markers, hashmap for quicker access when matching with routes
		airportMarkers = new ArrayList<Marker>();
		//airports = new HashMap<Integer, Location>();
		airports = new HashMap<Integer, AirportMarker>();

		// create markers from features
		for(PointFeature feature : features) {
			AirportMarker m = new AirportMarker(feature);

			m.setRadius(5);
			airportMarkers.add(m);

			// put airport in hashmap with OpenFlights unique id for key
			airports.put(Integer.parseInt(feature.getId()), m);
		}

		// parse route data
		List<ShapeFeature> routes = ParseFeed.parseRoutes(this, "routes.dat");
		outgoingRoutes = new HashMap<Integer, List<Integer>>();
		for(ShapeFeature route : routes) {

			// get source and destination airportIds
			int source = Integer.parseInt((String)route.getProperty("source"));
			int dest = Integer.parseInt((String)route.getProperty("destination"));

			// get locations for airports on route
			if(airports.containsKey(source) && airports.containsKey(dest)) {
				route.addLocation(airports.get(source).getLocation());
				route.addLocation(airports.get(dest).getLocation());
			}

			//SimpleLinesMarker sl = new SimpleLinesMarker(route.getLocations(), route.getProperties());

			//System.out.println(sl.getProperties());

			// Add route data to HashMap for outgoing connections
			if(outgoingRoutes.containsKey(source))
			{
				outgoingRoutes.get(source).add(dest);
			}
			else
			{
				// In this case, we first have to add a new list associated with the key source
				outgoingRoutes.put(source, new ArrayList<Integer>());
				outgoingRoutes.get(source).add(dest);
			}
		}

		// could be used for debugging
		//printQuakes();

		// (3) Add markers to map
		//     NOTE: Country markers are not added to the map.  They are used
		//           for their geometric properties
		map.addMarkers(quakeMarkers);
		map.addMarkers(cityMarkers);
		map.addMarkers(airportMarkers);
	}  // End setup


	public void draw() {
		background(0);
		map.draw();
		if(drawLinesBetweenOceanQuakeAndCities && lastClicked != null)
		{
			((CommonMarker) lastClicked).drawLines(this, true, cityMarkers, map);
		}
		if(drawLinesBetweenAirports && lastClicked != null)
		{
			drawRoutes(lastClicked, outgoingRoutes, airports);
		}
		addKey();
	}


	// TODO: Add the method:
	private void sortAndPrint(int numToPrint)
	{
		Object[] sortedQuakes = quakeMarkers.toArray();
		Arrays.sort(sortedQuakes, Collections.reverseOrder());
		numToPrint = Math.min(numToPrint, sortedQuakes.length);
		for(int i = 0; i < numToPrint; i++)
		{
			EarthquakeMarker m = (EarthquakeMarker)sortedQuakes[i];
			System.out.println(m.getTitle());
		}
	}
	// and then call that method from setUp

	/** Event handler that gets called automatically when the 
	 * mouse moves.
	 */
	@Override
	public void mouseMoved()
	{
		// clear the last selection
		if (lastSelected != null) {
			lastSelected.setSelected(false);
			lastSelected = null;

		}
		selectMarkerIfHover(quakeMarkers);
		if(lastSelected != null) { return; }
		selectMarkerIfHover(cityMarkers);
		if(lastSelected != null) { return; }
		selectMarkerIfHover(airportMarkers);
	}


	// If there is a marker under the cursor, and lastSelected is null 
	// set the lastSelected to be the first marker found under the cursor
	// Make sure you do not select two markers.
	// 
	private void selectMarkerIfHover(List<Marker> markers)
	{
		CommonMarker m = findMarker(markers);
		if(m == null)
		{
			return;
		}
		m.setSelected(true);
		lastSelected = m;
	}

	/** The event handler for mouse clicks
	 * It will display an earthquake and its threat circle of cities
	 * Or if a city is clicked, it will display all the earthquakes 
	 * where the city is in the threat circle
	 */
	@Override
	public void mouseClicked()
	{
		// Hint: You probably want a helper method or two to keep this code
		// from getting too long/disorganized
		drawLinesBetweenOceanQuakeAndCities = false;
		drawLinesBetweenAirports = false;
		if(lastClicked != null)
		{
			unhideMarkers();
			// Redraw lines between OceanQuakes and cities invisibly.
			((CommonMarker) lastClicked).drawLines(this, false, cityMarkers, map);
			lastClicked = null;
			return;
		}
		CommonMarker currentlyClicked = findMarker(quakeMarkers);
		if(currentlyClicked != null)
		{
			hideMarkersAfterEQMarkerClick(currentlyClicked);
			if(!((EarthquakeMarker) currentlyClicked).isOnLand())
			{
				// Draw lines between the OceanQuake and cities within its threat radius.
				drawLinesBetweenOceanQuakeAndCities = true;
				currentlyClicked.drawLines(this, true, cityMarkers, map);
			}
			lastClicked = currentlyClicked;
			return;
		}
		// Ok, so it was not an earthquake that was clicked. Let us look if it was a city.
		currentlyClicked = findMarker(cityMarkers);
		if(currentlyClicked != null)
		{
			hideMarkersAfterCityMarkerClick(currentlyClicked);
			lastClicked = currentlyClicked;
			return;
		}
		// Apparently it was not a city either, so let us check for airports.
		currentlyClicked = findMarker(airportMarkers);
		if(currentlyClicked != null)
		{
			drawLinesBetweenAirports = true;
			drawRoutes(currentlyClicked, outgoingRoutes, airports);
			hideMarkersAfterAirportMarkerClick(currentlyClicked);
			lastClicked = currentlyClicked;
			return;
		}
		// Nope, then it was a click into the void apparently.
		lastClicked = null;
	}

	// Find the first marker which the mouse is inside of
	private CommonMarker findMarker(List<Marker> markers)
	{
		for(Marker m : markers)
		{
			if(m.isInside(map, mouseX, mouseY))
			{
				// In this case, the mouse is inside the marker m
				return (CommonMarker) m;
			}
		}
		return null;
	}

	private void drawRoutes(CommonMarker m, HashMap<Integer, List<Integer>> outgoingRoutes,
			HashMap<Integer, AirportMarker> airports)
	{
		this.stroke(150);
		this.strokeWeight(2);
		int sourceID = Integer.parseInt(m.getId());
		if(!outgoingRoutes.containsKey(sourceID)) { return; }
		for(int destinationID : outgoingRoutes.get(sourceID))
		{
			m.drawLine(this, m.getScreenPosition(map),
					airports.get(destinationID).getScreenPosition(map));
		}

	}

	// Set all cities far away from m to be hidden.
	private void hideDistantCities(CommonMarker m, List<Marker> otherMarkers, double radius)
	{
		for(Marker otherMarker : cityMarkers)
		{
			if(((CommonMarker) m).getDist((CommonMarker) otherMarker) > radius)
			{
				otherMarker.setHidden(true);
			}
		}
	}

	// This method is a wrapper to hide markers after an earthQuakeMarker click
	private void hideMarkersAfterEQMarkerClick(CommonMarker eq)
	{
		hideDistantCities(eq, cityMarkers,
				((EarthquakeMarker) eq).threatCircle());
		hideOtherMarkers(eq, quakeMarkers);
	}

	// This method is a wrapper to hide markers after an airportMarker click
	private void hideMarkersAfterAirportMarkerClick(CommonMarker airport)
	{
		hideOtherMarkers(airport, quakeMarkers);
		hideOtherMarkers(airport, cityMarkers);
		hideOtherMarkers(airport, airportMarkers);
		try
		{
			showSpecificMarkers(outgoingRoutes.get(Integer.parseInt(airport.getId())), airports);
		}
		catch(Exception e)
		{
			System.out.println("Conversion of airport ID to Integer not possible" );
		}
	}


	// Hides all cities other than city
	private void hideOtherMarkers(CommonMarker m, List<Marker> otherMarkers)
	{
		for(Marker otherMarker : otherMarkers)
		{
			otherMarker.setHidden(true);
		}
		m.setHidden(false);
	}

	// This method shows (i.e. un-hides) some markers based on a list
	private void showSpecificMarkers(List<Integer> airportIDs, HashMap<Integer, AirportMarker> airports)
	{
		for(int id : airportIDs)
		{
			if(airports.get(id) != null) {
				//System.out.println(airports.get(id));
				airports.get(id).setHidden(false);}
		}
	}

	// Hides all earthquakes that are distant from city
	private void hideEarthquakesAfterCityMarkerClick(CommonMarker city, List<Marker> earthquakes)
	{
		for(Marker eq : earthquakes)
		{
			if(((EarthquakeMarker) eq).getDist(city) > ((EarthquakeMarker) eq).threatCircle())
			{
				// In this case, the city is safe
				eq.setHidden(true);
			}
		}
	}

	// Wrapper method to hide markers after cityMarker click
	private void hideMarkersAfterCityMarkerClick(CommonMarker city)
	{
		hideOtherMarkers(city, cityMarkers);
		hideEarthquakesAfterCityMarkerClick(city, quakeMarkers);
	}

	// loop over and unhide all markers
	private void unhideMarkers() {
		for(Marker marker : quakeMarkers) {
			marker.setHidden(false);
		}

		for(Marker marker : cityMarkers) {
			marker.setHidden(false);
		}

		for(Marker marker : airportMarkers) {
			marker.setHidden(false);
		}
	}

	// helper method to draw key in GUI
	private void addKey() {	
		// Remember you can use Processing's graphics methods here
		fill(255, 250, 240);

		int xbase = 25;
		int ybase = 50;

		rect(xbase, ybase, 150, 250);

		fill(0);
		textAlign(LEFT, CENTER);
		textSize(12);
		text("Earthquake Key", xbase+25, ybase+25);

		fill(150, 30, 30);
		int tri_xbase = xbase + 35;
		int tri_ybase = ybase + 50;
		triangle(tri_xbase, tri_ybase-CityMarker.TRI_SIZE, tri_xbase-CityMarker.TRI_SIZE, 
				tri_ybase+CityMarker.TRI_SIZE, tri_xbase+CityMarker.TRI_SIZE, 
				tri_ybase+CityMarker.TRI_SIZE);

		fill(0, 0, 0);
		textAlign(LEFT, CENTER);
		text("City Marker", tri_xbase + 15, tri_ybase);

		text("Land Quake", xbase+50, ybase+70);
		text("Ocean Quake", xbase+50, ybase+90);
		text("Airport", xbase+25, ybase+110);

		fill(255, 255, 255);
		ellipse(xbase+35, 
				ybase+70, 
				10, 
				10);
		rect(xbase+35-5, ybase+90-5, 10, 10);
		
		fill(255, 0, 255);
		ellipse(xbase+35, ybase+70, 5, 5);

		fill(color(255, 255, 0));
		ellipse(xbase+35, ybase+140, 12, 12);
		fill(color(0, 0, 255));
		ellipse(xbase+35, ybase+160, 12, 12);
		fill(color(255, 0, 0));
		ellipse(xbase+35, ybase+180, 12, 12);

		textAlign(LEFT, CENTER);
		fill(0, 0, 0);
		text("Shallow", xbase+50, ybase+140);
		text("Intermediate", xbase+50, ybase+160);
		text("Deep", xbase+50, ybase+180);

		text("Past hour", xbase+50, ybase+200);
		
		text("Size ~ Magnitude", xbase+25, ybase+220);
		
		fill(255, 255, 255);
		int centerx = xbase+35;
		int centery = ybase+200;
		ellipse(centerx, centery, 12, 12);

		strokeWeight(2);
		line(centerx-8, centery-8, centerx+8, centery+8);
		line(centerx-8, centery+8, centerx+8, centery-8);


	}



	// Checks whether this quake occurred on land.  If it did, it sets the 
	// "country" property of its PointFeature to the country where it occurred
	// and returns true.  Notice that the helper method isInCountry will
	// set this "country" property already.  Otherwise it returns false.
	private boolean isLand(PointFeature earthquake) {

		// IMPLEMENT THIS: loop over all countries to check if location is in any of them
		// If it is, add 1 to the entry in countryQuakes corresponding to this country.
		for (Marker country : countryMarkers) {
			if (isInCountry(earthquake, country)) {
				return true;
			}
		}

		// not inside any country
		return false;
	}

	// prints countries with number of earthquakes
	// You will want to loop through the country markers or country features
	// (either will work) and then for each country, loop through
	// the quakes to count how many occurred in that country.
	// Recall that the country markers have a "name" property, 
	// And LandQuakeMarkers have a "country" property set.
	private void printQuakes() 
	{
		HashMap<String, Integer> eqCounter = countEQs();
		int numOceanQuakes = 0;
		for(HashMap.Entry<String, Integer> entry : eqCounter.entrySet())
		{
			String country = entry.getKey();
			int numQuakes = entry.getValue();
			if(!country.equals("_Ocean"))
			{
				System.out.println(country + ": " + numQuakes);
			}
			else
			{
				numOceanQuakes = numQuakes;
			}
		}
		System.out.println("In the ocean: " + numOceanQuakes);
	}

	private HashMap<String, Integer> countEQs()
	{
		// Against the advice of the instructors, I will use a more efficient implementation
		// with a HashMap.
		HashMap<String, Integer> eqCounter = new HashMap<String, Integer>();
		int numOceanQuakes = 0;
		for(Marker m : quakeMarkers)
		{
			if(m.getProperties().containsKey("country"))
			{
				// In this case, it is a landQuake!
				String country = (String) m.getProperty("country");
				if(eqCounter.containsKey(country))
				{
					eqCounter.put(country, eqCounter.get(country) + 1);
				}
				else // In this case, we must add this key with a value of 1
				{
					eqCounter.put(country, 1);
				}
			}
			else // It is an oceanQuake!
			{
				numOceanQuakes += 1;
			}
		}
		eqCounter.put("_Ocean", numOceanQuakes); // The leading underscore ensures no collisions
		// with countries (idk if there is a country called "Ocean" ...)
		return eqCounter;
	}


	// helper method to test whether a given earthquake is in a given country
	// This will also add the country property to the properties of the earthquake feature if 
	// it's in one of the countries.
	// You should not have to modify this code
	private boolean isInCountry(PointFeature earthquake, Marker country) {
		// getting location of feature
		Location checkLoc = earthquake.getLocation();

		// some countries represented it as MultiMarker
		// looping over SimplePolygonMarkers which make them up to use isInsideByLoc
		if(country.getClass() == MultiMarker.class) {

			// looping over markers making up MultiMarker
			for(Marker marker : ((MultiMarker)country).getMarkers()) {

				// checking if inside
				if(((AbstractShapeMarker)marker).isInsideByLocation(checkLoc)) {
					earthquake.addProperty("country", country.getProperty("name"));

					// return if is inside one
					return true;
				}
			}
		}

		// check if inside country represented by SimplePolygonMarker
		else if(((AbstractShapeMarker)country).isInsideByLocation(checkLoc)) {
			earthquake.addProperty("country", country.getProperty("name"));

			return true;
		}
		return false;
	}

}
