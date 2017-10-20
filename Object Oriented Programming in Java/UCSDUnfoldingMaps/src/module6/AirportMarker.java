package module6;

import java.util.List;

import de.fhpotsdam.unfolding.data.Feature;
import de.fhpotsdam.unfolding.data.PointFeature;
import de.fhpotsdam.unfolding.marker.SimpleLinesMarker;
import processing.core.PConstants;
import processing.core.PGraphics;

/** 
 * A class to represent AirportMarkers on a world map.
 *   
 * @author Adam Setters and the UC San Diego Intermediate Software Development
 * MOOC team
 *
 */
public class AirportMarker extends CommonMarker {
	public static List<SimpleLinesMarker> routes;
	
	public static int CIRCLE_SIZE = 5;  // The size of the circle marker
	
	public AirportMarker(Feature city) {
		super(((PointFeature)city).getLocation(), city.getProperties());
		this.setId(city.getId());
	}
	
	@Override
	public void drawMarker(PGraphics pg, float x, float y) {
		pg.fill(255, 0, 255);
		pg.ellipse(x, y, 5, 5);
		
		
	}

	@Override
	public void showTitle(PGraphics pg, float x, float y) {
		 // show rectangle with title
		String city = getCity() + " " + getCountry() + " ";
		String name = getName();
		
		pg.pushStyle();
		
		pg.fill(255, 255, 255);
		pg.textSize(12);
		pg.rectMode(PConstants.CORNER);
		pg.rect(x, y-CIRCLE_SIZE-39, 2 * Math.max(pg.textWidth(name), pg.textWidth(city)) + 6, 39);
		pg.fill(0, 0, 0);
		pg.textAlign(PConstants.LEFT, PConstants.TOP);
		pg.text(name, x+3, y-CIRCLE_SIZE-33);
		pg.text(city, x+3, y - CIRCLE_SIZE -18);
	}

	private String getName()
	{
		return getStringProperty("name");
	}
	
	private String getCity()
	{
		return getStringProperty("city");
	}
	
	private String getCountry()
	{
		return getStringProperty("country");
	}
	
}
