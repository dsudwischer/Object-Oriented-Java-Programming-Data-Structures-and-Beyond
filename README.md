# Object Oriented Java Programming: Data Structures and Beyond by the University of California, San Diego, offered at Coursera.

This is the first course in a five course specialization.

I am taking this course to learn Java coming from a background of mainly C++ and Python.

The course utilizes the UnfoldingMaps and Processing libraries to build an interactive map applet.

The project taught how to implement an interactive map with the UnfoldingMaps library using the processing library. The map shows cities, earthquakes (both land and ocean quakes) and airports using data
from a websites or CSV files with such information.

The following features have been added by me:
* Markers show additional information on mouseover (e.g. population of a city or magnitude of an earthquake)
* Clicks on cities hide all other cities and earthquakes that did not affect this city.
* Clicks on earthquakes will hide all other earthquakes and all cities were not affected by this earthquake. If an ocean quake was clicked, it will additionally draw lines to affected cites.
* In both above cases, airports will be hidden.
* Clicks on airports hide all other markers except for airports that are directly connected to this airport via flight routes. The flight route data may be incomplete.
* All markers can be revealed again by clicking onto a free space.

Notes:
* A threat circle of an earthquake was calculated using an approximate formula.
* The incompleteness of flight data can be solved by using more flight data. This programming assignment mainly aimed on the technical aspect of implementation rather than completeness of data.
