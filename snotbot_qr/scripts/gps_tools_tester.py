import gps_tools

def distance_unittest():
	p1 = gps_tools.Point(40.951036, -73.963374, 0) # Start of 100 m track
	p2 = gps_tools.Point(40.951920, -73.963407, 0) # End of 100 m track

	return gps_tools.distance(p1, p2)

def bearing_unittest():
	p1 = gps_tools.Point(0, -90, 0) # Olin
	p2 = gps_tools.Point(0, 90, 0) # NW of Olin

	return gps_tools.bearing(p1, p2)

print bearing_unittest()