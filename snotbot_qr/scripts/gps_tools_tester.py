import gps_tools

p1 = gps_tools.Point(42.2936, -71.26413, 0) # Start of 100 m track
p2 = gps_tools.Point(42.2935, -71.2641, 0) # End of 100 m track

def distance_unittest():
	return gps_tools.distance(p1, p2)

def bearing_unittest():
	return gps_tools.bearing(p1, p2)

print distance_unittest()