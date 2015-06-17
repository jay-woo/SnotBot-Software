import gps_tools

p1 = gps_tools.Point(0, 0, 0) # Start of 100 m track
p2 = gps_tools.Point(-1, 90, 0) # End of 100 m track

def distance_unittest():
	return gps_tools.distance(p1, p2)

def bearing_unittest():
	return gps_tools.bearing(p1, p2)

print bearing_unittest()