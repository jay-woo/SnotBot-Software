from math import radians, cos, sin, asin, atan2, sqrt, pi

class Point():
	def __init__(self, longitude, latitude, altitude):
		self.x = longitude
		self.y = latitude
		self.z = altitude

def distance(point1, point2):
	''' Calculates the ground distance between two GPS coordinates
		using their latitudes and longitudes. Uses the equirectangular
		approximation.

		Inputs: point1 (Point object)
				point2 (Point object)

		Output: distance (float)
	'''
	lon1, lat1, lon2, lat2 = map(radians, [point1.x, point1.y, point2.x, point2.y])

	dlon = lon2 - lon1
	dlat = lat2 - lat1
	x = dlat * cos((lon1 + lon2) / 2)
	y = dlon
	r = 6371
	d = sqrt(x**2 + y**2) * r

	return d

def bearing(point1, point2):
	''' Calculates the bearing between the two points, assuming
		that north is 0 radians. Uses Cartesian approximation.

		Inputs: point1 (Point object)
				point2 (Point object)

		Output: bearing (float)
	'''

	lon1, lat1, lon2, lat2 = map(radians, [point1.x, point1.y, point2.x, point2.y])

	dlon = lon2 - lon1
	dlat = lat2 - lat1
	b = atan2(dlon, dlat)

	if b < 0:
		b += 2 * pi

	return b