from shapely.geometry import Point, MultiPoint, shape
from shapely.geometry.polygon import Polygon
import matplotlib.pyplot as plt


##-----------------------------------------------
point = Point((1.5, 1))

##-----------------------------------------------
# polys = [(1,0), (2,1),(2,2), (1,2)]
polys = [(2,1),(1,0),(2,2), (1,2)]

##-----------------------------------------------
polygon = Polygon(polys)
poly_bound = polygon.boundary.coords
# print("poly_bound", poly_bound[:])

##-----------------------------------------------
points = [Point(p) for p in  polys]
mpt = MultiPoint([shape(p) for p in points])
mp = mpt.convex_hull
mp_bound = mp.boundary.coords
print("mp", mp)

##-----------------------------------------------
# print(point.coords[0])
is_in = polygon.contains(point)
print(" ========= is_in ", is_in)

is_in = mp.contains(point)
print(" ========= is_in ", is_in)

##-----------------------------------------------
polys_x = [p[0] for p in poly_bound]
polys_y = [p[1] for p in poly_bound]
plt.plot(polys_x, polys_y)
plt.plot(point.coords[0][0], point.coords[0][1], 'go')

mp_x = [p[0] for p in mp_bound]
mp_y = [p[1] for p in mp_bound]
plt.plot(mp_x, mp_y)


##-----------------------------------------------

plt.show()
