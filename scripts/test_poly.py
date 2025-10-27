from shapely import Polygon
from matplotlib import pyplot as plt

# define a polygon with non-convex shape
# points = [(0, 0), (1, 0), (0.5, 0.5), (1, 1), (0, 1)]
points = [(0, 0), (1, 0)]

# create a polygon object from a list of points
poly = Polygon(points)

# create convex hull
convex_hull = poly.convex_hull


# plot polygon and its convex hull
x, y = poly.exterior.xy
plt.plot(x, y, color='blue', label='Polygon')
x_h, y_h = convex_hull.exterior.xy
plt.plot(x_h, y_h, color='red', linestyle='--', label='Convex Hull')
plt.fill(x, y, alpha=0.5, fc='lightblue', ec='none')
plt.fill(x_h, y_h, alpha=0.3, fc='lightcoral', ec='none')
plt.legend()
plt.title('Polygon and its Convex Hull')
plt.show()
