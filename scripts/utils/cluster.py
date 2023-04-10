import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

def cluster_circles(id_numbers, centers, R, D):
    clusters = []
    while id_numbers:
        id_number = id_numbers.pop()
        cluster = [id_number]
        for other_id in id_numbers.copy():
            distance = math.sqrt((centers[id_number]['x'] - centers[other_id]['x'])**2 + (centers[id_number]['y'] - centers[other_id]['y'])**2)
            if distance <= 2*R + D:
                cluster.append(other_id)
                id_numbers.remove(other_id)
        clusters.append(cluster)
    return clusters

def plot_circles(centers, R, clusters):
    fig, ax = plt.subplots()
    for id_number in centers:
        circle = Circle((centers[id_number]['x'], centers[id_number]['y']), R, fill=False)
        ax.add_patch(circle)
    for cluster in clusters:
        if len(cluster) > 1:
            x_coords = [centers[id_number]['x'] for id_number in cluster]
            y_coords = [centers[id_number]['y'] for id_number in cluster]
            x_center = sum(x_coords)/len(x_coords)
            y_center = sum(y_coords)/len(y_coords)
            circle = Circle((x_center, y_center), R + D/2, fill=False, color='r')
            ax.add_patch(circle)
    ax.set_aspect('equal', adjustable='box')
    plt.show()

id_numbers = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
centers = {1: {'x': 1, 'y': 2}, 2: {'x': 3, 'y': 4}, 3: {'x': 4, 'y': 6}, 4: {'x': 6, 'y': 8}, 5: {'x': 7, 'y': 9},
           6: {'x': 10, 'y': 11}, 7: {'x': 12, 'y': 14}, 8: {'x': 14, 'y': 16}, 9: {'x': 15, 'y': 18}, 10: {'x': 17, 'y': 20}}
R = 1
D = 2

clusters = cluster_circles(id_numbers, centers, R, D)
print(clusters)

plot_circles(centers, R, clusters)
