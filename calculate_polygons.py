import os
import sys
import math
import numpy as np
from scipy.spatial import Delaunay
from scipy.spatial import ConvexHull

import sklearn.cluster
import sklearn.metrics
import time
import numpy as np

from shapely.ops import cascaded_union, polygonize
import shapely.geometry as geometry


class FHPolygon():
    # Array of 2D points
    def __init__(self):
        self.points = []


def calculate_convex_hull_polygon(coords):
    if len(coords) < 4:
        fhpolygon = FHPolygon()
        fhpolygon.points = coords
        return [fhpolygon]

    convexHull = ConvexHull(coords, qhull_options='QbB')

    fhpolygon = FHPolygon()

    for vertex_index in convexHull.vertices:
        fhpolygon.points.append(convexHull.points[vertex_index])

    # This is always a single polygon but want to return an array.
    return [fhpolygon]


def calculate_alpha_shape_polygons(coords):
    multi_polygon = alpha_shape(coords)

    print("Multi Polygon")
    polygons = list(multi_polygon)

    polygons_list = []

    for polygon in polygons:
        fhpolygon = FHPolygon()
        fhpolygon.points = polygon.exterior.coords

        polygons_list.append(fhpolygon)

    return polygons_list


def cluster_points(coords, n_clusters):
    k_means = sklearn.cluster.KMeans(init='k-means++', n_clusters=n_clusters, n_init=10)
    k_means.fit(coords)
    k_means_cluster_centers = np.sort(k_means.cluster_centers_, axis=0)
    k_means_labels = sklearn.metrics.pairwise_distances_argmin(coords, k_means_cluster_centers)

    clusters = [[] for i in range(n_clusters)]

    for point, label in zip(coords, k_means_labels):
        clusters[label].append(point)

    return clusters


def k_means_clustering(coords, n_clusters):
    k_means = sklearn.cluster.KMeans(init='k-means++', n_clusters=n_clusters, n_init=10,precompute_distances=True,max_iter=100)
    k_means.fit(coords)
    k_means_cluster_centers = np.sort(k_means.cluster_centers_, axis=0)
    k_means_labels = sklearn.metrics.pairwise_distances_argmin(coords, k_means_cluster_centers)
    return coords, k_means_labels


def pick_k(coords):
    scores = []
    max_k = min(6, len(coords) - 1)
    for k in range(1, max_k):
        points, labels = k_means_clustering(coords, k)
        if k == 1:
            score = 0.5
        else:
            score = sklearn.metrics.silhouette_score(points, labels)
        scores.append(score)
    chosen_k = 1 + np.argmax(scores)
    print(f'scores={scores} picking k={chosen_k}')
    return chosen_k


def calculate_k_means_polygons(coords):
    k = pick_k(coords)  # run the method to choose k
    repeat = True
    while (repeat):
        try:
            points, labels = k_means_clustering(coords, k)  # performs k means
            clusters = [[] for i in range(k)]  # generate list of lists
            for point, label in zip(points, labels):  # assign every point to it's coordinate
                clusters[label].append(point)

            # clusters = cluster_points(coords, k)

            polygon_list = []

            for cluster in clusters:
                polygon_list += calculate_convex_hull_polygon(cluster)
            repeat = True
            return polygon_list
        except Exception as ex:
            print(ex)
            print(f"error with k={k},trying with k--")
            k = k - 1

    # return polygon_list


def calculate_polygons(coords):
    # Choice of methods to compute polygon.

    # print("convex hull")
    # polygons_list = calculate_convex_hull_polygon(coords)
    #
    # print("alpha")
    # polygons_list = calculate_alpha_shape_polygons(coords)

    print("kmeans")
    start = time.time()
    polygons_list = calculate_k_means_polygons(coords)
    end = time.time()
    print(f'time to cluster {end - start:.2f} secs')
    # print(polygons_list_1)
    # print(polygons_list)

    return polygons_list


def calculate_silhouette_coefficient(clusters):
    '''calculates the average standard deviation of each cluster, the lower the better'''
    stds = []
    for cluster_index in range(clusters):
        cluster = clusters[cluster_index]
        mean = np.mean(cluster)
        std = np.std(cluster)
        stds.append(std)
    return np.mean(stds)


# From https://tereshenkov.wordpress.com/2017/11/28/building-concave-hulls-alpha-shapes-with-pyqt-shapely-and-arcpy/
def add_edge(edges, edge_points, coords, i, j):
    """
    Add a line between the i-th and j-th points,
    if not in the list already
    """
    if (i, j) in edges or (j, i) in edges:
        # already added
        return
    edges.add((i, j))
    edge_points.append(coords[[i, j]])


def alpha_shape(points, alpha=0.1):
    """
    Compute the alpha shape (concave hull) of a set
    of points.
    @param points: Iterable container of points.
    @param alpha: alpha value to influence the
        gooeyness of the border. Smaller numbers
        don't fall inward as much as larger numbers.
        Too large, and you lose everything!
    """
    if len(points) < 4:
        # When you have a triangle, there is no sense
        # in computing an alpha shape.
        return geometry.MultiPoint(list(points)).convex_hull

    coords = np.array([point for point in points])
    tri = Delaunay(coords)
    edges = set()
    edge_points = []

    # loop over triangles:
    # ia, ib, ic = indices of corner points of the
    # triangle
    # print(tri)
    for ia, ib, ic in tri.vertices:
        pa = coords[ia]
        pb = coords[ib]
        pc = coords[ic]

        # Lengths of sides of triangle
        a = math.sqrt((pa[0] - pb[0]) ** 2 + (pa[1] - pb[1]) ** 2)
        b = math.sqrt((pb[0] - pc[0]) ** 2 + (pb[1] - pc[1]) ** 2)
        c = math.sqrt((pc[0] - pa[0]) ** 2 + (pc[1] - pa[1]) ** 2)

        # Semiperimeter of triangle
        s = (a + b + c) / 2.0

        # Area of triangle by Heron's formula
        area = math.sqrt(s * (s - a) * (s - b) * (s - c))
        circum_r = a * b * c / (4.0 * area)

        # Here's the radius filter.
        # print(circum_r, 1.0 / alpha)
        if circum_r < 1.0 / alpha:
            add_edge(edges, edge_points, coords, ia, ib)
            add_edge(edges, edge_points, coords, ib, ic)
            add_edge(edges, edge_points, coords, ic, ia)

    m = geometry.MultiLineString(edge_points)
    triangles = list(polygonize(m))

    return list(cascaded_union(triangles))  # , edge_points
