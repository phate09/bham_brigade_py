import os
import sys
import math
import numpy as np
from scipy.spatial import Delaunay
from scipy.spatial import ConvexHull

import numpy as np

from shapely.ops import cascaded_union, polygonize
import shapely.geometry as geometry

class FHPolygon():
    # Array of 2D points
    def __init__(self):
        self.points = []



def calculate_convex_hull_polygon(coords):
    convexHull = ConvexHull(coords)

    fhpolygon = FHPolygon()

    for vertex_index in convexHull.vertices:
        fhpolygon.points.append(convexHull.points[vertex_index])

    # This is always a single polygon but want to return an array.
    return [fhpolygon]

def calculate_alpha_shape_polygons(coords):
    multi_polygon = alpha_shape(coords)

    print("Multi Polygon")

    print(multi_polygon)
    polygons = list(multi_polygon)

    polygons_list = []

    for polygon in polygons:
        fhpolygon = FHPolygon()
        print(polygon.exterior)
        fhpolygon.points = polygon.exterior.coords

        polygons_list.append(fhpolygon)

    return polygon_list



def calculate_polygons(coords):
    # Choice of methods to compute polygon.

    print("convex hull")
    polygons_list_1 = calculate_convex_hull_polygon(coords)

    #print("alpha")
    #polygon_list = calculate_alpha_shape_polygons(coords)

    print(polygons_list_1)
    #print(polygons_list)

    return polygons_list_1






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

def alpha_shape(points, alpha=2):
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




    coords = np.array([point.coords[0] for point in points])
    tri = Delaunay(coords)
    edges = set()
    edge_points = []

    # loop over triangles:
    # ia, ib, ic = indices of corner points of the
    # triangle
    print(tri)
    for ia, ib, ic in tri.vertices:
        pa = coords[ia]
        pb = coords[ib]
        pc = coords[ic]

        # Lengths of sides of triangle
        a = math.sqrt((pa[0] - pb[0])**2 + (pa[1] - pb[1])**2)
        b = math.sqrt((pb[0] - pc[0])**2 + (pb[1] - pc[1])**2)
        c = math.sqrt((pc[0] - pa[0])**2 + (pc[1] - pa[1])**2)

        # Semiperimeter of triangle
        s = (a + b + c) / 2.0

    # Area of triangle by Heron's formula
    area = math.sqrt(s * (s - a) * (s - b) * (s - c))
    circum_r = a * b * c / (4.0 * area)

    # Here's the radius filter.
    #print(circum_r, 1.0 / alpha)
    if circum_r < 1.0 / alpha:
        add_edge(edges, edge_points, coords, ia, ib)
        add_edge(edges, edge_points, coords, ib, ic)
        add_edge(edges, edge_points, coords, ic, ia)

    m = geometry.MultiLineString(edge_points)
    triangles = list(polygonize(m))

    return cascaded_union(triangles), edge_points
