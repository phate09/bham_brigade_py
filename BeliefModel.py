import numpy as np
import math


class BeliefModel:
    def __init__(self):
        self.polygons = []  # the polygons at each time t
        self.growth_mean = 0  # estimated growth factor of the fire
        self.growth_std = 0
        self.translation_mean = 0  # estimated translation factor of the fire
        self.translation_std = 0
        self.angle_mean = 0  # estimated angle of translation of the fire
        self.angle_std = 0
        self.attention_window = []  # the list of last n readings to consider when updating the belief
        self.attention_window_index = 0  # the index at which to insert the new readings
        self.attention_window_size = 100  # number of elements to store in the attention window

    def score_growth(self, point):
        #todo check no mistakes
        last_polygon = self.polygons[-1]
        distance_from_hull, closest_point = self.distance_from_hull(last_polygon, point)
        inside = -1 if last_polygon.contains(point) == True else 1
        return distance_from_hull * inside

    def score_translation(self, point):
        #todo do this
        pass

    def score_angle(self, point):
        '''gets the angle between the point and the closest point of the hull so that we get the angle of growth'''
        last_polygon = self.polygons[-1]
        distance_from_hull, closest_point = self.distance_from_hull(last_polygon, point)
        v1_theta = math.atan2(point[0], point[1])
        v2_theta = math.atan2(closest_point[0], closest_point[1])  # v2.y, v2.x)
        r = (v2_theta - v1_theta) * (180.0 / math.pi)

        if r < 0:
            r += 360.0
        #todo check this method works
        return r

    def aggregate(self, function):
        '''Aggregates the points in attention window by using the given function'''
        scores = []
        for point in self.attention_window:
            score = function(point)
            scores.append(score)
        std = np.std(scores)
        mean = np.mean(scores)
        return (mean, std)

    def update_beliefs(self, heatmap,last_detected):
        '''updates all the beliefs based on the new population, the population is inferred from the heatmap and last detected array'''
        self.attention_window = self.get_points_population(heatmap,last_detected) #gets the population

        #updates the estimates
        self.growth_mean, self.growth_std = self.aggregate(self.score_growth)
        self.translation_mean, self.translation_std = self.aggregate(self.score_translation)
        self.angle_mean, self.angle_std = self.aggregate(self.score_angle)
        pass

    def distance_from_hull(self, hull, p):
        '''given a polygon and a point finds the distance from the polygon'''
        dists = []
        points = hull.points
        for i in range(len(points) - 1):
            dists.append(self.dist(points[i][0], points[i][1], points[i + 1][0], points[i + 1][1], p[0], p[1]))
        index = np.argmin(dists)[0]
        dist = dists[index]
        return dist, points[index]

    def get_points_population(self, heatmap, last_detected):
        '''gets the population from the heatmap'''
        coords = []
        last_seen_list = []
        for row in range(heatmap.shape[0]):
            for col in range(heatmap.shape[1]):
                if heatmap[row][col] > 0.95:  # This could be a 1 check but we are pre-empting expanding this for decay.
                    coords.append((row, col))
                    last_seen_list.append(last_detected[row][col])
        sorted_coords = [x for _, x in sorted(zip(last_seen_list, coords))] #sort the coordinates according to last seen
        population = sorted_coords[-self.attention_window_size:] #gets the last elements
        return population

    def dist(self, x1, y1, x2, y2, x3, y3):  # x3,y3 is the point
        '''calculates the distance between a line and a point'''
        px = x2 - x1
        py = y2 - y1

        something = px * px + py * py

        u = ((x3 - x1) * px + (y3 - y1) * py) / float(something)

        if u > 1:
            u = 1
        elif u < 0:
            u = 0

        x = x1 + u * px
        y = y1 + u * py

        dx = x - x3
        dy = y - y3

        # Note: If the actual distance does not matter,
        # if you only want to compare what this function
        # returns to other results of this function, you
        # can just return the squared distance instead
        # (i.e. remove the sqrt) to gain a little performance

        dist = math.sqrt(dx * dx + dy * dy)

        return dist
