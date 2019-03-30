import numpy as np


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

    def score_growht(self, point):
        last_polygon = self.polygons[-1]
        pass

    def score_translation(self, point):
        pass

    def score_angle(self, point):
        pass

    def aggregate(self, function):
        '''Aggregates the points in attention window by using the given function'''
        scores = []
        for point in self.attention_window:
            score = function(point)
            scores.append(score)
        std = np.std(scores)
        mean = np.mean(scores)
        return (mean, std)

    def update_beliefs(self, point):
        '''updates all the beliefs based on the new point'''
        if self.attention_window_index < self.attention_window_size:
            self.attention_window.append(point)
        else:
            self.attention_window.insert(self.attention_window_index, point)  # replace old points
        self.attention_window_index = (self.attention_window_index + 1) % self.attention_window_size

        self.growth_mean, self.growth_std = self.aggregate(self.score_growht)
        self.translation_mean, self.translation_std = self.aggregate(self.score_translation)
        self.angle_mean, self.angle_std = self.aggregate(self.score_angle)
        pass
