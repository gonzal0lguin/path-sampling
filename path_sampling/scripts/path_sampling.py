import numpy as np
import numpy.linalg as ln 


class PathSampler(object):
    def __init__(self, waypoint_dst=0.5):
        self._waypoint_dst = waypoint_dst

    def sample_plan_uniform(self, path: np.ndarray):
        L = 0
        waypoints = []
        for i in range(1, len(path)):
            segment_l = self.calculate_segment_lenght(
                path[i-1][:2],
                path[i][:2]
            ) 
            L += segment_l
            
            if L >= self._waypoint_dst:
                waypoints.append(path[i])
                L -= self._waypoint_dst

        if L < self._waypoint_dst and len(waypoints)>0:
            waypoints.pop()
        waypoints.append(path[-1])

        return waypoints

    def sample_curvature(self):
        pass

    def sample_direction(self):
        pass
    

    @staticmethod
    def calculate_segment_lenght(s1, s2):
        x1, y1 = s1
        x2, y2 = s2
        return np.sqrt( (x1 - x2)**2 + (y1 - y2)**2 )