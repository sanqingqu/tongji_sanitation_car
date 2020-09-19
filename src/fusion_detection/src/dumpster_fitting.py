import numpy as np
import scipy
import utils
from sklearn.linear_model import RANSACRegressor

def distance_to_line(x, y, p1, p2):
    # line is defined by two points (p1, p2)
    return np.abs((p2[1] - p1[1]) * x - (p2[0] - p1[0]) * y + (p2[0] * p1[1] - p2[1] * p1[0])) / math.sqrt((p2[1]-p1[1])**2 + (p2[0]-p1[0])**2)

class LShapeRegressor:

    """
    parameters:
        - p0 : rightmost point
        - p1 : leftmost point
        - p2 : corner point
    """

    def __init__(self, max_iter=50, stop_error=0.1):
        self.max_iter = max_iter
        self.stop_error = stop_error
        self.model = None
        self.args = None

    def fit(self, x, y):
        # initial value
        x, y = x.flatten(), y.flatten()
        p = np.stack((x, y), axis=1)
        n = np.linalg.norm(p, axis=1)
        i0, i1, i2 = np.argmax(x), np.argmin(x), np.argmin(n)
        self.args = (x, y, p[i0], p[i1])
        # iterate
        self.model = scipy.optimize.minimize(self._error, p[i2], args=self.args, method='Nelder-Mead', tol=self.stop_error)

    def _error(self, p2, x, y, p0, p1):
        return distance_to_line(x, y, p0, p2) + distance_to_line(x, y, p1, p2)
    
    def score(self, x, y):
        return 1 / (1 + self._error(self.model.x, self.args[0], self.args[1], self.args[2], self.args[3]))
    

class DumpsterFitter:

    def __init__(self, min_points=10, fit_score_gate=0.5):
        self.min_points = min_points
        self.ransac = RANSACRegressor(min_samples=min_points)
        self.fit_score_gate = fit_score_gate
        self.fit_score_ = None

    def __call__(self, lidar_points, scr):

        if len(lidar_points) < self.min_points: return None

        X, y = lidar_points[:, 1].reshape((-1, 1)), lidar_points[:, 0].flatten()

        self.ransac.fit(X, y)
        self.fit_score_ = round(self.ransac.score(X[self.ransac.inlier_mask_],
                y[self.ransac.inlier_mask_]), 2)