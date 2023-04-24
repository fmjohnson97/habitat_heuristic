import habitat_sim
import numpy as np

DIFFICULTY_THRESHOLDS = {None: [-np.inf, np.inf],
                         'easy': [1.5, 3],
                         'medium': [3, 5],
                         'hard': [5, 10]}


def getShortestPath(sim, start=None, end=None):
    if start is None:
        start = sim.pathfinder.get_random_navigable_point()
    if end is None:
        end = sim.pathfinder.get_random_navigable_point()

    path = habitat_sim.ShortestPath()
    path.requested_start = start
    path.requested_end = end
    found_path = sim.pathfinder.find_path(path)
    if found_path:
        return {'path': np.stack(path.points),
                'distance': path.geodesic_distance,
                'start': start,
                'end': end}
    else:
        return {'path': None,
                'distance': np.inf,
                'start': start,
                'end': end}


def generateTrajectoryGraph(sim, difficulty=None):
    diff_thresh = DIFFICULTY_THRESHOLDS[difficulty]
    path_data = getShortestPath(sim)
    while path_data['path'] is None:
        path_data = getShortestPath(sim)
