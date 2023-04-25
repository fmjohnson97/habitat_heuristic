import habitat_sim
import numpy as np

from graph import Graph

DIFFICULTY_THRESHOLDS = {None: [-np.inf, np.inf],
                         'easy': [1.5, 3],
                         'medium': [3, 5],
                         'hard': [5, 10]}

def computerForwardAction(start_loc, end_loc, base_movement=0.25):
    distance = ((end_loc[2]-start_loc[2])**2 + (end_loc[0]-start_loc[0])**2)**0.5
    return ['move_forward'] * distance//base_movement
    #TODO: check to see if this is close enough? I think just has to be within 1m to count so this is actually too much?


def computeTurnAction(start_loc, end_loc, base_angle=15):
    ang_diff0 = np.arctan2((end_loc[2]-start_loc[2])/(end_loc[0]-start_loc[0]))
    ang_diff360 = (360 - ang_diff0) % 360
    if abs(ang_diff360) >= base_angle and abs(ang_diff0) >= base_angle:
        if ang_diff0 < ang_diff360:
            times = ang_diff0 // base_angle
            return ['turn_left'] * times
        else:
            times = ang_diff360 // base_angle
            return ['turn_right'] * times
    return []


def generateTrajectoryGraph(sim, difficulty=None):
    diff_thresh = DIFFICULTY_THRESHOLDS[difficulty]
    path_data = getShortestPath(sim)
    counter = 0
    while path_data['path'] is None or path_data['distance'] < diff_thresh[0] or path_data['distance'] > diff_thresh[1]:
        path_data = getShortestPath(sim)
        counter += 1
        if counter > 100:
            print('Generate Trajectory Graph timed out with path generation')
            exit(-1)

    trajGraph = Graph()
    actions = getActionsForPath(path_data['path'])
    start_obs = sim.step('stop')

    for act in actions:



def getActionsForPath(path):
    actions = []
    for p in range(len(path)-2):
        actions.extend(computeTurnAction(path[p], path[p+1]))
        actions.extend(computerForwardAction(path[p], path[p+1]))
    return actions


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
