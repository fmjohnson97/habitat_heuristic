import habitat_sim
import numpy as np

from graph import Graph
from habitat_utils import saveOBS

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


def generateTrajectoryGraph(sim, sim_settings, difficulty=None):
    diff_thresh = DIFFICULTY_THRESHOLDS[difficulty]
    path_data = getShortestPath(sim)
    counter = 0
    breakpoint()
    while path_data['path'] is None or path_data['distance'] < diff_thresh[0] or path_data['distance'] > diff_thresh[1]:
        path_data = getShortestPath(sim)
        counter += 1
        if counter > 100:
            print('Generate Trajectory Graph timed out with path generation')
            exit(-1)

    breakpoint()
    #TODO: make folder path variable based on the env name and difficulty
    folder_path = None

    trajGraph = Graph()
    actions = getActionsForPath(path_data['path'])

    agent = sim.initialize_agent(sim_settings["default_agent"])
    agent_state = habitat_sim.AgentState()
    agent_state.position = np.array([path_data['start'][0], sim_settings['sensor_height'], path_data['start'][2]])
    agent.set_state(agent_state)
    agent_state = agent.get_state()
    agent_loc = agent_state.position
    agent_orient =  agent_state.rotation

    start_obs = sim.step('stop')
    img_names=saveOBS(start_obs,folder_path)
    trajGraph.addNode(img_names[0], path_data['start'], agent_orient, img_names[1])

    for act in actions:
        #TODO: do we have to physically move the agent or does it move on its own with sim.step???
        obs = sim.step(act)
        if 'forward' in act:
            breakpoint()
            img_names = saveOBS(obs, folder_path)
            location = None
            orientation = None
            trajGraph.addNode(img_names[0], location, orientation, img_names[1])


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
