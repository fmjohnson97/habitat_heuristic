import os
from glob import glob

import cv2
import habitat_sim
import numpy as np
from habitat_sim.nav import GreedyGeodesicFollower

from graph import Graph
from habitat_utils import saveOBS

DIFFICULTY_THRESHOLDS = {None: [-np.inf, np.inf],
                         'easy': [1.5, 3],
                         'medium': [3, 5],
                         'hard': [5, 10]}


def computerForwardAction(start_loc, end_loc, base_movement=0.25):
    distance = ((end_loc[2] - start_loc[2]) ** 2 + (end_loc[0] - start_loc[0]) ** 2) ** 0.5
    return ['move_forward'] * int(np.ceil(distance // base_movement))
    # TODO: check to see if this is close enough? I think just has to be within 1m to count so this is actually too much?


def computeTurnAction(start_loc, end_loc, base_angle=15):
    # breakpoint()
    # TODO: maybe check if 360-angle would be shorter?
    ang_diff0 = np.arctan2((end_loc[2] - start_loc[2]), (end_loc[0] - start_loc[0])) * 180 / np.pi
    # if ang_diff0 < 0:
    #     ang_diff360 = ang_diff0 + 360
    # else:
    #     ang_diff360 = ang_diff0 - 360
    if abs(ang_diff0) >= base_angle:  # and abs(ang_diff360) >= base_angle:
        times = ang_diff0 // base_angle
        print(times, ang_diff0, np.arctan2((end_loc[0] - start_loc[0]), (end_loc[2] - start_loc[2])) * 180 / np.pi)
        if ang_diff0 < 0:  # ang_diff360:
            # times = ang_diff0 // base_angle
            return ['turn_left'] * int(abs(times))
        else:
            # times = ang_diff360 // base_angle
            return ['turn_right'] * int(abs(times))
    return []


def generateTrajectoryGraph(sim, sim_settings, difficulty=None, save=False):
    diff_thresh = DIFFICULTY_THRESHOLDS[difficulty]
    path_data = getShortestPath(sim)
    counter = 0

    while path_data['path'] is None or path_data['distance'] < diff_thresh[0] or path_data['distance'] > diff_thresh[1]:
        path_data = getShortestPath(sim)
        counter += 1
        if counter > 100:
            print('Generate Trajectory Graph timed out with path generation')
            exit(-1)

    folder_path = '/home/faith/GitRepos/habitat_heuristic/test_env_graphs/' + str(
        difficulty) + '/' + sim.curr_scene_name + '/'
    if os.path.exists(folder_path):
        existing_traj_folders = glob(folder_path + '*')
        existing_traj_folders.sort()
        most_recent_traj = int(existing_traj_folders[-1].split('/')[-1][4:])
        folder_path += 'traj' + str(most_recent_traj + 1) + '/'
    else:
        os.mkdir(folder_path)
        folder_path += 'traj0/'
    os.mkdir(folder_path)

    agent = sim.initialize_agent(sim_settings["default_agent"])
    agent_state = habitat_sim.AgentState()
    agent_state.position = path_data['start']
    agent.set_state(agent_state)
    agent_state = agent.get_state()
    agent_location = agent_state.position
    agent_orient = agent_state.rotation

    # breakpoint()
    try:
        follower = GreedyGeodesicFollower(sim.pathfinder, agent, goal_radius=1.99 * agent.agent_config.action_space[
            'move_forward'].actuation.amount)
        follower.find_path(path_data['end'])
        trajGraph = Graph(goal_location=path_data['end'])
        # actions = orientTowardsFirstPoint(path_data['path'][0], path_data['path'][1])
        actions = follower.find_path(path_data['end'])  # getActionsForPath(path_data['path'], agent_orient)
    except GreedyGeodesicFollower:
        return False  # to signify the graph couldn't be created
    breakpoint()

    # TODO: stop action not working! maybe fix???
    start_obs = sim.get_sensor_observations()
    img_names = saveOBS(start_obs, folder_path, agent_location)
    trajGraph.addNode(img_names[0], agent_location, agent_orient, img_names[1])

    for i, act in enumerate(actions):
        if act is not None:
            obs = sim.step(act)
            agent_state = agent.get_state()
            agent_loc = agent_state.position
            agent_orient = agent_state.rotation
            cv2.imshow('', obs['color_sensor'])
            cv2.waitKey(400)
            if 'forward' in act:
                img_names = saveOBS(obs, folder_path, agent_location)
                trajGraph.addNode(img_names[0], agent_loc, agent_orient, img_names[1])
            # TODO: get rid of print when we're happy with this
            print(((agent_loc[0] - path_data['end'][0]) ** 2 + (agent_loc[2] - path_data['end'][2]) ** 2) ** 0.5)
            if ((agent_loc[0] - path_data['end'][0]) ** 2 + (agent_loc[2] - path_data['end'][2]) ** 2) ** 0.5 <= \
                    1 * agent.agent_config.action_space['move_forward'].actuation.amount:
                end_act_ind = i
                break

    actions = actions[:end_act_ind]

    # TODO: assuming goal image is last one collected from the node, but could need rotations once get there
    # should make a case where that's true sometimes and add more images/actions to the graph
    # ie. just change the rgb/depth images associated with the last node/goal node

    breakpoint()  # TODO: check that it actually reaches the goal
    # TODO: check that the end point/goal point is actually the end of path
    if save:
        trajGraph.save(folder_path + 'trajGraph.json')

    return trajGraph  # to signify that the graph was created successfully


def getActionsForPath(path, agent_orient=None):
    actions = []
    if agent_orient:
        agent_orient  # TODO: finish this!!!!
        actions.extend(computeTurnAction(path[0], path[0]))
    # TODO: orient the robot towards the first point then just append the rest of this
    for p in range(len(path) - 1):
        actions.extend(computeTurnAction(path[p], path[p + 1]))
        actions.extend(computerForwardAction(path[p], path[p + 1]))
    return actions


def getShortestPath(sim, start=None, end=None):
    if start is None:
        start = sim.pathfinder.get_random_navigable_point()
        start[1] = sim.config.agents[0].height
    if end is None:
        end = sim.pathfinder.get_random_navigable_point()
        end[1] = sim.config.agents[0].height

    path = habitat_sim.ShortestPath()
    path.requested_start = start
    path.requested_end = end
    found_path = sim.pathfinder.find_path(path)
    if found_path:
        return {'path': np.stack(path.points),
                'path_object': path,
                'distance': path.geodesic_distance,
                'start': start,
                'end': end}
    else:
        return {'path': None,
                'path_object': path,
                'distance': np.inf,
                'start': start,
                'end': end}


def orientTowardsFirstPoint(start, point):
    breakpoint()
    # eye = start
    # look = point
    up = np.array([0, 1, 0]).T
    zhat = start - point
