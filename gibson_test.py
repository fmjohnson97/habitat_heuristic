import argparse
import random

import cv2
import habitat_sim
import torch

from habitat_utils import get_split_files, make_cfg
from models import Worker, MidManager
from graph_utils import generateTrajectoryGraph


def getArgs():
    parser = argparse.ArgumentParser()

    # Superglue arguments
    parser.add_argument('--nms_radius', type=int, default=4,
                        help='SuperPoint Non Maximum Suppression (NMS) radius (Must be positive)')
    parser.add_argument('--sinkhorn_iterations', type=int, default=20,
                        help='Number of Sinkhorn iterations performed by SuperGlue')
    parser.add_argument('--match_threshold', type=float, default=0.2, help='SuperGlue match threshold')
    parser.add_argument('--superglue', choices={'indoor', 'outdoor'}, default='indoor', help='SuperGlue weights')
    parser.add_argument('--max_keypoints', type=int, default=1024,
                        help='Maximum number of keypoints detected by Superpoint (\'-1\' keeps all keypoints)')
    parser.add_argument('--keypoint_threshold', type=float, default=0.01,
                        help='SuperPoint keypoint detector confidence threshold')
    parser.add_argument('--x_size', type=int, default=640, help='x dimension of the image')
    parser.add_argument('--y_size', type=int, default=480, help='y dimension of the image')

    # Habitat arguments
    parser.add_argument('--meters_per_pixel', type=float, default=0.1,
                        help='meters per pixel for topdown pathfinder map')
    parser.add_argument('--max_search_radius', type=float, default=2,
                        help='max search radius for pathfinding distance to a point')

    # MidManager arguments
    parser.add_argument('--cluster_dist_thresh', type=float, default=20,
                        help='distance threshold for the agglomorative clustering')

    return parser.parse_args()


if __name__ == '__main__':
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    args = getArgs()

    # set up the environment
    test_scene_files = get_split_files(split='test', env='gibson')
    test_scene = random.choice(list(test_scene_files['all']))
    sim_settings = {"scene_dataset_config_file": "default",
                    "scene": '/home/faith/GitRepos/habitat/gibson/' + test_scene,
                    "default_agent": 0,
                    "sensor_height": 1.5,
                    "width": 640,
                    "height": 480,
                    "hfov": 120,
                    "zfar": 1000.0,
                    "color_sensor": True,
                    "semantic_sensor": False,
                    "depth_sensor": True,
                    "ortho_rgba_sensor": False,
                    "ortho_depth_sensor": False,
                    "ortho_semantic_sensor": False,
                    "fisheye_rgba_sensor": False,
                    "fisheye_depth_sensor": False,
                    "fisheye_semantic_sensor": False,
                    "equirect_rgba_sensor": False,
                    "equirect_depth_sensor": False,
                    "equirect_semantic_sensor": False,
                    "seed": 1,
                    "physics_config_file": "data/default.physics_config.json",
                    "enable_physics": False}
    habitat_cfg = make_cfg(sim_settings)
    sim = habitat_sim.Simulator(habitat_cfg)

    #TODO: try to read traj graph; else do this
    generateTrajectoryGraph(sim, sim_settings, difficulty=None)

    # get sim area stats and topdown map
    print("Navmesh area=", sim.pathfinder.navigable_area)
    bounds = sim.pathfinder.get_bounds()
    print('Bounds=', bounds)
    # tried the topdown map stuff from the tutorial, but it didn't return anything but Falses

    # initialize our hierarchical agents
    midMan = MidManager(args, device)
    # goalimg = midMan.step(testimg)
    worker = Worker(args, device)
    # moves = worker.step(testimg, goalimg)

    # initialize a sim agent to represent the goal
    goal = sim.initialize_agent(sim_settings["default_agent"])
    # Set agent state
    goal_state = habitat_sim.AgentState()
    goal_position = sim.pathfinder.get_random_navigable_point()
    goal_position[1] = sim_settings['sensor_height']
    goal_state.position = goal_position  # in world space
    goal.set_state(goal_state)
    obs = sim.step('turn_left')
    goal_image = obs['color_sensor']
    goal_depth = obs['depth_sensor']

    # initialize a sim agent to move for the hierarchical agents
    agent = sim.initialize_agent(sim_settings["default_agent"])
    # Set agent state
    agent_state = habitat_sim.AgentState()
    agent_position = sim.pathfinder.get_random_navigable_point()
    # TODO: put these in when it's actual test time that way we can guarantee they're not on top of each other
    # while abs(goal_position[0]-agent_position[0])<1 or abs(goal_position[2]-agent_position[2])<1:
    #     agent_position = sim.pathfinder.get_random_navigable_point()
    goal_position[1] = sim_settings['sensor_height']
    goal_state.position = goal_position  # in world space
    agent.set_state(agent_state)
    obs = sim.step('turn_left')
    obs_rgb = obs['color_sensor']
    obs_depth = obs['depth_sensor']

    # get subgoal image from middle manager
    subgoalimg = midMan.step(obs_rgb)
    moves = []
    while 'stop' not in moves:
        moves = worker.step(obs_rgb, subgoalimg)
        print(moves)
        for action in moves:
            if action != 'stop':
                obs = sim.step(action)
        obs_rgb = obs['color_sensor']
        cv2.imshow('', obs_rgb)
        cv2.waitKey(500)

    # TODO: generate a random goal image

    # Get agent state
    # agent_state = agent.get_state()
    # print("agent_state: position", agent_state.position, "rotation", agent_state.rotation)
    # obtain the default, discrete actions that an agent can perform
    # default action space contains 3 actions: move_forward, turn_left, and turn_right
    # action_names = list(habitat_cfg.agents[sim_settings["default_agent"]].action_space.keys())
    # print("Discrete action space: ", action_names)

    # execute 10 random actions and view the corresponding frames
    # max_frames = 10
    # for mf in range(max_frames):
    #     action = random.choice(action_names)
    #     print('Action:',action)
    #     obs = sim.step(action)
    #     rgb = obs['color_sensor']
    #     depth = obs['depth_sensor']
    #     cv2.imshow('',rgb)
    #     cv2.waitKey(500)

    # get random navigable point and check can navigate to there
    # pathfinder_seed=1
    # sim.pathfinder.seed(pathfinder_seed)
    # nav_point = sim.pathfinder.get_random_navigable_point()
    # print("random point:",nav_point, "is navigable?", sim.pathfinder.is_navigable(nav_point))
    # print("distance to random point:", sim.pathfinder.distance_to_closest_obstacle(nav_point, args.max_search_radius))

    # get shortest path between 2 points
    print('finished')
